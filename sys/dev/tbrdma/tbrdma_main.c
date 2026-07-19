/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tbrdma_main.c - Thunderbolt RDMA ib_device provider, module + registration.
 *
 * Registers one ib_device per Thunderbolt controller that thunderbolt.ko
 * offers through the native tb_rdma interface (see
 * <dev/thunderbolt/tb_rdma_if.h>).  This is the LinuxKPI/OFED half of the
 * mlx4-style split; it depends on thunderbolt.ko (transport), ibcore.ko
 * (verbs core), and linuxkpi.ko.
 *
 * M1a assembles the ib_device and wires the query_* verbs from
 * tbrdma_verbs.c; QP/CQ/MR come in later milestones.  The assembly sequence
 * (ib_alloc_device -> fill fields -> INIT_RDMA_OBJ_SIZE -> assign verb method
 * pointers -> ib_register_device) mirrors mlx4_ib_add() field for field.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>

#include <sys/sysctl.h>

#include <rdma/ib_verbs.h>
#include <rdma/ib_user_verbs.h>

#include "tbrdma.h"

static LIST_HEAD(tbrdma_dev_list);
static DEFINE_MUTEX(tbrdma_dev_lock);

struct tbrdma_link {
	struct list_head	link;
	struct tbrdma_dev	*tdev;
	struct tb_rdma_dev	*tbd;
};

static void
tbrdma_fill_device(struct tbrdma_dev *tdev)
{
	struct ib_device *ibdev = &tdev->ib_dev;

	strlcpy(ibdev->name, "tbrdma%d", IB_DEVICE_NAME_MAX);
	ibdev->owner = THIS_MODULE;
	ibdev->node_type = RDMA_NODE_IB_CA;
	ibdev->phys_port_cnt = 1;
	ibdev->num_comp_vectors = 1;
	ibdev->dma_device = &linux_root_device;	/* real parent wired in M2 */
	ibdev->uverbs_abi_ver = 1;

	/* Identity: derive a stable node GUID from the router UUID. */
	memcpy(&ibdev->node_guid, tdev->gid, sizeof(ibdev->node_guid));

	ibdev->uverbs_cmd_mask =
	    (1ull << IB_USER_VERBS_CMD_GET_CONTEXT)	|
	    (1ull << IB_USER_VERBS_CMD_QUERY_DEVICE)	|
	    (1ull << IB_USER_VERBS_CMD_QUERY_PORT)	|
	    (1ull << IB_USER_VERBS_CMD_ALLOC_PD)	|
	    (1ull << IB_USER_VERBS_CMD_DEALLOC_PD)	|
	    (1ull << IB_USER_VERBS_CMD_REG_MR)		|
	    (1ull << IB_USER_VERBS_CMD_DEREG_MR)	|
	    (1ull << IB_USER_VERBS_CMD_CREATE_CQ)	|
	    (1ull << IB_USER_VERBS_CMD_DESTROY_CQ)	|
	    (1ull << IB_USER_VERBS_CMD_CREATE_QP)	|
	    (1ull << IB_USER_VERBS_CMD_MODIFY_QP)	|
	    (1ull << IB_USER_VERBS_CMD_DESTROY_QP);

	/*
	 * driver_id + core object sizes.  Not INIT_IB_DEVICE_OPS (that macro
	 * also demands ah/cq/srq driver subclasses we do not have in M1a); set
	 * the two we actually subclass by hand.
	 */
	ibdev->ops.driver_id = RDMA_DRIVER_UNKNOWN;
	ibdev->ops INIT_RDMA_OBJ_SIZE(ib_pd, tbrdma_pd, ibpd);
	ibdev->ops INIT_RDMA_OBJ_SIZE(ib_ucontext, tbrdma_ucontext, ibucontext);
	ibdev->ops INIT_RDMA_OBJ_SIZE(ib_cq, tbrdma_cq, ibcq);

	/* Verb method pointers are direct fields of ib_device in this tree. */
	ibdev->query_device = tbrdma_query_device;
	ibdev->query_port = tbrdma_query_port;
	ibdev->get_link_layer = tbrdma_get_link_layer;
	ibdev->get_netdev = tbrdma_get_netdev;
	ibdev->query_gid = tbrdma_query_gid;
	ibdev->add_gid = tbrdma_add_gid;
	ibdev->del_gid = tbrdma_del_gid;
	ibdev->query_pkey = tbrdma_query_pkey;
	ibdev->get_port_immutable = tbrdma_get_port_immutable;
	ibdev->alloc_ucontext = tbrdma_alloc_ucontext;
	ibdev->dealloc_ucontext = tbrdma_dealloc_ucontext;
	ibdev->alloc_pd = tbrdma_alloc_pd;
	ibdev->dealloc_pd = tbrdma_dealloc_pd;

	/* Mandatory-table stubs (real data path in M1b/M2). */
	ibdev->create_qp = tbrdma_create_qp;
	ibdev->modify_qp = tbrdma_modify_qp;
	ibdev->destroy_qp = tbrdma_destroy_qp;
	ibdev->post_send = tbrdma_post_send;
	ibdev->post_recv = tbrdma_post_recv;
	ibdev->create_cq = tbrdma_create_cq;
	ibdev->destroy_cq = tbrdma_destroy_cq;
	ibdev->poll_cq = tbrdma_poll_cq;
	ibdev->req_notify_cq = tbrdma_req_notify_cq;
	ibdev->get_dma_mr = tbrdma_get_dma_mr;
	ibdev->reg_user_mr = tbrdma_reg_user_mr;
	ibdev->dereg_mr = tbrdma_dereg_mr;
	ibdev->mmap = tbrdma_mmap;
}

static void
tbrdma_add(struct tb_rdma_dev *tbd)
{
	struct tbrdma_dev *tdev;
	struct tbrdma_link *l;
	device_t dev;
	int err;

	dev = tb_rdma_get_dev(tbd);

	tdev = (struct tbrdma_dev *)ib_alloc_device(sizeof(*tdev));
	if (tdev == NULL) {
		device_printf(dev, "tbrdma: ib_alloc_device failed\n");
		return;
	}
	tdev->tbd = tbd;
	tb_rdma_get_uuid(tbd, tdev->gid);
	mutex_init(&tdev->res_lock);
	tdev->ring_map = 0;

	tbrdma_fill_device(tdev);

	err = ib_register_device(&tdev->ib_dev, NULL);
	if (err != 0) {
		device_printf(dev, "tbrdma: ib_register_device failed: %d\n",
		    err);
		ib_dealloc_device(&tdev->ib_dev);
		return;
	}

	l = kzalloc(sizeof(*l), GFP_KERNEL);
	if (l == NULL) {
		ib_unregister_device(&tdev->ib_dev);
		ib_dealloc_device(&tdev->ib_dev);
		return;
	}
	l->tdev = tdev;
	l->tbd = tbd;
	mutex_lock(&tbrdma_dev_lock);
	list_add_tail(&l->link, &tbrdma_dev_list);
	mutex_unlock(&tbrdma_dev_lock);

	device_printf(dev, "tbrdma: registered %s\n", tdev->ib_dev.name);
}

static void
tbrdma_remove(struct tb_rdma_dev *tbd)
{
	struct tbrdma_link *l, *found = NULL;

	mutex_lock(&tbrdma_dev_lock);
	list_for_each_entry(l, &tbrdma_dev_list, link) {
		if (l->tbd == tbd) {
			found = l;
			list_del(&l->link);
			break;
		}
	}
	mutex_unlock(&tbrdma_dev_lock);

	if (found == NULL)
		return;

	ib_unregister_device(&found->tdev->ib_dev);
	ib_dealloc_device(&found->tdev->ib_dev);
	kfree(found);
}

static struct tb_rdma_interface tbrdma_interface = {
	.add	= tbrdma_add,
	.remove	= tbrdma_remove,
};

/*
 * White-box self-test trigger: `sysctl dev.tbrdma.selftest=1` drives a QP to
 * RTS on the first registered device.  Proves the M1b control plane with no
 * userspace provider.
 */
static int
tbrdma_sysctl_selftest(SYSCTL_HANDLER_ARGS)
{
	struct tbrdma_link *l;
	struct tbrdma_dev *tdev = NULL;
	int val = 0, error, e;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val == 0)
		return (0);

	mutex_lock(&tbrdma_dev_lock);
	l = list_first_entry_or_null(&tbrdma_dev_list, struct tbrdma_link, link);
	if (l != NULL)
		tdev = l->tdev;
	mutex_unlock(&tbrdma_dev_lock);
	if (tdev == NULL)
		return (ENXIO);

	e = tbrdma_selftest(tdev);
	return (e < 0 ? -e : e);
}

static SYSCTL_NODE(_dev, OID_AUTO, tbrdma, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Thunderbolt RDMA provider");
SYSCTL_PROC(_dev_tbrdma, OID_AUTO, selftest,
    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, 0, 0,
    tbrdma_sysctl_selftest, "I",
    "write 1: run the QP RESET->INIT->RTR->RTS control-plane self-test");

static int __init
tbrdma_init(void)
{
	return (tb_rdma_register_interface(&tbrdma_interface));
}

static void __exit
tbrdma_cleanup(void)
{
	tb_rdma_unregister_interface(&tbrdma_interface);
}

module_init_order(tbrdma_init, SI_ORDER_SEVENTH);
module_exit_order(tbrdma_cleanup, SI_ORDER_SEVENTH);

static int
tbrdma_evhand(module_t mod, int event, void *arg)
{
	return (0);
}

static moduledata_t tbrdma_mod = {
	.name = "tbrdma",
	.evhand = tbrdma_evhand,
};

DECLARE_MODULE(tbrdma, tbrdma_mod, SI_SUB_LAST, SI_ORDER_ANY);
MODULE_DEPEND(tbrdma, thunderbolt, 1, 1, 1);
MODULE_DEPEND(tbrdma, ibcore, 1, 1, 1);
MODULE_DEPEND(tbrdma, linuxkpi, 1, 1, 1);
MODULE_VERSION(tbrdma, 1);

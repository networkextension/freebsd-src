/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tbrdma_verbs.c - the M1a verbs surface: identity/query only.
 *
 * Fixed capabilities, copied from Apple's AppleThunderboltRDMA + the hardware
 * facts our RE pinned (see rdma-demo/FREEBSD-PROVIDER.md): UC-only, send/recv
 * only, 4 KiB frame granule, <= 10 QP, <= 4095 WR, single SGE, GID = the
 * paired Thunderbolt-IP address (a placeholder here until create_qp/tbt0
 * pairing lands in M1b/M2).  No QP/CQ/MR yet - this milestone only has to make
 * the device enumerate (ibv_devinfo) with a PORT_ACTIVE, MTU-4096, one-GID,
 * pkey-0xffff port.
 *
 * Verb shapes cross-checked field-for-field against
 * sys/dev/mlx4/mlx4_ib/mlx4_ib_main.c on this tree.
 */
#include <sys/param.h>
#include <sys/proc.h>
#include <vm/vm.h>
#include <vm/vm_page.h>
#include <vm/vm_map.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#include "tbrdma.h"

int
tbrdma_query_device(struct ib_device *ibdev, struct ib_device_attr *props,
    struct ib_udata *uhw)
{
	struct tbrdma_dev *tdev = to_tdev(ibdev);

	if (uhw->inlen || uhw->outlen)
		return (-EINVAL);

	memset(props, 0, sizeof(*props));

	props->sys_image_guid = ibdev->node_guid;
	props->vendor_id = 0x8086;		/* Intel NHI */
	props->vendor_part_id = 0;
	props->hw_ver = 0;
	props->max_mr_size = ~0ull;
	props->page_size_cap = PAGE_SIZE;
	props->max_qp = TBRDMA_MAX_QP;
	props->max_qp_wr = TBRDMA_MAX_QP_WR;
	props->max_sge = TBRDMA_MAX_SGE;
	props->max_sge_rd = 0;
	props->max_cq = TBRDMA_MAX_QP;
	props->max_cqe = TBRDMA_MAX_QP_WR;
	props->max_mr = 4096;
	props->max_pd = 4096;
	props->max_qp_rd_atom = 0;		/* no one-sided */
	props->max_qp_init_rd_atom = 0;
	props->max_ah = 0;
	props->atomic_cap = IB_ATOMIC_NONE;

	(void)tdev;
	return (0);
}

int
tbrdma_query_port(struct ib_device *ibdev, u8 port,
    struct ib_port_attr *props)
{

	if (port != 1)
		return (-EINVAL);

	/*
	 * ib_query_port() zeroes *props for us in this tree, but be explicit -
	 * some callers pass a stack copy through get_port_immutable.
	 */
	memset(props, 0, sizeof(*props));

	props->state = IB_PORT_ACTIVE;
	props->phys_state = IB_PORT_PHYS_STATE_LINK_UP;
	props->max_mtu = IB_MTU_4096;
	props->active_mtu = IB_MTU_4096;
	props->gid_tbl_len = 1;
	props->pkey_tbl_len = 1;
	props->max_msg_sz = TBRDMA_MAX_MSG_SZ;
	props->port_cap_flags = 0;
	props->active_width = IB_WIDTH_1X;
	props->active_speed = 1;		/* SDR-equivalent placeholder */

	return (0);
}

enum rdma_link_layer
tbrdma_get_link_layer(struct ib_device *ibdev, u8 port)
{

	/* GID = TB-IP address -> RoCE (Ethernet) link layer, like Apple. */
	return (IB_LINK_LAYER_ETHERNET);
}

int
tbrdma_query_gid(struct ib_device *ibdev, u8 port, int index,
    union ib_gid *gid)
{
	struct tbrdma_dev *tdev = to_tdev(ibdev);

	if (port != 1 || index != 0)
		return (-EINVAL);

	memcpy(gid->raw, tdev->gid, sizeof(gid->raw));
	return (0);
}

int
tbrdma_add_gid(const struct ib_gid_attr *attr, void **context)
{

	/*
	 * RoCE GID management may push a netdev-derived GID once tbt0 is
	 * paired (M2).  Accept and store nothing - the address plumbing is
	 * not wired yet.
	 */
	*context = NULL;
	return (0);
}

int
tbrdma_del_gid(const struct ib_gid_attr *attr, void **context)
{

	return (0);
}

int
tbrdma_query_pkey(struct ib_device *ibdev, u8 port, u16 index, u16 *pkey)
{

	if (port != 1 || index != 0)
		return (-EINVAL);

	*pkey = 0xffff;				/* default partition */
	return (0);
}

int
tbrdma_get_port_immutable(struct ib_device *ibdev, u8 port_num,
    struct ib_port_immutable *immutable)
{
	struct ib_port_attr attr;
	int err;

	err = tbrdma_query_port(ibdev, port_num, &attr);
	if (err)
		return (err);

	immutable->pkey_tbl_len = attr.pkey_tbl_len;
	immutable->gid_tbl_len = attr.gid_tbl_len;
	immutable->core_cap_flags = RDMA_CORE_PORT_IBA_ROCE;
	immutable->max_mad_size = 0;		/* no MAD agent */

	return (0);
}

int
tbrdma_alloc_ucontext(struct ib_ucontext *uctx, struct ib_udata *udata)
{

	/*
	 * Core-allocated (INIT_RDMA_OBJ_SIZE): the struct is already carved
	 * out of struct tbrdma_ucontext, so there is nothing to allocate.  The
	 * userspace mmap/doorbell state arrives in M2.
	 */
	return (0);
}

void
tbrdma_dealloc_ucontext(struct ib_ucontext *ibcontext)
{

}

int
tbrdma_alloc_pd(struct ib_pd *ibpd, struct ib_udata *udata)
{

	/* Core-allocated; PD is a pure software handle for M1a. */
	return (0);
}

void
tbrdma_dealloc_pd(struct ib_pd *ibpd, struct ib_udata *udata)
{

}

/*
 * M1b: real QP/CQ control plane (data path = M2).  create_cq is
 * core-allocated (fill the embedded ib_cq, don't kzalloc); create_qp is
 * driver-allocated (kzalloc, return &qp->ibqp).  The QP engine lives in
 * tbrdma_qp.c.
 */
struct ib_qp *
tbrdma_create_qp(struct ib_pd *pd, struct ib_qp_init_attr *attr,
    struct ib_udata *udata)
{
	struct tbrdma_dev *tdev = to_tdev(pd->device);
	struct tbrdma_qp *qp;

	/* Apple's stack is UC-only; refuse anything else. */
	if (attr->qp_type != IB_QPT_UC)
		return (ERR_PTR(-EINVAL));

	qp = tbrdma_qp_alloc(tdev, attr->cap.max_send_wr,
	    attr->cap.max_recv_wr);
	if (IS_ERR(qp))
		return (ERR_CAST(qp));

	/*
	 * Userspace QP (udata present): rings will be handed off for
	 * kernel-bypass, and mmap() looks the QP up via the ucontext.
	 */
	if (udata != NULL && pd->uobject != NULL) {
		qp->userspace = true;
		to_tucontext(pd->uobject->context)->qp = qp;
	}

	return (&qp->ibqp);
}

int
tbrdma_modify_qp(struct ib_qp *ibqp, struct ib_qp_attr *attr, int attr_mask,
    struct ib_udata *udata)
{
	struct tbrdma_qp *qp = to_tqp(ibqp);
	enum ib_qp_state new_state;
	u32 dest_qpn;
	const u8 *dgid;

	new_state = (attr_mask & IB_QP_STATE) ? attr->qp_state : qp->state;
	dest_qpn = (attr_mask & IB_QP_DEST_QPN) ? attr->dest_qp_num : 0;
	dgid = (attr_mask & IB_QP_AV) ? attr->ah_attr.grh.dgid.raw : NULL;

	return (tbrdma_qp_modify(qp, new_state, dest_qpn, dgid));
}

int
tbrdma_destroy_qp(struct ib_qp *ibqp, struct ib_udata *udata)
{

	tbrdma_qp_free(to_tqp(ibqp));
	return (0);
}

int
tbrdma_post_send(struct ib_qp *qp, const struct ib_send_wr *wr,
    const struct ib_send_wr **bad_wr)
{

	*bad_wr = wr;
	return (-EOPNOTSUPP);
}

int
tbrdma_post_recv(struct ib_qp *qp, const struct ib_recv_wr *wr,
    const struct ib_recv_wr **bad_wr)
{

	*bad_wr = wr;
	return (-EOPNOTSUPP);
}

int
tbrdma_create_cq(struct ib_cq *ibcq, const struct ib_cq_init_attr *attr,
    struct ib_udata *udata)
{

	/*
	 * Core-allocated (struct tbrdma_cq carved around ibcq).  M1b has no
	 * completion ring yet (data path = M2); just record the depth so
	 * ibv_create_cq succeeds and the QP can reference it.
	 */
	ibcq->cqe = attr->cqe;
	return (0);
}

void
tbrdma_destroy_cq(struct ib_cq *ibcq, struct ib_udata *udata)
{

	/* Core-allocated: nothing to free. */
}

int
tbrdma_poll_cq(struct ib_cq *cq, int num_entries, struct ib_wc *wc)
{

	return (-EOPNOTSUPP);
}

int
tbrdma_req_notify_cq(struct ib_cq *cq, enum ib_cq_notify_flags flags)
{

	return (-EOPNOTSUPP);
}

struct ib_mr *
tbrdma_get_dma_mr(struct ib_pd *pd, int acc)
{

	/* Kernel DMA MR (no userspace buffer) - not used by the UC datapath. */
	return (ERR_PTR(-EOPNOTSUPP));
}

/*
 * M2b: pin a userspace buffer and derive a region_id (the NHI bus address of
 * the MR start).  This box's NHI has no active IOMMU, so a pinned page's
 * physical address IS the bus address the DMA engine uses - so we hold the
 * user pages natively (vm_fault_quick_hold_pages) and read their physical
 * addresses directly, rather than ib_umem_get(), whose internal
 * linux_dma_map_sg against our (non-PCI, dma_priv-less) ib_device faults.
 * Under an SMMU this would instead bus_dmamap_load on the nhi parent_dmat (R2).
 */
struct ib_mr *
tbrdma_reg_user_mr(struct ib_pd *pd, u64 start, u64 length, u64 virt_addr,
    int access, struct ib_udata *udata)
{
	struct tbrdma_dev *tdev = to_tdev(pd->device);
	struct tbrdma_mr *mr;
	vm_map_t map;
	vm_offset_t base;
	int npages, held;

	base = trunc_page((vm_offset_t)start);
	npages = atop(round_page((vm_offset_t)start + length) - base);
	if (npages <= 0)
		return (ERR_PTR(-EINVAL));

	mr = kzalloc(sizeof(*mr), GFP_KERNEL);
	if (mr == NULL)
		return (ERR_PTR(-ENOMEM));

	mr->pages = malloc(npages * sizeof(vm_page_t), M_TEMP,
	    M_WAITOK | M_ZERO);
	mr->length = length;

	map = &curproc->p_vmspace->vm_map;
	held = vm_fault_quick_hold_pages(map, base, ptoa(npages),
	    VM_PROT_READ | VM_PROT_WRITE, mr->pages, npages);
	if (held != npages) {
		if (held > 0)
			vm_page_unhold_pages(mr->pages, held);
		free(mr->pages, M_TEMP);
		kfree(mr);
		return (ERR_PTR(-EFAULT));
	}
	mr->npages = npages;

	mr->dma_base = VM_PAGE_TO_PHYS(mr->pages[0]) +
	    ((vm_offset_t)start & PAGE_MASK);
	mr->region_id = (u32)(mr->dma_base >> PAGE_SHIFT);
	mr->ibmr.lkey = mr->ibmr.rkey = mr->region_id;

	device_printf(tb_rdma_get_dev(tdev->tbd),
	    "tbrdma: reg_mr va=0x%jx len=%ju npages=%d region_id=0x%x "
	    "dma_base=0x%jx\n", (uintmax_t)virt_addr, (uintmax_t)length,
	    npages, mr->region_id, (uintmax_t)mr->dma_base);

	return (&mr->ibmr);
}

int
tbrdma_dereg_mr(struct ib_mr *ibmr, struct ib_udata *udata)
{
	struct tbrdma_mr *mr = to_tmr(ibmr);

	if (mr->pages != NULL) {
		if (mr->npages > 0)
			vm_page_unhold_pages(mr->pages, mr->npages);
		free(mr->pages, M_TEMP);
	}
	kfree(mr);
	return (0);
}

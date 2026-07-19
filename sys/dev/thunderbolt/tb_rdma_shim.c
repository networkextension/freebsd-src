/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tb_rdma_shim.c - native side of the thunderbolt.ko <-> tbrdma.ko boundary.
 *
 * Keeps a registry of live NHI controllers and a single registered consumer
 * (the tbrdma provider).  Modeled on Linux mlx4_core's interface registry:
 *   - controller_ready() adds a controller and, if a consumer is registered,
 *     calls consumer->add() so the provider can create an ib_device for it;
 *   - register_interface() replays add() for every controller already up, so
 *     load order (thunderbolt before or after tbrdma) does not matter;
 *   - controller_gone() / unregister_interface() drive remove() symmetrically.
 *
 * Uses an sx lock, not a mutex: consumer->add() calls ib_register_device()
 * inside tbrdma.ko, which allocates and may sleep.
 */
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/sx.h>
#include <sys/queue.h>
#include <sys/module.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/tb_reg.h>
#include <dev/thunderbolt/tbcfg_reg.h>
#include <dev/thunderbolt/router_var.h>
#include <dev/thunderbolt/tb_rdma_if.h>

/*
 * Hop-table entry format (2 dwords), mirrored from tb_icm.c's SW-CM path.
 * DW0: HopID[10:0] | OutPort[16:11] | Credits[24:17] | Valid[31].
 * DW1: WRR/priority/flow-control flags; bit 25 = egress flow control.
 */
#define	TB_RDMA_NHI_PORT	7
#define	TB_RDMA_HOP_DW1		0x0180c501u		/* egress_fc = 0 (RX) */
#define	TB_RDMA_HOP_DW1_TX	(TB_RDMA_HOP_DW1 | (1u << 25))	/* egress_fc = 1 */
#define	TB_RDMA_HOP_DW0(hopid, outport, credits)			\
	(((hopid) & 0x7ffu) | (((outport) & 0x3fu) << 11) |		\
	 (((credits) & 0xffu) << 17) | (1u << 31))

struct tb_rdma_dev {
	struct nhi_softc		*sc;
	bool				announced;	/* add() delivered */
	SLIST_ENTRY(tb_rdma_dev)	link;
};

static struct sx			tb_rdma_lock;
static SLIST_HEAD(, tb_rdma_dev)	tb_rdma_devs =
    SLIST_HEAD_INITIALIZER(tb_rdma_devs);
static struct tb_rdma_interface	*tb_rdma_consumer;

static void
tb_rdma_shim_init(void *arg __unused)
{
	sx_init(&tb_rdma_lock, "tbrdma");
}
SYSINIT(tb_rdma_shim, SI_SUB_LOCK, SI_ORDER_ANY, tb_rdma_shim_init, NULL);

int
tb_rdma_register_interface(struct tb_rdma_interface *intf)
{
	struct tb_rdma_dev *tbd;

	sx_xlock(&tb_rdma_lock);
	if (tb_rdma_consumer != NULL) {
		sx_xunlock(&tb_rdma_lock);
		return (EBUSY);
	}
	tb_rdma_consumer = intf;
	SLIST_FOREACH(tbd, &tb_rdma_devs, link) {
		if (!tbd->announced) {
			tbd->announced = true;
			intf->add(tbd);
		}
	}
	sx_xunlock(&tb_rdma_lock);
	return (0);
}

void
tb_rdma_unregister_interface(struct tb_rdma_interface *intf)
{
	struct tb_rdma_dev *tbd;

	sx_xlock(&tb_rdma_lock);
	if (tb_rdma_consumer != intf) {
		sx_xunlock(&tb_rdma_lock);
		return;
	}
	SLIST_FOREACH(tbd, &tb_rdma_devs, link) {
		if (tbd->announced) {
			tbd->announced = false;
			intf->remove(tbd);
		}
	}
	tb_rdma_consumer = NULL;
	sx_xunlock(&tb_rdma_lock);
}

void
tb_rdma_controller_ready(struct nhi_softc *sc)
{
	struct tb_rdma_dev *tbd;

	tbd = malloc(sizeof(*tbd), M_NHI, M_WAITOK | M_ZERO);
	tbd->sc = sc;

	sx_xlock(&tb_rdma_lock);
	SLIST_INSERT_HEAD(&tb_rdma_devs, tbd, link);
	if (tb_rdma_consumer != NULL) {
		tbd->announced = true;
		tb_rdma_consumer->add(tbd);
	}
	sx_xunlock(&tb_rdma_lock);
}

void
tb_rdma_controller_gone(struct nhi_softc *sc)
{
	struct tb_rdma_dev *tbd;

	sx_xlock(&tb_rdma_lock);
	SLIST_FOREACH(tbd, &tb_rdma_devs, link) {
		if (tbd->sc == sc)
			break;
	}
	if (tbd == NULL) {
		sx_xunlock(&tb_rdma_lock);
		return;
	}
	if (tbd->announced && tb_rdma_consumer != NULL) {
		tbd->announced = false;
		tb_rdma_consumer->remove(tbd);
	}
	SLIST_REMOVE(&tb_rdma_devs, tbd, tb_rdma_dev, link);
	sx_xunlock(&tb_rdma_lock);
	free(tbd, M_NHI);
}

device_t
tb_rdma_get_dev(struct tb_rdma_dev *tbd)
{
	return (tbd->sc->dev);
}

bus_dma_tag_t
tb_rdma_get_parent_dmat(struct tb_rdma_dev *tbd)
{
	return (tbd->sc->parent_dmat);
}

int
tb_rdma_get_unit(struct tb_rdma_dev *tbd)
{
	return (device_get_unit(tbd->sc->dev));
}

void
tb_rdma_get_uuid(struct tb_rdma_dev *tbd, uint8_t out[16])
{
	memcpy(out, tbd->sc->uuid, 16);
}

int
tb_rdma_get_peer(struct tb_rdma_dev *tbd, struct tb_rdma_peer *out)
{
	return (tb_icm_get_peer(tbd->sc->icm, out));
}

int
tb_rdma_ring_create(struct tb_rdma_dev *tbd, u_int ringnum, u_int tx_depth,
    u_int rx_depth, uint16_t frame_size, uint8_t e2e_hopid,
    uint16_t sof_mask, uint16_t eof_mask, struct tb_rdma_ring **out)
{
	struct nhi_ring_opts opts;
	struct nhi_ring_pair *r;
	int error;

	bzero(&opts, sizeof(opts));
	opts.frame_size = frame_size;
	opts.frame_mode = 1;
	opts.e2e = 1;
	opts.e2e_hopid = e2e_hopid;
	opts.sof_mask = sof_mask;
	opts.eof_mask = eof_mask;

	error = nhi_ring_create(tbd->sc, ringnum, tx_depth, rx_depth, &opts, &r);
	if (error == 0)
		*out = (struct tb_rdma_ring *)r;
	return (error);
}

int
tb_rdma_ring_start(struct tb_rdma_ring *r)
{
	return (nhi_ring_start((struct nhi_ring_pair *)r));
}

int
tb_rdma_ring_stop(struct tb_rdma_ring *r)
{
	return (nhi_ring_stop((struct nhi_ring_pair *)r));
}

void
tb_rdma_ring_destroy(struct tb_rdma_ring *r)
{
	nhi_ring_destroy((struct nhi_ring_pair *)r);
}

int
tb_rdma_program_rx_hop(struct tb_rdma_dev *tbd, uint8_t lane,
    u_int remote_tx_hopid, u_int rx_ring, u_int credits)
{
	struct router_softc *rsc = tbd->sc->root_rsc;
	uint32_t buf[2];

	if (rsc == NULL)
		return (ENXIO);
	/* RX hop lives on the lane adapter; next hop = the NHI ring. */
	buf[0] = TB_RDMA_HOP_DW0(rx_ring, TB_RDMA_NHI_PORT, credits);
	buf[1] = TB_RDMA_HOP_DW1;
	return (tb_config_path_write(rsc, lane, remote_tx_hopid, 1, buf));
}

int
tb_rdma_program_tx_hop(struct tb_rdma_dev *tbd, u_int local_tx_hopid,
    uint8_t lane, u_int tx_ring, u_int credits)
{
	struct router_softc *rsc = tbd->sc->root_rsc;
	uint32_t buf[2];

	if (rsc == NULL)
		return (ENXIO);
	/* TX hop lives on the NHI adapter; next hop = the lane, egress FC on. */
	buf[0] = TB_RDMA_HOP_DW0(local_tx_hopid, lane, credits);
	buf[1] = TB_RDMA_HOP_DW1_TX;
	return (tb_config_path_write(rsc, TB_RDMA_NHI_PORT, tx_ring, 1, buf));
}

void
tb_rdma_invalidate_hop(struct tb_rdma_dev *tbd, uint8_t adap, u_int hopid)
{
	struct router_softc *rsc = tbd->sc->root_rsc;
	uint32_t buf[2] = { 0, 0 };

	if (rsc != NULL)
		(void)tb_config_path_write(rsc, adap, hopid, 1, buf);
}

int
tb_rdma_read_hop(struct tb_rdma_dev *tbd, uint8_t adap, u_int hopid,
    uint32_t out[2])
{
	struct router_softc *rsc = tbd->sc->root_rsc;

	if (rsc == NULL)
		return (ENXIO);
	return (tb_config_path_read(rsc, adap, hopid, 1, out));
}

/*
 * Dependency token for the out-of-tree RDMA provider: tbrdma.ko declares
 * MODULE_DEPEND(tbrdma, thunderbolt, ...) so it only loads against a
 * thunderbolt.ko that carries this shim.
 */
MODULE_VERSION(thunderbolt, 1);

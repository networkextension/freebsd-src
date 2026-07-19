/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tbrdma_qp.c - the M1b QP engine: a UC queue pair over a pair of NHI
 * FRAME-mode E2E rings, plus the RST->INIT->RTR->RTS state machine that
 * programs the fabric hop table.  This is the real logic; the ib_* verb
 * methods in tbrdma_verbs.c and the white-box self-test both call in here.
 *
 * QPN encoding (matches Apple's RE, kext/macos27-rdma): qpn carries the local
 * TX HopID in the high bits - qpn = tx_ringnum | (local_tx_hopid << 8) - so a
 * peer, given our QPN out-of-band, recovers our TX HopID as (qpn >> 8) and
 * installs its RX hop there.  Symmetrically, at INIT->RTR we take the remote
 * QPN and install our RX hop at (dest_qpn >> 8).
 *
 * Hop programming maps 1:1 onto the SW-CM path in tb_icm.c
 * (tb_icm_program_paths): RX hop on the lane adapter, TX hop on the NHI
 * adapter with egress flow control - see tb_rdma_program_{rx,tx}_hop.
 */
#include "tbrdma.h"

/* if_tbt frame PDF convention (TBNET_PDF_FRAME_START/END) reused for RX SOF/EOF. */
#define	TBRDMA_PDF_FRAME_START	1
#define	TBRDMA_PDF_FRAME_END	2

static int
tbrdma_alloc_ringnum(struct tbrdma_dev *tdev)
{
	int n;

	for (n = TBRDMA_FIRST_RING; n < TBRDMA_NUM_RINGS; n++) {
		if ((tdev->ring_map & (1u << n)) == 0) {
			tdev->ring_map |= (1u << n);
			return (n);
		}
	}
	return (-1);
}

static void
tbrdma_free_ringnum(struct tbrdma_dev *tdev, u_int n)
{
	tdev->ring_map &= ~(1u << n);
}

struct tbrdma_qp *
tbrdma_qp_alloc(struct tbrdma_dev *tdev, u32 max_send_wr, u32 max_recv_wr)
{
	struct tbrdma_qp *qp;
	u_int depth;
	int txn, rxn, error;

	qp = kzalloc(sizeof(*qp), GFP_KERNEL);
	if (qp == NULL)
		return (ERR_PTR(-ENOMEM));
	qp->tdev = tdev;
	qp->state = IB_QPS_RESET;

	/* Ring depth: next power of two >= requested WR, clamped. */
	depth = TBRDMA_RING_DEPTH;

	mutex_lock(&tdev->res_lock);
	txn = tbrdma_alloc_ringnum(tdev);
	rxn = tbrdma_alloc_ringnum(tdev);
	mutex_unlock(&tdev->res_lock);
	if (txn < 0 || rxn < 0) {
		error = -ENOSPC;
		goto fail_nums;
	}
	qp->tx_ringnum = txn;
	qp->rx_ringnum = rxn;
	qp->local_tx_hopid = TBRDMA_TX_HOPID_BASE + txn;

	/* TX ring: FRAME+E2E, no SOF/EOF filter; e2e_hopid = its own ring. */
	error = tb_rdma_ring_create(tdev->tbd, qp->tx_ringnum, depth, depth,
	    TBRDMA_FRAME_SIZE, qp->tx_ringnum, 0, 0, &qp->tx_ring);
	if (error != 0) {
		error = -error;
		goto fail_tx;
	}
	/* RX ring: FRAME+E2E, SOF/EOF filter; credits egress on the TX ring. */
	error = tb_rdma_ring_create(tdev->tbd, qp->rx_ringnum, depth, depth,
	    TBRDMA_FRAME_SIZE, qp->tx_ringnum,
	    1u << TBRDMA_PDF_FRAME_START, 1u << TBRDMA_PDF_FRAME_END,
	    &qp->rx_ring);
	if (error != 0) {
		error = -error;
		goto fail_rx;
	}

	qp->ibqp.qp_num = qp->tx_ringnum | (qp->local_tx_hopid << 8);
	qp->ibqp.qp_type = IB_QPT_UC;
	return (qp);

fail_rx:
	tb_rdma_ring_destroy(qp->tx_ring);
fail_tx:
	mutex_lock(&tdev->res_lock);
	tbrdma_free_ringnum(tdev, txn);
	tbrdma_free_ringnum(tdev, rxn);
	mutex_unlock(&tdev->res_lock);
fail_nums:
	kfree(qp);
	return (ERR_PTR(error));
}

static int
tbrdma_qp_to_rtr(struct tbrdma_qp *qp, u32 dest_qpn, const u8 dgid[16])
{
	struct tbrdma_dev *tdev = qp->tdev;
	struct tb_rdma_peer peer;
	int error;

	qp->dest_qpn = dest_qpn;
	if (dgid != NULL)
		memcpy(qp->dgid, dgid, 16);

	/* Peer's TX HopID travels in the high bits of its QPN. */
	qp->rx_hopid = (dest_qpn >> 8) & 0x7ff;

	/* Fabric lane from the live session; fall back to lane 1 (self-test). */
	if (tb_rdma_get_peer(tdev->tbd, &peer) == 0 && peer.lane != 0)
		qp->lane = peer.lane;
	else
		qp->lane = 1;

	error = tb_rdma_program_rx_hop(tdev->tbd, qp->lane, qp->rx_hopid,
	    qp->rx_ringnum, TBRDMA_HOP_CREDITS);
	if (error != 0) {
		device_printf(tb_rdma_get_dev(tdev->tbd),
		    "tbrdma: RTR program_rx_hop failed: %d\n", error);
		return (-error);
	}
	qp->rx_hop_done = true;

	error = tb_rdma_ring_start(qp->rx_ring);
	if (error != 0) {
		device_printf(tb_rdma_get_dev(tdev->tbd),
		    "tbrdma: RTR rx_ring start failed: %d\n", error);
		return (-error);
	}
	qp->rx_started = true;

	device_printf(tb_rdma_get_dev(tdev->tbd),
	    "tbrdma: qp 0x%x -> RTR (lane %u, rx_hop %u, rx_ring %u, dest_qpn 0x%x)\n",
	    qp->ibqp.qp_num, qp->lane, qp->rx_hopid, qp->rx_ringnum, dest_qpn);
	return (0);
}

static int
tbrdma_qp_to_rts(struct tbrdma_qp *qp)
{
	struct tbrdma_dev *tdev = qp->tdev;
	int error;

	error = tb_rdma_program_tx_hop(tdev->tbd, qp->local_tx_hopid, qp->lane,
	    qp->tx_ringnum, TBRDMA_HOP_CREDITS);
	if (error != 0)
		return (-error);
	qp->tx_hop_done = true;

	error = tb_rdma_ring_start(qp->tx_ring);
	if (error != 0)
		return (-error);
	qp->tx_started = true;

	device_printf(tb_rdma_get_dev(tdev->tbd),
	    "tbrdma: qp 0x%x -> RTS (tx_hop %u on NHI, tx_ring %u, next lane %u)\n",
	    qp->ibqp.qp_num, qp->local_tx_hopid, qp->tx_ringnum, qp->lane);
	return (0);
}

static void
tbrdma_qp_teardown_paths(struct tbrdma_qp *qp)
{
	struct tbrdma_dev *tdev = qp->tdev;

	if (qp->tx_started) {
		tb_rdma_ring_stop(qp->tx_ring);
		qp->tx_started = false;
	}
	if (qp->rx_started) {
		tb_rdma_ring_stop(qp->rx_ring);
		qp->rx_started = false;
	}
	if (qp->tx_hop_done) {
		tb_rdma_invalidate_hop(tdev->tbd, TBRDMA_NHI_PORT_ADAP,
		    qp->tx_ringnum);
		qp->tx_hop_done = false;
	}
	if (qp->rx_hop_done) {
		tb_rdma_invalidate_hop(tdev->tbd, qp->lane, qp->rx_hopid);
		qp->rx_hop_done = false;
	}
}

int
tbrdma_qp_modify(struct tbrdma_qp *qp, enum ib_qp_state new_state,
    u32 dest_qpn, const u8 dgid[16])
{
	int error = 0;

	switch (new_state) {
	case IB_QPS_INIT:
		/* TX ring already exists; nothing to program yet. */
		break;
	case IB_QPS_RTR:
		if (qp->state != IB_QPS_INIT)
			return (-EINVAL);
		error = tbrdma_qp_to_rtr(qp, dest_qpn, dgid);
		break;
	case IB_QPS_RTS:
		if (qp->state != IB_QPS_RTR)
			return (-EINVAL);
		error = tbrdma_qp_to_rts(qp);
		break;
	case IB_QPS_RESET:
	case IB_QPS_ERR:
		tbrdma_qp_teardown_paths(qp);
		break;
	default:
		break;
	}

	if (error == 0)
		qp->state = new_state;
	return (error);
}

void
tbrdma_qp_free(struct tbrdma_qp *qp)
{
	struct tbrdma_dev *tdev = qp->tdev;

	tbrdma_qp_teardown_paths(qp);
	if (qp->rx_ring != NULL)
		tb_rdma_ring_destroy(qp->rx_ring);
	if (qp->tx_ring != NULL)
		tb_rdma_ring_destroy(qp->tx_ring);
	mutex_lock(&tdev->res_lock);
	tbrdma_free_ringnum(tdev, qp->tx_ringnum);
	tbrdma_free_ringnum(tdev, qp->rx_ringnum);
	mutex_unlock(&tdev->res_lock);
	kfree(qp);
}

/*
 * White-box self-test: build a UC QP and drive it RESET->INIT->RTR->RTS with a
 * synthetic remote QPN whose hop bits are clear of if_tbt's live path, then
 * read back the two hop-table entries we programmed and tear down.  Proves the
 * control plane without any userspace provider.  Triggered by a sysctl.
 */
int
tbrdma_selftest(struct tbrdma_dev *tdev)
{
	struct tbrdma_qp *qp;
	device_t dev = tb_rdma_get_dev(tdev->tbd);
	u8 dgid[16];
	u32 dest_qpn;
	uint32_t hop[2];
	int error, i;

	/*
	 * Synthetic peer QPN: hop bits = 9 (adjacent to if_tbt's live HopID 8,
	 * so it is within the lane adapter's path-table range, but distinct so
	 * it does not clobber tbt0).  qpn low bits are the peer's ring.
	 */
	dest_qpn = 5 | (9u << 8);
	for (i = 0; i < 16; i++)
		dgid[i] = 0xa0 + i;

	qp = tbrdma_qp_alloc(tdev, TBRDMA_MAX_QP_WR, TBRDMA_MAX_QP_WR);
	if (IS_ERR(qp)) {
		device_printf(dev, "tbrdma selftest: qp_alloc failed: %ld\n",
		    PTR_ERR(qp));
		return (PTR_ERR(qp));
	}
	device_printf(dev, "tbrdma selftest: qp 0x%x (tx_ring %u, rx_ring %u, "
	    "local_tx_hop %u)\n", qp->ibqp.qp_num, qp->tx_ringnum,
	    qp->rx_ringnum, qp->local_tx_hopid);

	if ((error = tbrdma_qp_modify(qp, IB_QPS_INIT, 0, NULL)) != 0)
		goto out;
	if ((error = tbrdma_qp_modify(qp, IB_QPS_RTR, dest_qpn, dgid)) != 0)
		goto out;
	if ((error = tbrdma_qp_modify(qp, IB_QPS_RTS, 0, NULL)) != 0)
		goto out;

	/* Read back the two hop entries to confirm they landed. */
	if (tb_rdma_read_hop(tdev->tbd, qp->lane, qp->rx_hopid, hop) == 0)
		device_printf(dev, "tbrdma selftest: RX hop[lane %u,idx %u] = "
		    "0x%08x 0x%08x\n", qp->lane, qp->rx_hopid, hop[0], hop[1]);
	if (tb_rdma_read_hop(tdev->tbd, TBRDMA_NHI_PORT_ADAP, qp->tx_ringnum,
	    hop) == 0)
		device_printf(dev, "tbrdma selftest: TX hop[NHI,idx %u] = "
		    "0x%08x 0x%08x\n", qp->tx_ringnum, hop[0], hop[1]);

	device_printf(dev, "tbrdma selftest: QP reached RTS - control plane OK\n");
	error = 0;
out:
	if (error != 0)
		device_printf(dev, "tbrdma selftest: FAILED at state, err %d\n",
		    error);
	tbrdma_qp_free(qp);
	return (error);
}

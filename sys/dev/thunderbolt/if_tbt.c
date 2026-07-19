/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * if_tbt.c - Apple ThunderboltIP network interface ("tbt") on the in-tree NHI
 * transport (ROADMAP.md Step 1b).
 *
 * The data plane for ThunderboltIP / USB4NET: a FRAME-mode, E2E data ring pair
 * created through the transport's public data-ring API (nhi_ring_create, Step
 * 1a), a standard Ethernet ifnet, TX that fragments an mbuf into ThunderboltIP
 * data frames, and RX driven by the transport's per-PDF callback.  Interoperates
 * with macOS "Thunderbolt Bridge" and Windows / other USB4NET hosts.
 *
 * Ported from the standalone nhi_icm driver (sys/dev/nhi/if_tbt.c).  The framing
 * and reassembly are unchanged; only the transport calls differ:
 *   - TX: nhi_alloc_tx_frame + nhi_tx_schedule (frame auto-frees on completion).
 *   - RX: a callback registered on the frame-end PDF (nhi_register_pdf), instead
 *     of a busy-poll loop.  The transport routes this ring to its own MSI-X
 *     vector, so RX is interrupt-driven (no poll ceiling).
 *
 * NOT yet wired to a connection manager: tbnet_attach takes the negotiated hop
 * ids as parameters; opening the DMA tunnel (APPROVE_XDOMAIN) and the manual
 * NHI-adapter TX HOPS config-write belong in the ICM module (Step 3).
 *
 * UNTESTED: written against the API, not yet compiled or run (board offline).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/endian.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <machine/bus.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/ethernet.h>
#include <net/vnet.h>
#include <net/bpf.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp_lro.h>

#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/if_tbt.h>

/*
 * ThunderboltIP data-frame framing (Apple protocol; not in the USB4 spec).  A
 * data frame is a 12-byte little-endian header + up to TBNET_MAX_PAYLOAD bytes;
 * one Ethernet frame is split across frame_count frames keyed by frame_id.
 */
#define	TBNET_DATA_TX_RING	1	/* local NHI ring: our TX (frame_mode + E2E) */
#define	TBNET_DATA_RX_RING	2	/* local NHI ring: our RX; E2E credits feed via the TX ring */
#define	TBNET_RING_DEPTH	1024
#define	TBNET_FRAME_SIZE	4096
#define	TBNET_HDR_LEN		12
#define	 TBNET_OFF_SIZE		0	/* __le32 this frame's payload length */
#define	 TBNET_OFF_INDEX	4	/* __le16 frame index (0-based) */
#define	 TBNET_OFF_ID		6	/* __le16 frame id (per network packet) */
#define	 TBNET_OFF_COUNT	8	/* __le32 frame count (frames in packet) */
#define	TBNET_MAX_PAYLOAD	(TBNET_FRAME_SIZE - TBNET_HDR_LEN)
#define	TBNET_MAX_FRAGS		17	/* howmany(64K, TBNET_MAX_PAYLOAD) */
#define	TBNET_TXQ_LEN		2048	/* software TX queue depth (packets) */
#define	TBNET_PDF_FRAME_START	1	/* descriptor SOF for a data frame */
#define	TBNET_PDF_FRAME_END	2	/* descriptor EOF for a data frame */

struct tbnet_softc {
	struct nhi_softc	*nsc;		/* the transport */
	if_t			ifp;
	struct nhi_ring_pair	*txring;	/* FRAME-mode TX ring (hop -> peer) */
	struct nhi_ring_pair	*rxring;	/* FRAME-mode RX ring (E2E -> txring) */
	struct mtx		lock;

	uint8_t			mac[ETHER_ADDR_LEN];
	uint8_t			peer_mac[ETHER_ADDR_LEN];

	uint16_t		tx_frame_id;	/* per-packet id, incrementing */

	/* Software TX queue: absorbs bursts the (flow-controlled) TX ring
	 * cannot, instead of dropping on ring-full (#69: multi-stream TCP
	 * collapsed to Mbit/s on RTO storms).  Guarded by ts->lock; drained
	 * on every transmit and by the tx_co callout while non-empty. */
	struct mbufq		tx_q;
	struct callout		tx_co;

	/* RX reassembly state (touched only by the RX callback). */
	struct mbuf		*rx_m;
	uint16_t		rx_id;
	u_int			rx_count;
	u_int			rx_index;
	int			rx_stamp;	/* ticks at last progress */

	/* LRO: coalesce TCP segments in the RX poll, flush once per batch
	 * (the FreeBSD equivalent of Linux's napi_gro_receive). */
	struct lro_ctrl		lro;
	bool			lro_ok;
};

static int	tbnet_transmit(if_t, struct mbuf *);
static void	tbnet_qflush(if_t);
static void	tbnet_init(void *);
static int	tbnet_ioctl(if_t, u_long, caddr_t);
static void	tbnet_rx_cb(void *, union nhi_ring_desc *, struct nhi_cmd_frame *);
static void	tbnet_rx_flush(void *);

/* Drop any half-reassembled RX packet. */
static void
tbnet_reass_reset(struct tbnet_softc *ts)
{
	if (ts->rx_m != NULL) {
		m_freem(ts->rx_m);
		ts->rx_m = NULL;
	}
	ts->rx_count = 0;
	ts->rx_index = 0;
}

/*
 * Create the resident ThunderboltIP interface at driver load - present as soon
 * as the driver attaches, independent of any peer (macOS-style: the Thunderbolt
 * Bridge is always there; carrier follows the link).  No data rings yet: those
 * come up in tbnet_connect() once the ThunderboltIP login + APPROVE complete.
 * The MAC is caller-supplied and stable (peer-independent).
 */
/* Accessor for the RDMA provider: the resident tbt0 ifnet (RoCE GID source). */
struct ifnet *
tbnet_get_ifp(struct tbnet_softc *ts)
{
	return (ts != NULL ? (struct ifnet *)ts->ifp : NULL);
}

int
tbnet_create(struct nhi_softc *nsc, const uint8_t *mac, struct tbnet_softc **tsp)
{
	struct tbnet_softc *ts;
	if_t ifp;

	ts = malloc(sizeof(*ts), M_NHI, M_NOWAIT | M_ZERO);
	if (ts == NULL)
		return (ENOMEM);
	ts->nsc = nsc;
	memcpy(ts->mac, mac, ETHER_ADDR_LEN);
	mtx_init(&ts->lock, "tbnet", NULL, MTX_DEF);
	mbufq_init(&ts->tx_q, TBNET_TXQ_LEN);
	callout_init_mtx(&ts->tx_co, &ts->lock, 0);

	ifp = if_alloc(IFT_ETHER);
	if (ifp == NULL)
		return (ENOMEM);
	ts->ifp = ifp;
	if_setsoftc(ifp, ts);
	if_initname(ifp, "tbt", device_get_unit(nsc->dev));
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setmtu(ifp, ETHERMTU);
	if_settransmitfn(ifp, tbnet_transmit);
	if_setqflushfn(ifp, tbnet_qflush);
	if_setinitfn(ifp, tbnet_init);
	if_setioctlfn(ifp, tbnet_ioctl);
	/*
	 * We are called from the ICM taskqueue thread, which has no vnet
	 * context; if_attach_internal dereferences curvnet (VIMAGE is on in
	 * GENERIC) and panics without this.
	 */
	CURVNET_SET(vnet0);
	ether_ifattach(ifp, ts->mac);
	CURVNET_RESTORE();
	/* Interface exists but the link is down until a peer logs in. */
	if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);
	if_link_state_change(ifp, LINK_STATE_DOWN);

	/*
	 * LRO: coalesce received TCP segments before the stack (the FreeBSD
	 * equivalent of Linux thunderbolt-net's napi_gro_receive - the main
	 * lever over our per-packet if_input path).  Best-effort: on failure
	 * we fall back to per-packet delivery.
	 */
	if (tcp_lro_init_args(&ts->lro, ifp, 8, 0) == 0) {
		ts->lro_ok = true;
		if_setcapabilitiesbit(ifp, IFCAP_LRO, 0);
		if_setcapenablebit(ifp, IFCAP_LRO, 0);
	}

	*tsp = ts;
	return (0);
}

/*
 * Raise carrier: allocate the data rings and start moving frames.  Called once
 * the ThunderboltIP login completes and the CM has APPROVEd the tunnel.
 * Idempotent - a second call while already connected is a no-op.
 */
int
tbnet_connect(struct tbnet_softc *ts)
{
	struct nhi_ring_opts opts;
	struct nhi_dispatch txd[1], rxd[2];
	int error;

	if (ts->txring != NULL)			/* already up */
		return (0);

	/*
	 * Two FRAME-mode, E2E rings, matching the proven ThunderboltIP data
	 * path: a TX ring (our frames -> peer) and a *separate* RX ring.  E2E
	 * flow control needs them distinct - the RX ring's credits are emitted
	 * on the TX ring (e2e_hopid = the TX ring), so a single bidirectional
	 * ring never sources credits and the peer's fully-E2E transmitter never
	 * puts a frame on the wire (observed as RX = 0).  Local ring numbers are
	 * not visible to the peer; the CM's APPROVE maps the fabric paths onto
	 * transmit_ring = TX ring, receive_ring = RX ring.
	 */
	memset(&opts, 0, sizeof(opts));
	opts.frame_size = TBNET_FRAME_SIZE;
	opts.frame_mode = 1;
	opts.e2e = 1;
	opts.e2e_hopid = TBNET_DATA_TX_RING;	/* credits egress via the TX ring */
	error = nhi_ring_create(ts->nsc, TBNET_DATA_TX_RING, TBNET_RING_DEPTH,
	    TBNET_RING_DEPTH, &opts, &ts->txring);
	if (error != 0)
		return (error);

	opts.sof_mask = 1u << TBNET_PDF_FRAME_START;
	opts.eof_mask = 1u << TBNET_PDF_FRAME_END;
	error = nhi_ring_create(ts->nsc, TBNET_DATA_RX_RING, TBNET_RING_DEPTH,
	    TBNET_RING_DEPTH, &opts, &ts->rxring);
	if (error != 0)
		goto fail_txring;

	/* RX: dispatch frame-end (EOF) frames on the RX ring to our callback.
	 * No TX callback (SOF=FRAME_START unregistered -> nhi_tx_complete auto-
	 * frees on the TX ring). */
	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = TBNET_PDF_FRAME_END;
	rxd[0].cb = tbnet_rx_cb;
	rxd[0].context = ts;
	rxd[1].pdf = 0; rxd[1].cb = NULL; rxd[1].context = NULL;
	if ((error = nhi_register_pdf(ts->rxring, txd, rxd)) != 0)
		goto fail_rxring;

	if ((error = nhi_ring_start(ts->txring)) != 0)
		goto fail_pdf;
	if ((error = nhi_ring_start(ts->rxring)) != 0)
		goto fail_pdf;

	/* NAPI-style batch end: flush LRO once after each RX ring drain. */
	if (ts->lro_ok) {
		ts->rxring->rx_batch_cb = tbnet_rx_flush;
		ts->rxring->rx_batch_ctx = ts;
	}

	if_setdrvflagbits(ts->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
	if_link_state_change(ts->ifp, LINK_STATE_UP);
	return (0);

fail_pdf:
	nhi_deregister_pdf(ts->rxring, txd, rxd);
fail_rxring:
	nhi_ring_destroy(ts->rxring);
	ts->rxring = NULL;
fail_txring:
	nhi_ring_destroy(ts->txring);
	ts->txring = NULL;
	return (error);
}

/*
 * Drop carrier and tear down the data rings, but keep the resident interface so
 * it reconnects cleanly.  Idempotent - safe if the rings were never up.
 */
void
tbnet_disconnect(struct tbnet_softc *ts)
{
	struct nhi_dispatch txd[1], rxd[2];

	if (ts->ifp != NULL) {
		if_setdrvflagbits(ts->ifp, 0, IFF_DRV_RUNNING);
		if_link_state_change(ts->ifp, LINK_STATE_DOWN);
	}
	if (ts->txring == NULL)			/* rings were never up */
		return;
	nhi_ring_stop(ts->rxring);
	nhi_ring_stop(ts->txring);
	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = TBNET_PDF_FRAME_END; rxd[0].cb = tbnet_rx_cb;
	rxd[0].context = ts;
	rxd[1].pdf = 0; rxd[1].cb = NULL; rxd[1].context = NULL;
	nhi_deregister_pdf(ts->rxring, txd, rxd);
	nhi_ring_destroy(ts->rxring);
	nhi_ring_destroy(ts->txring);
	ts->rxring = NULL;
	ts->txring = NULL;
	mtx_lock(&ts->lock);
	callout_stop(&ts->tx_co);
	mbufq_drain(&ts->tx_q);
	tbnet_reass_reset(ts);
	mtx_unlock(&ts->lock);
}

/* Full teardown at driver unload: drop the link, detach and free the ifnet. */
void
tbnet_destroy(struct tbnet_softc *ts)
{
	tbnet_disconnect(ts);
	callout_drain(&ts->tx_co);
	if (ts->ifp != NULL) {
		CURVNET_SET(vnet0);		/* no vnet context in the ICM taskq */
		ether_ifdetach(ts->ifp);
		CURVNET_RESTORE();
		if_free(ts->ifp);
		ts->ifp = NULL;
	}
	if (ts->lro_ok) {
		tcp_lro_free(&ts->lro);
		ts->lro_ok = false;
	}
	mtx_destroy(&ts->lock);
	free(ts, M_NHI);
}

/*
 * Fragment one Ethernet frame into ThunderboltIP data frames and schedule each
 * on the TX ring.  MTU 1500 => one frame; the loop also handles the multi-frame
 * (jumbo) case.  ts->lock held.  Consumes the mbuf on every return EXCEPT
 * ENOBUFS (ring full) - there the caller keeps ownership and queues it.
 */
static int
tbnet_encap_locked(struct tbnet_softc *ts, struct mbuf *m)
{
	if_t ifp = ts->ifp;
	struct nhi_cmd_frame *cmd, *cmds[TBNET_MAX_FRAGS];
	uint8_t *frame;
	u_int pktlen, off, index, count, chunk;
	uint16_t id;
	int error = 0;

	pktlen = m_length(m, NULL);
	if (pktlen == 0) {
		m_freem(m);
		return (0);
	}
	count = howmany(pktlen, TBNET_MAX_PAYLOAD);
	if (count > TBNET_MAX_FRAGS) {
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		m_freem(m);
		return (EMSGSIZE);
	}

	/*
	 * All-or-nothing: reserve every frame up front.  Scheduling a prefix
	 * and then running out mid-packet strands the peer's reassembly on
	 * the missing tail fragments (review P1).
	 */
	for (index = 0; index < count; index++) {
		cmds[index] = nhi_alloc_tx_frame(ts->txring);
		if (cmds[index] == NULL) {
			while (index > 0)
				nhi_free_tx_frame(ts->txring, cmds[--index]);
			return (ENOBUFS);	/* caller queues m */
		}
	}
	id = ts->tx_frame_id++;
	for (index = 0, off = 0; index < count; index++, off += chunk) {
		chunk = MIN(pktlen - off, TBNET_MAX_PAYLOAD);

		cmd = cmds[index];
		frame = (uint8_t *)cmd->data;
		le32enc(frame + TBNET_OFF_SIZE, chunk);
		le16enc(frame + TBNET_OFF_INDEX, index);
		le16enc(frame + TBNET_OFF_ID, id);
		le32enc(frame + TBNET_OFF_COUNT, count);
		m_copydata(m, off, chunk, (caddr_t)(frame + TBNET_HDR_LEN));

		cmd->req_len = TBNET_HDR_LEN + chunk;
		cmd->sof = TBNET_PDF_FRAME_START;	/* descriptor SOF */
		cmd->pdf = TBNET_PDF_FRAME_END;		/* descriptor EOF */
		error = nhi_tx_schedule(ts->txring, cmd);
		if (error != 0) {
			/* Free this and every not-yet-scheduled fragment;
			 * the already-scheduled prefix cannot be recalled. */
			for (; index < count; index++)
				nhi_free_tx_frame(ts->txring, cmds[index]);
			/*
			 * EBUSY on the FIRST fragment = descriptor ring full
			 * (1023 usable slots vs 1024 cmd frames - alloc can
			 * succeed while the ring is at capacity under flow-
			 * control backpressure).  Nothing is on the wire yet,
			 * so requeue instead of dropping.  Mid-packet failure
			 * has a live prefix and must drop the whole packet.
			 */
			if (error == EBUSY && off == 0)
				return (ENOBUFS);	/* caller queues m */
			break;
		}
	}

	if (error == 0) {
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
		if_inc_counter(ifp, IFCOUNTER_OBYTES, pktlen);
	} else
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
	m_freem(m);
	return (error);
}

/* Push queued packets onto the ring until it fills again.  ts->lock held. */
static void
tbnet_txq_drain_locked(struct tbnet_softc *ts)
{
	struct mbuf *m;

	/*
	 * Dequeue BEFORE encap: encap consumes (frees) the mbuf on success,
	 * and mbufq links through m_nextpkt - a peek-then-dequeue order reads
	 * the freed mbuf's m_nextpkt (UAF panic: GPF in m_length off a stale
	 * queue head).  On ENOBUFS the mbuf is untouched; put it back in front.
	 */
	while ((m = mbufq_dequeue(&ts->tx_q)) != NULL) {
		if (tbnet_encap_locked(ts, m) == ENOBUFS) {
			(void)mbufq_prepend(&ts->tx_q, m);
			break;
		}
	}
}

/* Callout: retry the queue while the ring drains at credit-return rate. */
static void
tbnet_txq_timer(void *arg)
{
	struct tbnet_softc *ts = arg;		/* ts->lock held (mtx callout) */

	if ((if_getdrvflags(ts->ifp) & IFF_DRV_RUNNING) == 0 ||
	    ts->txring == NULL) {
		mbufq_drain(&ts->tx_q);		/* link went down: flush */
		return;
	}
	tbnet_txq_drain_locked(ts);
	if (mbufq_len(&ts->tx_q) > 0)
		callout_reset_sbt(&ts->tx_co, SBT_1MS / 2, 0,
		    tbnet_txq_timer, ts, 0);
}

/*
 * if_transmit: send now if the (flow-controlled) TX ring has room, otherwise
 * queue with backpressure - dropping on ring-full turns peer-paced credit flow
 * into TCP RTO collapse (#69: 4-stream iperf fell to Mbit/s).
 */
static int
tbnet_transmit(if_t ifp, struct mbuf *m)
{
	struct tbnet_softc *ts = if_getsoftc(ifp);
	int error;

	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0 || ts->txring == NULL) {
		m_freem(m);		/* no carrier / rings not up (no peer) */
		return (ENETDOWN);
	}

	ETHER_BPF_MTAP(ifp, m);

	mtx_lock(&ts->lock);
	tbnet_txq_drain_locked(ts);
	if (mbufq_len(&ts->tx_q) == 0) {
		error = tbnet_encap_locked(ts, m);
		if (error != ENOBUFS) {
			mtx_unlock(&ts->lock);
			return (error);
		}
	}
	/* Ring full (or older packets still queued): keep FIFO order. */
	error = mbufq_enqueue(&ts->tx_q, m);
	if (error == 0) {
		if (!callout_pending(&ts->tx_co))
			callout_reset_sbt(&ts->tx_co, SBT_1MS / 2, 0,
			    tbnet_txq_timer, ts, 0);
	} else {
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		m_freem(m);
	}
	mtx_unlock(&ts->lock);
	return (error);
}

static void
tbnet_qflush(if_t ifp)
{
	struct tbnet_softc *ts = if_getsoftc(ifp);

	mtx_lock(&ts->lock);
	mbufq_drain(&ts->tx_q);
	mtx_unlock(&ts->lock);
}

static void
tbnet_init(void *arg)
{
	struct tbnet_softc *ts = arg;

	/*
	 * The interface is resident and can be brought up (ifconfig up, IP
	 * assignment) with no peer.  Do NOT claim IFF_DRV_RUNNING here - the
	 * driver is only ready to transmit once the data rings exist, which
	 * happens in tbnet_connect() when a peer logs in.  Setting RUNNING with
	 * ts->txring == NULL lets tbnet_transmit dereference a NULL ring.
	 */
	(void)ts;
}

static int
tbnet_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct ifreq *ifr = (struct ifreq *)data;

	switch (cmd) {
	case SIOCSIFMTU:
		/* TX fragments into 4 KB frames and RX reassembles, so the
		 * whole tbnet jumbo range works. */
		if (ifr->ifr_mtu < ETHERMIN || ifr->ifr_mtu > 9216)
			return (EINVAL);
		if_setmtu(ifp, ifr->ifr_mtu);
		return (0);
	case SIOCSIFFLAGS:
		/*
		 * ThunderboltIP is point-to-point: we already receive every
		 * frame on the RX ring, so promiscuous / allmulti are no-ops.
		 * Accept the flag change (ether_ioctl rejects SIOCSIFFLAGS with
		 * EINVAL) so bpf/tcpdump can enable promiscuous mode.
		 */
		return (0);
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		/* No hardware multicast filter; all frames are delivered. */
		return (0);
	case SIOCSIFCAP: {
		struct tbnet_softc *ts = if_getsoftc(ifp);
		int mask = ifr->ifr_reqcap ^ if_getcapenable(ifp);

		/* LRO is the only toggleable capability. */
		if ((mask & IFCAP_LRO) != 0 && ts->lro_ok)
			if_togglecapenable(ifp, IFCAP_LRO);
		return (0);
	}
	default:
		return (ether_ioctl(ifp, cmd, data));
	}
}

/* Validate a received frame header. */
static bool
tbnet_frame_ok(u_int len, u_int fsize, u_int count, u_int index)
{
	if (len < TBNET_HDR_LEN)
		return (false);
	if (fsize == 0 || fsize > len - TBNET_HDR_LEN)
		return (false);
	if (count == 0 || count > TBNET_MAX_FRAGS)
		return (false);
	if (index >= count)
		return (false);
	return (true);
}

/* Reassemble multi-frame packets; returns the completed mbuf or NULL. */
static struct mbuf *
tbnet_reass(struct tbnet_softc *ts, const uint8_t *payload, u_int len,
    u_int index, uint16_t id, u_int count)
{
	if_t ifp = ts->ifp;
	struct mbuf *m;

	if (index == 0) {
		tbnet_reass_reset(ts);
		ts->rx_m = m_gethdr(M_NOWAIT, MT_DATA);
		if (ts->rx_m == NULL) {
			if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
			return (NULL);
		}
		ts->rx_m->m_pkthdr.len = 0;
		ts->rx_m->m_len = 0;
		/*
		 * #69: rcvif was never set - this path first became reachable
		 * with the len==0-means-4096 fix (full-size first fragments
		 * used to die in the sanity check), and ether_input panics on
		 * a NULL rcvif ("ifnet mismatch ... rcvif 0").
		 */
		ts->rx_m->m_pkthdr.rcvif = ifp;
		ts->rx_id = id;
		ts->rx_count = count;
		ts->rx_index = 0;
	}
	if (ts->rx_m == NULL || id != ts->rx_id || count != ts->rx_count ||
	    index != ts->rx_index) {
		if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		tbnet_reass_reset(ts);
		return (NULL);
	}
	if (m_append(ts->rx_m, len, (const char *)payload) == 0) {
		if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		tbnet_reass_reset(ts);
		return (NULL);
	}
	ts->rx_index++;
	ts->rx_stamp = ticks;
	if (ts->rx_index == ts->rx_count) {
		m = ts->rx_m;
		ts->rx_m = NULL;
		ts->rx_count = 0;
		return (m);
	}
	return (NULL);
}

/*
 * Deliver one reassembled Ethernet frame.  Offer it to LRO first to coalesce
 * TCP segments; tcp_lro_rx() can itself flush->if_input when it evicts an entry,
 * so hold the net epoch across it and the non-LRO fallback.  Coalesced segments
 * are pushed up in one shot by tbnet_rx_flush() at the end of the ring drain.
 */
static void
tbnet_rx_deliver(struct tbnet_softc *ts, struct mbuf *m)
{
	struct epoch_tracker et;

	NET_EPOCH_ENTER(et);
	if (!ts->lro_ok || (if_getcapenable(ts->ifp) & IFCAP_LRO) == 0 ||
	    tcp_lro_rx(&ts->lro, m, 0) != 0)
		if_input(ts->ifp, m);
	NET_EPOCH_EXIT(et);
}

/*
 * Batch-end hook (nhi_ring_pair.rx_batch_cb): the transport calls this once
 * after draining the whole RX ring, so we flush all coalesced LRO segments in a
 * single pass - the NAPI/GRO shape that amortizes the per-packet stack cost.
 */
static void
tbnet_rx_flush(void *context)
{
	struct tbnet_softc *ts = context;
	struct epoch_tracker et;

	if (!ts->lro_ok)
		return;
	NET_EPOCH_ENTER(et);
	tcp_lro_flush_all(&ts->lro);
	NET_EPOCH_EXIT(et);
}

/*
 * RX PDF callback (interrupt context): the transport hands us a completed data
 * frame in cmd->data (reuse-in-place - copy out before returning).  Parse the
 * little-endian header, validate, and either fast-path a single-frame packet or
 * reassemble, then hand the mbuf to tbnet_rx_deliver (LRO/if_input).
 *
 * Reassembly state is touched only here and RX completions are serialized per
 * ring, so no lock is needed for it; we hold no lock across if_input (the stack
 * may transmit synchronously from input, which takes ts->lock).
 */
static void
tbnet_rx_cb(void *context, union nhi_ring_desc *rdesc,
    struct nhi_cmd_frame *cmd)
{
	struct tbnet_softc *ts = context;
	struct nhi_rx_post_desc *desc = (struct nhi_rx_post_desc *)rdesc;
	if_t ifp = ts->ifp;
	const uint8_t *frame = (const uint8_t *)cmd->data;
	const uint8_t *payload;
	struct mbuf *m;
	u_int len, fsize, index, count;
	uint16_t id;

	if (ifp == NULL)
		return;

	len = desc->eof_len & RX_BUFFER_DESC_LEN_MASK;
	/*
	 * #69: the 12-bit length field wraps - 0 means 4096 (the TX side
	 * already encodes it that way).  A full-size frame (4084-byte payload
	 * + 12-byte header) is EXACTLY 4096, so every first fragment of the
	 * peer's 2-frame packets read len=0 and died in the sanity check:
	 * ~all bulk RX dropped + reassembly cascade errors.
	 */
	if (len == 0)
		len = TBNET_FRAME_SIZE;
	fsize = le32dec(frame + TBNET_OFF_SIZE);
	index = le16dec(frame + TBNET_OFF_INDEX);
	id = le16dec(frame + TBNET_OFF_ID);
	count = le32dec(frame + TBNET_OFF_COUNT);
	if (!tbnet_frame_ok(len, fsize, count, index)) {
		if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		return;
	}

	/* A reassembly the peer abandoned (link flap mid-packet) would hold
	 * rx_m forever if only single-frame packets follow: age it out. */
	if (ts->rx_m != NULL && (u_int)(ticks - ts->rx_stamp) > (u_int)hz) {
		if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		tbnet_reass_reset(ts);
	}
	payload = frame + TBNET_HDR_LEN;

	if (count == 1) {			/* fast path: whole packet */
		m = m_devget(__DECONST(char *, payload), fsize, 0, ifp, NULL);
		if (m == NULL) {
			if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
			return;
		}
		if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
		if_inc_counter(ifp, IFCOUNTER_IBYTES, fsize);
	} else {
		m = tbnet_reass(ts, payload, fsize, index, id, count);
		if (m == NULL)
			return;
		if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
		if_inc_counter(ifp, IFCOUNTER_IBYTES, m->m_pkthdr.len);
	}
	tbnet_rx_deliver(ts, m);
}

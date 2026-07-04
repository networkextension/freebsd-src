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

#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/if_tbt.h>

/*
 * ThunderboltIP data-frame framing (Apple protocol; not in the USB4 spec).  A
 * data frame is a 12-byte little-endian header + up to TBNET_MAX_PAYLOAD bytes;
 * one Ethernet frame is split across frame_count frames keyed by frame_id.
 */
#define	TBNET_DATA_RING		1	/* our local NHI ring number for the pair */
#define	TBNET_RING_DEPTH	1024
#define	TBNET_FRAME_SIZE	4096
#define	TBNET_HDR_LEN		12
#define	 TBNET_OFF_SIZE		0	/* __le32 this frame's payload length */
#define	 TBNET_OFF_INDEX	4	/* __le16 frame index (0-based) */
#define	 TBNET_OFF_ID		6	/* __le16 frame id (per network packet) */
#define	 TBNET_OFF_COUNT	8	/* __le32 frame count (frames in packet) */
#define	TBNET_MAX_PAYLOAD	(TBNET_FRAME_SIZE - TBNET_HDR_LEN)
#define	TBNET_PDF_FRAME_START	1	/* descriptor SOF for a data frame */
#define	TBNET_PDF_FRAME_END	2	/* descriptor EOF for a data frame */

struct tbnet_softc {
	struct nhi_softc	*nsc;		/* the transport */
	if_t			ifp;
	struct nhi_ring_pair	*ring;		/* FRAME-mode data ring pair */
	struct mtx		lock;

	uint8_t			mac[ETHER_ADDR_LEN];
	uint8_t			peer_mac[ETHER_ADDR_LEN];

	uint16_t		tx_frame_id;	/* per-packet id, incrementing */

	/* RX reassembly state (touched only by the RX callback). */
	struct mbuf		*rx_m;
	uint16_t		rx_id;
	u_int			rx_count;
	u_int			rx_index;
};

static int	tbnet_transmit(if_t, struct mbuf *);
static void	tbnet_qflush(if_t);
static void	tbnet_init(void *);
static int	tbnet_ioctl(if_t, u_long, caddr_t);
static void	tbnet_rx_cb(void *, union nhi_ring_desc *, struct nhi_cmd_frame *);

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
 * Bring the ThunderboltIP interface up on the official transport.  The hop ids
 * come from the ThunderboltIP login (done elsewhere); the connection manager
 * (Step 3) must still APPROVE_XDOMAIN the tunnel before frames actually flow.
 */
int
tbnet_attach(struct nhi_softc *nsc, const uint8_t *mac, const uint8_t *peer_mac,
    u_int local_tx_hopid __unused, u_int remote_tx_hopid __unused,
    struct tbnet_softc **tsp)
{
	struct nhi_ring_opts opts;
	struct nhi_dispatch txd[1], rxd[2];
	struct tbnet_softc *ts;
	if_t ifp;
	int error;

	ts = malloc(sizeof(*ts), M_NHI, M_NOWAIT | M_ZERO);
	if (ts == NULL)
		return (ENOMEM);
	ts->nsc = nsc;
	memcpy(ts->mac, mac, ETHER_ADDR_LEN);
	memcpy(ts->peer_mac, peer_mac, ETHER_ADDR_LEN);
	mtx_init(&ts->lock, "tbnet", NULL, MTX_DEF);

	/*
	 * One FRAME-mode, E2E ring pair.  Unlike nhi_icm (which used separate
	 * NHI rings for TX hop 1 and RX hop 2), the in-tree transport bundles
	 * TX+RX per ring number, so we use a single bidirectional pair and pair
	 * its RX E2E credit with its own TX hop.  Local ring numbers are not
	 * visible to the peer; only the fabric paths (from login) are.  The
	 * connection manager's APPROVE must therefore use this ring number for
	 * both transmit_ring and receive_ring.
	 */
	memset(&opts, 0, sizeof(opts));
	opts.frame_size = TBNET_FRAME_SIZE;
	opts.frame_mode = 1;
	opts.e2e = 1;
	opts.e2e_hopid = TBNET_DATA_RING;
	opts.sof_mask = 1u << TBNET_PDF_FRAME_START;
	opts.eof_mask = 1u << TBNET_PDF_FRAME_END;
	error = nhi_ring_create(nsc, TBNET_DATA_RING, TBNET_RING_DEPTH,
	    TBNET_RING_DEPTH, &opts, &ts->ring);
	if (error != 0)
		goto fail;

	/* RX: dispatch frame-end (EOF) frames to our callback.  No TX callback
	 * (SOF=FRAME_START is unregistered, so nhi_tx_complete auto-frees). */
	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = TBNET_PDF_FRAME_END;
	rxd[0].cb = tbnet_rx_cb;
	rxd[0].context = ts;
	rxd[1].pdf = 0; rxd[1].cb = NULL; rxd[1].context = NULL;
	if ((error = nhi_register_pdf(ts->ring, txd, rxd)) != 0)
		goto fail_ring;

	if ((error = nhi_ring_start(ts->ring)) != 0)
		goto fail_pdf;

	ifp = if_alloc(IFT_ETHER);
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
	if_setdrvflagbits(ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
	if_link_state_change(ifp, LINK_STATE_UP);

	*tsp = ts;
	return (0);

fail_pdf:
	nhi_deregister_pdf(ts->ring, txd, rxd);
fail_ring:
	nhi_ring_destroy(ts->ring);
fail:
	mtx_destroy(&ts->lock);
	free(ts, M_NHI);
	return (error);
}

void
tbnet_detach(struct tbnet_softc *ts)
{
	struct nhi_dispatch txd[1], rxd[2];

	if (ts->ifp != NULL) {
		if_link_state_change(ts->ifp, LINK_STATE_DOWN);
		CURVNET_SET(vnet0);		/* no vnet context in the ICM taskq */
		ether_ifdetach(ts->ifp);
		CURVNET_RESTORE();
		if_free(ts->ifp);
	}
	nhi_ring_stop(ts->ring);
	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = TBNET_PDF_FRAME_END; rxd[0].cb = tbnet_rx_cb;
	rxd[0].context = ts;
	rxd[1].pdf = 0; rxd[1].cb = NULL; rxd[1].context = NULL;
	nhi_deregister_pdf(ts->ring, txd, rxd);
	nhi_ring_destroy(ts->ring);
	tbnet_reass_reset(ts);
	mtx_destroy(&ts->lock);
	free(ts, M_NHI);
}

/*
 * if_transmit: fragment one Ethernet frame into ThunderboltIP data frames and
 * schedule each on the TX ring.  MTU 1500 => one frame; the loop also handles
 * the multi-frame (jumbo) case.
 */
static int
tbnet_transmit(if_t ifp, struct mbuf *m)
{
	struct tbnet_softc *ts = if_getsoftc(ifp);
	struct nhi_cmd_frame *cmd;
	uint8_t *frame;
	u_int pktlen, off, index, count, chunk;
	uint16_t id;
	int error = 0;

	if ((if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0) {
		m_freem(m);
		return (ENETDOWN);
	}
	pktlen = m_length(m, NULL);
	if (pktlen == 0) {
		m_freem(m);
		return (0);
	}
	count = howmany(pktlen, TBNET_MAX_PAYLOAD);

	mtx_lock(&ts->lock);
	id = ts->tx_frame_id++;
	for (index = 0, off = 0; index < count; index++, off += chunk) {
		chunk = MIN(pktlen - off, TBNET_MAX_PAYLOAD);

		cmd = nhi_alloc_tx_frame(ts->ring);
		if (cmd == NULL) {		/* TX ring full */
			error = ENOBUFS;
			break;
		}
		frame = (uint8_t *)cmd->data;
		le32enc(frame + TBNET_OFF_SIZE, chunk);
		le16enc(frame + TBNET_OFF_INDEX, index);
		le16enc(frame + TBNET_OFF_ID, id);
		le32enc(frame + TBNET_OFF_COUNT, count);
		m_copydata(m, off, chunk, (caddr_t)(frame + TBNET_HDR_LEN));

		cmd->req_len = TBNET_HDR_LEN + chunk;
		cmd->sof = TBNET_PDF_FRAME_START;	/* descriptor SOF */
		cmd->pdf = TBNET_PDF_FRAME_END;		/* descriptor EOF */
		error = nhi_tx_schedule(ts->ring, cmd);
		if (error != 0) {
			nhi_free_tx_frame(ts->ring, cmd);
			break;
		}
	}
	mtx_unlock(&ts->lock);

	if (error == 0) {
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
		if_inc_counter(ifp, IFCOUNTER_OBYTES, pktlen);
	} else
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
	m_freem(m);
	return (error);
}

static void
tbnet_qflush(if_t ifp __unused)
{
}

static void
tbnet_init(void *arg)
{
	struct tbnet_softc *ts = arg;

	if (ts->ifp != NULL)
		if_setdrvflagbits(ts->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
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
	default:
		return (ether_ioctl(ifp, cmd, data));
	}
}

/* Validate a received frame header. */
static bool
tbnet_frame_ok(u_int len, u_int fsize, u_int count)
{
	if (len < TBNET_HDR_LEN)
		return (false);
	if (fsize == 0 || fsize > len - TBNET_HDR_LEN)
		return (false);
	if (count == 0 || count > TBNET_RING_DEPTH)
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
	if (ts->rx_index == ts->rx_count) {
		m = ts->rx_m;
		ts->rx_m = NULL;
		ts->rx_count = 0;
		return (m);
	}
	return (NULL);
}

/*
 * RX PDF callback (interrupt context): the transport hands us a completed data
 * frame in cmd->data (reuse-in-place - copy out before returning).  Parse the
 * little-endian header, validate, and either fast-path a single-frame packet or
 * reassemble, then ether_input inside the network epoch.
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
	struct epoch_tracker et;
	if_t ifp = ts->ifp;
	const uint8_t *frame = (const uint8_t *)cmd->data;
	const uint8_t *payload;
	struct mbuf *m;
	u_int len, fsize, index, count;
	uint16_t id;

	if (ifp == NULL)
		return;

	len = desc->eof_len & RX_BUFFER_DESC_LEN_MASK;
	fsize = le32dec(frame + TBNET_OFF_SIZE);
	index = le16dec(frame + TBNET_OFF_INDEX);
	id = le16dec(frame + TBNET_OFF_ID);
	count = le32dec(frame + TBNET_OFF_COUNT);
	if (!tbnet_frame_ok(len, fsize, count)) {
		if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		return;
	}
	payload = frame + TBNET_HDR_LEN;

	NET_EPOCH_ENTER(et);
	if (count == 1) {			/* fast path: whole packet */
		m = m_devget(__DECONST(char *, payload), fsize, 0, ifp, NULL);
		if (m == NULL) {
			if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		} else {
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
			if_inc_counter(ifp, IFCOUNTER_IBYTES, fsize);
			if_input(ifp, m);
		}
	} else {
		m = tbnet_reass(ts, payload, fsize, index, id, count);
		if (m != NULL) {
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
			if_inc_counter(ifp, IFCOUNTER_IBYTES, m->m_pkthdr.len);
			if_input(ifp, m);
		}
	}
	NET_EPOCH_EXIT(et);
}

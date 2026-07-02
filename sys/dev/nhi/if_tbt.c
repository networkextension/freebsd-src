/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * if_tbt.c - Apple ThunderboltIP network interface "tbt0" (Phase 3b).
 *
 * The data plane for the ThunderboltIP (USB4NET) service: once the ring-0 login
 * handshake completes (nhi_tbip.c) this allocates the data-ring pair, issues
 * APPROVE_XDOMAIN_PATHS to open the DMA tunnel, and attaches a standard Ethernet
 * ifnet.  TX fragments an mbuf into ThunderboltIP data frames; RX reassembles
 * them and feeds ether_input().  Interoperates with macOS "Thunderbolt Bridge"
 * and Windows/Linux USB4NET because all three speak the same Apple protocol.
 *
 * Cross-checked against Linux drivers/net/thunderbolt/main.c (tbnet_start_xmit /
 * tbnet_poll / tbnet_check_frame).  Data frames are little-endian raw bytes on
 * the ring (no ring-0 byteswap/CRC).  v1 is a single queue, MTU 1500, no
 * checksum offload (the stack computes L4 csums for us), no jumbo/RSS (D2).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/lock.h>
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

#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

static int	tbt_transmit(if_t ifp, struct mbuf *m);
static void	tbt_qflush(if_t ifp);
static void	tbt_init(void *arg);
static int	tbt_ioctl(if_t ifp, u_long cmd, caddr_t data);

/* Drop any half-reassembled RX packet. */
static void
tbt_reass_reset(struct nhi_softc *sc)
{
	if (sc->rx_m != NULL) {
		m_freem(sc->rx_m);
		sc->rx_m = NULL;
	}
	sc->rx_count = 0;
	sc->rx_index = 0;
}

/*
 * Bring the network service up: allocate the data rings, approve the DMA tunnel,
 * then attach (first time) and raise carrier on tbt0.  Called from the ring-0
 * event loop when the login handshake completes in both directions.
 */
int
nhi_tbt_connect(struct nhi_softc *sc)
{
	if_t ifp;
	int error;

	error = nhi_data_setup(sc);
	if (error != 0)
		return (error);
	error = nhi_icm_approve_xdomain_paths(sc);
	if (error != 0) {
		nhi_data_teardown(sc);
		return (error);
	}
	/* The MTL ICM approves but leaves the NHI-side TX hop empty - our
	 * frames died inside our own router until this is written. */
	nhi_install_tx_hop(sc);

	if (sc->ifp == NULL) {
		mtx_init(&sc->tbt_lock, "tbt", NULL, MTX_DEF);
		ifp = if_alloc(IFT_ETHER);
		sc->ifp = ifp;
		if_setsoftc(ifp, sc);
		if_initname(ifp, "tbt", device_get_unit(sc->dev));
		if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
		if_setmtu(ifp, ETHERMTU);
		if_settransmitfn(ifp, tbt_transmit);
		if_setqflushfn(ifp, tbt_qflush);
		if_setinitfn(ifp, tbt_init);
		if_setioctlfn(ifp, tbt_ioctl);
		/*
		 * We run in the ring-0 event-loop kthread, which has no vnet
		 * context; if_attach_internal dereferences curvnet (VIMAGE is
		 * on in GENERIC) and panics without this.
		 */
		CURVNET_SET(vnet0);
		ether_ifattach(ifp, sc->tbt_mac);
		CURVNET_RESTORE();
		device_printf(sc->dev, "tbt%d: attached, mac %6D peer %6D\n",
		    device_get_unit(sc->dev), sc->tbt_mac, ":", sc->peer_mac, ":");
	}

	sc->paths_approved = true;
	if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
	if_link_state_change(sc->ifp, LINK_STATE_UP);
	device_printf(sc->dev, "tbt%d: link UP\n", device_get_unit(sc->dev));
	return (0);
}

/* Drop carrier; keep the rings + ifnet for a later reconnect (full teardown at
 * detach). */
void
nhi_tbt_disconnect(struct nhi_softc *sc)
{
	bool was_up = sc->paths_approved;

	/* Reset ThunderboltIP login state so the next connect starts clean. */
	sc->tbip_login_sent = false;
	sc->tbip_login_received = false;
	sc->login_last = 0;
	sc->login_tries = 0;
	sc->logout_kicks = 0;
	sc->paths_approved = false;

	if (sc->ifp != NULL) {
		if_setdrvflagbits(sc->ifp, 0, IFF_DRV_RUNNING);
		if_link_state_change(sc->ifp, LINK_STATE_DOWN);
		mtx_lock(&sc->tbt_lock);
		tbt_reass_reset(sc);
		mtx_unlock(&sc->tbt_lock);
	}
	nhi_data_teardown(sc);		/* no-op if the data rings aren't up */
	if (was_up)
		device_printf(sc->dev, "tbt%d: link DOWN\n",
		    device_get_unit(sc->dev));
}

/*
 * ifnet if_transmit: fragment one Ethernet frame into ThunderboltIP data frames.
 * With MTU 1500 every packet fits one frame (payload <= NHI_TBIP_MAX_PAYLOAD),
 * so frame_count is normally 1; the loop also handles the multi-frame case.
 */
static int
tbt_transmit(if_t ifp, struct mbuf *m)
{
	struct nhi_softc *sc = if_getsoftc(ifp);
	uint8_t frame[NHI_DATA_FRAME_SIZE];
	u_int pktlen, off, index, count, chunk;
	uint16_t id;
	int error = 0;

	if (!sc->paths_approved ||
	    (if_getdrvflags(ifp) & IFF_DRV_RUNNING) == 0) {
		m_freem(m);
		return (ENETDOWN);
	}
	pktlen = m_length(m, NULL);
	if (pktlen == 0) {
		m_freem(m);
		return (0);
	}
	count = howmany(pktlen, NHI_TBIP_MAX_PAYLOAD);

	mtx_lock(&sc->tbt_lock);
	id = sc->tx_frame_id++;
	for (index = 0, off = 0; index < count; index++, off += chunk) {
		chunk = MIN(pktlen - off, NHI_TBIP_MAX_PAYLOAD);
		le32enc(frame + NHI_TBIP_FRAME_OFF_SIZE, chunk);
		le16enc(frame + NHI_TBIP_FRAME_OFF_INDEX, index);
		le16enc(frame + NHI_TBIP_FRAME_OFF_ID, id);
		le32enc(frame + NHI_TBIP_FRAME_OFF_COUNT, count);
		m_copydata(m, off, chunk,
		    (caddr_t)(frame + NHI_TBIP_FRAME_HDR_LEN));
		error = nhi_data_tx(sc, frame, NHI_TBIP_FRAME_HDR_LEN + chunk);
		if (error != 0)
			break;
	}
	mtx_unlock(&sc->tbt_lock);

	if (error == 0) {
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
		if_inc_counter(ifp, IFCOUNTER_OBYTES, pktlen);
	} else
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
	m_freem(m);
	return (error);
}

static void
tbt_qflush(if_t ifp __unused)
{
}

static void
tbt_init(void *arg)
{
	struct nhi_softc *sc = arg;

	if (sc->ifp != NULL && sc->paths_approved)
		if_setdrvflagbits(sc->ifp, IFF_DRV_RUNNING, IFF_DRV_OACTIVE);
}

static int
tbt_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	struct ifreq *ifr = (struct ifreq *)data;

	switch (cmd) {
	case SIOCSIFMTU:
		/* Jumbo: TX fragments into 4 KB ring frames and RX
		 * reassembles (frame_count), so anything up to the tbnet
		 * 9K range just works; 1500 costs ~270k pps at 3 Gbit/s. */
		if (ifr->ifr_mtu < ETHERMIN || ifr->ifr_mtu > 9216)
			return (EINVAL);
		if_setmtu(ifp, ifr->ifr_mtu);
		return (0);
	default:
		return (ether_ioctl(ifp, cmd, data));
	}
}

/* Validate a received frame header (Linux tbnet_check_frame). */
static bool
tbt_frame_ok(u_int len, u_int fsize, u_int count)
{
	if (len < NHI_TBIP_FRAME_HDR_LEN)
		return (false);
	if (fsize == 0 || fsize > len - NHI_TBIP_FRAME_HDR_LEN)
		return (false);
	if (count == 0 || count > NHI_DATA_RING_COUNT)
		return (false);
	return (true);
}

/* Reassemble multi-frame packets; RETURNS the completed packet (caller
 * if_input()s it after dropping tbt_lock) or NULL while incomplete. */
static struct mbuf *
tbt_reass(struct nhi_softc *sc, const uint8_t *payload, u_int len,
    u_int index, uint16_t id, u_int count)
{
	if_t ifp = sc->ifp;
	struct mbuf *m;

	if (index == 0) {
		tbt_reass_reset(sc);
		sc->rx_m = m_gethdr(M_NOWAIT, MT_DATA);
		if (sc->rx_m == NULL) {
			if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
			return (NULL);
		}
		sc->rx_m->m_pkthdr.len = 0;
		sc->rx_m->m_len = 0;
		sc->rx_id = id;
		sc->rx_count = count;
		sc->rx_index = 0;
	}
	if (sc->rx_m == NULL || id != sc->rx_id || count != sc->rx_count ||
	    index != sc->rx_index) {
		if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
		tbt_reass_reset(sc);
		return (NULL);
	}
	if (m_append(sc->rx_m, len, (const char *)payload) == 0) {
		if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		tbt_reass_reset(sc);
		return (NULL);
	}
	sc->rx_index++;
	if (sc->rx_index == sc->rx_count) {
		m = sc->rx_m;
		sc->rx_m = NULL;
		sc->rx_count = 0;
		return (m);
	}
	return (NULL);
}

/*
 * Drain the data RX ring (called from the ring-0 event loop).  Parse each
 * frame's little-endian header, validate, and either fast-path a single-frame
 * packet straight into an mbuf or hand it to the reassembler.
 */
void
nhi_tbt_rx_poll(struct nhi_softc *sc)
{
	uint8_t frame[NHI_DATA_FRAME_SIZE];
	struct epoch_tracker et;
	if_t ifp = sc->ifp;
	const uint8_t *payload;
	struct mbuf *m;
	u_int len, fsize, index, count;
	uint16_t id;

	if (!sc->data_up || ifp == NULL)
		return;

	/* if_input() must run inside the network epoch; we're a plain kthread
	 * (driver ithreads get this implicitly, we don't). */
	NET_EPOCH_ENTER(et);
	for (;;) {
		/*
		 * Take tbt_lock only for ring/reassembly work and DROP it
		 * before if_input(): the stack may transmit synchronously from
		 * input (e.g. the ARP reply), and tbt_transmit takes the same
		 * lock - holding it across if_input recursed and panicked the
		 * moment the first real frame arrived (if_tbt.c:162).
		 */
		mtx_lock(&sc->tbt_lock);
		len = sizeof(frame);
		if (nhi_data_rx(sc, frame, &len) != 0) {
			mtx_unlock(&sc->tbt_lock);
			break;			/* ring empty */
		}

		fsize = le32dec(frame + NHI_TBIP_FRAME_OFF_SIZE);
		index = le16dec(frame + NHI_TBIP_FRAME_OFF_INDEX);
		id = le16dec(frame + NHI_TBIP_FRAME_OFF_ID);
		count = le32dec(frame + NHI_TBIP_FRAME_OFF_COUNT);
		if (!tbt_frame_ok(len, fsize, count)) {
			mtx_unlock(&sc->tbt_lock);
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
			continue;
		}
		payload = frame + NHI_TBIP_FRAME_HDR_LEN;

		if (count == 1) {		/* fast path: whole packet, one frame */
			m = m_devget(__DECONST(char *, payload), fsize, 0,
			    ifp, NULL);
			mtx_unlock(&sc->tbt_lock);
			if (m == NULL) {
				if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
				continue;
			}
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
			if_inc_counter(ifp, IFCOUNTER_IBYTES, fsize);
			if_input(ifp, m);
			continue;
		}
		m = tbt_reass(sc, payload, fsize, index, id, count);
		mtx_unlock(&sc->tbt_lock);
		if (m != NULL) {
			if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
			if_inc_counter(ifp, IFCOUNTER_IBYTES, m->m_pkthdr.len);
			if_input(ifp, m);
		}
	}
	NET_EPOCH_EXIT(et);
}

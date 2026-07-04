/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_icm.c - ICM DRIVER_READY handshake (the TP-1b / ICM-gate probe).
 *
 * DRIVER_READY is a ring-0 control packet (PDF ICM_CMD), NOT a mailbox command
 * (the ICM firmware protocol wraps it as TB_CFG_PKG_ICM_CMD;
 * the response arrives as TB_CFG_PKG_ICM_RESP).  We send the 4-byte request,
 * wait for the response on ring 0, decode the integrated-controller
 * (icm_tr_pkg_driver_ready_response) layout, then re-read the firmware opmode to
 * see whether the dormant safe-mode ICM woke into CM mode.
 *
 * This is a deliberately self-contained verification probe living in nhi(4);
 * when Phase 2 adds tb(4) the ICM client moves there behind nhi's ring-0
 * transport API (DESIGN.md §4.1).  Message layout cross-checked against the ICM
 * firmware protocol (struct icm_pkg_header / icm_tr_pkg_driver_ready_response).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <machine/bus.h>

#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

int
nhi_icm_driver_ready(struct nhi_softc *sc)
{
	uint8_t req[4];
	uint8_t resp[NHI_CTL_FRAME_SIZE];
	uint16_t info, devid;
	uint32_t nvm;
	uint8_t code, flags, pdf;
	u_int rlen;
	int error, opmode;

	/* Request: icm_pkg_header{ code = ICM_DRIVER_READY }, rest zero. */
	bzero(req, sizeof(req));
	req[0] = NHI_ICM_DRIVER_READY;

	error = nhi_ctl_tx(sc, NHI_PDF_ICM_CMD, req, sizeof(req));
	if (error != 0) {
		device_printf(sc->dev, "DRIVER_READY tx failed: %d\n", error);
		return (error);
	}

	rlen = sizeof(resp);
	error = nhi_ctl_rx(sc, resp, &rlen, &pdf, 500);
	if (error != 0) {
		device_printf(sc->dev,
		    "DRIVER_READY: no ICM response (%d); intr_count=%u\n",
		    error, sc->intr_count);
		return (error);
	}

	code = resp[0];
	flags = resp[1];
	if (pdf != NHI_PDF_ICM_RESP || code != NHI_ICM_DRIVER_READY) {
		device_printf(sc->dev,
		    "DRIVER_READY: unexpected reply pdf=%u code=0x%02x len=%u\n",
		    pdf, code, rlen);
		return (EINVAL);
	}
	if ((flags & NHI_ICM_FLAGS_ERROR) != 0) {
		device_printf(sc->dev,
		    "DRIVER_READY rejected by ICM (flags=0x%02x)\n", flags);
		return (EIO);
	}

	/* Integrated controllers use the icm_tr (16-byte) response layout. */
	if (rlen >= 16) {
		info = le16dec(&resp[6]);
		nvm = le32dec(&resp[8]);
		devid = le16dec(&resp[12]);
		device_printf(sc->dev,
		    "DRIVER_READY ok: slevel=%u proto=%u nvm=0x%x devid=0x%04x\n",
		    info & NHI_ICM_TR_SLEVEL_MASK,
		    (info & NHI_ICM_TR_PROTO_VER_MASK) >>
		    NHI_ICM_TR_PROTO_VER_SHIFT, nvm, devid);
	} else {
		device_printf(sc->dev,
		    "DRIVER_READY ok (short response, len=%u)\n", rlen);
	}

	/* The gate: re-read opmode - did SAFE transition to CM? */
	opmode = nhi_mailbox_mode(sc);
	sc->fw_sts = nhi_read(sc, NHI_REG_FW_STS);
	device_printf(sc->dev, "ICM after DRIVER_READY: fw_mode=%s(0x%x) "
	    "fw_sts=0x%08x\n", nhi_fw_mode_str(opmode), opmode, sc->fw_sts);
	return (0);
}

/*
 * Drain up to n ICM notifications from ring 0 and hexdump them.  After
 * DRIVER_READY the firmware replays the current topology as ICM events
 * (DEVICE_CONNECTED / XDOMAIN_CONNECTED, ...).  This is the first look at the
 * event path a future tb(4) will consume - in particular an XDOMAIN_CONNECTED
 * carries the remote host's route + UUID (the start of the XDomain handshake).
 */
void
nhi_icm_drain(struct nhi_softc *sc, int n)
{
	uint8_t ev[NHI_CTL_FRAME_SIZE];
	u_int rlen;
	uint8_t pdf;
	int error, i, got = 0;

	for (i = 0; i < n; i++) {
		rlen = sizeof(ev);
		error = nhi_ctl_rx(sc, ev, &rlen, &pdf, 1000);
		if (error != 0)
			break;		/* ring drained */
		got++;
		device_printf(sc->dev, "ICM event %d: pdf=%u len=%u code=0x%02x\n",
		    i, pdf, rlen, rlen > 0 ? ev[0] : 0);
		hexdump(ev, rlen, "  icm ", 0);
	}
	if (got == 0)
		device_printf(sc->dev, "no ICM events on ring 0\n");
}

/*
 * Handle one ICM notification (PDF ICM_EVENT) from the event loop.  The one we
 * act on is XDOMAIN_CONNECTED (code 0x06): a 56-byte icm_tr event with
 * reserved(4) + remote_uuid(16)@8 + local_uuid(16)@24 + local_route(8)@40 +
 * remote_route(8)@48.  We stash the UUIDs + our route for the XDomain responder.
 */
void
nhi_icm_handle_event(struct nhi_softc *sc, const uint8_t *f, u_int len)
{
	uint8_t code = (len > 0) ? f[0] : 0;

	switch (code) {
	case NHI_ICM_EVENT_XDOMAIN_CONNECTED:
		if (len < 48) {
			device_printf(sc->dev, "xdomain connected: short event\n");
			break;
		}
		memcpy(sc->peer_uuid, f + 8, 16);
		memcpy(sc->local_uuid, f + 24, 16);
		sc->local_route_hi = le32dec(f + 40);
		sc->local_route_lo = le32dec(f + 44);
		/*
		 * The route we USE to reach the peer is the event's LOCAL route
		 * (the ICM firmware protocol: route = get_route(pkg->
		 * local_route_hi, local_route_lo) -> tb_xdomain_alloc).  The
		 * remote_route field is the path as seen from the peer's side -
		 * using it broke multi-hop topologies (peer behind a dock at
		 * hop 3 while our path to it is 0:1).
		 */
		sc->peer_route_hi = sc->local_route_hi;
		sc->peer_route_lo = sc->local_route_lo;
		sc->has_peer = true;
		nhi_tbt_disconnect(sc);	/* clean any stale session from a prior cycle */
		nhi_tbip_init(sc);	/* derive our MAC from the local UUID */
		nhi_tbip_logout(sc);	/* reset the peer's ThunderboltIP session too */
		nhi_xdomain_notify_changed(sc);	/* prompt the peer to re-read us */
		nhi_xdomain_request_props(sc);	/* read the peer's dir as a reference */
		device_printf(sc->dev, "xdomain connected: peer "
		    "%02x%02x%02x%02x-... local %02x%02x%02x%02x-... route %x:%x\n",
		    sc->peer_uuid[0], sc->peer_uuid[1], sc->peer_uuid[2],
		    sc->peer_uuid[3], sc->local_uuid[0], sc->local_uuid[1],
		    sc->local_uuid[2], sc->local_uuid[3], sc->local_route_hi,
		    sc->local_route_lo);
		break;
	case NHI_ICM_EVENT_XDOMAIN_DISCONNECTED:
		sc->has_peer = false;
		nhi_tbt_disconnect(sc);
		device_printf(sc->dev, "xdomain disconnected\n");
		break;
	case NHI_ICM_EVENT_DEVICE_CONNECTED:
		/*
		 * A downstream Thunderbolt/USB4 *device* (not a peer host) was
		 * attached - a TB SSD enclosure, dock, eGPU, etc.  Using one needs
		 * a PCIe (or DP/USB3) TUNNEL programmed through the fabric to the
		 * device, which is a different data path from the host-interface
		 * DMA rings this driver drives: the flow is ICM_APPROVE_DEVICE ->
		 * firmware builds the tunnel -> the tunneled endpoint hotplugs onto
		 * the host PCIe tree (FreeBSD pci/nvme then binds it).  NONE of that
		 * is implemented: decision D1 scopes v1 to host-to-host XDomain
		 * networking only, and PCIe/DP tunnelling is a Phase-5 item.  So we
		 * just log it here - a connected device will NOT appear on the host.
		 */
		device_printf(sc->dev, "ICM device connected (code 0x03, len=%u): "
		    "PCIe tunnelling not implemented (D1/Phase 5); device ignored\n",
		    len);
		break;
	default:
		device_printf(sc->dev, "ICM event code=0x%02x len=%u\n", code, len);
		break;
	}
}

/*
 * ICM_DISCONNECT_XDOMAIN (0x11) - tear the XDomain link down at the FABRIC
 * level, so the peer also sees the link drop and restarts its whole discovery
 * + login state machine.  This is the piece that makes a software "replug"
 * visible to the other side (our force-power reset alone only refreshes us).
 * The ICM firmware protocol XDOMAIN tear-down: 32-byte icm_tr_pkg_disconnect_xdomain
 * { hdr(4); u8 stage; u8 rsvd[3]; u32 route_hi; u32 route_lo; uuid[16] },
 * sent twice: stage 1, wait 10-50us, stage 2.
 */
static int
nhi_icm_disconnect_stage(struct nhi_softc *sc, uint8_t stage)
{
	uint8_t req[32];
	uint8_t resp[NHI_CTL_FRAME_SIZE];
	uint8_t pdf;
	u_int rlen;
	int i;

	bzero(req, sizeof(req));
	req[0] = NHI_ICM_DISCONNECT_XDOMAIN;
	req[4] = stage;
	le32enc(req + 8, sc->peer_route_hi);
	le32enc(req + 12, sc->peer_route_lo);
	memcpy(req + 16, sc->peer_uuid, 16);
	if (nhi_ctl_tx(sc, NHI_PDF_ICM_CMD, req, sizeof(req)) != 0)
		return (EIO);
	for (i = 0; i < 10; i++) {
		rlen = sizeof(resp);
		if (nhi_ctl_rx(sc, resp, &rlen, &pdf, 400) != 0)
			break;
		if (pdf == NHI_PDF_ICM_RESP &&
		    resp[0] == NHI_ICM_DISCONNECT_XDOMAIN)
			return ((resp[1] & 0x01) != 0 ? EIO : 0);
	}
	return (ETIMEDOUT);
}

int
nhi_icm_disconnect_xdomain(struct nhi_softc *sc)
{
	int e1, e2;

	if (!sc->has_peer)
		return (0);
	e1 = nhi_icm_disconnect_stage(sc, 1);
	DELAY(50);
	e2 = nhi_icm_disconnect_stage(sc, 2);
	device_printf(sc->dev, "DISCONNECT_XDOMAIN: stage1=%d stage2=%d\n",
	    e1, e2);
	return (e1 != 0 ? e1 : e2);
}

/*
 * Install the NHI-side TX hop entry the MTL ICM leaves empty: APPROVE
 * programs the lane-side RX entry (lane HOPS[peer hopid] -> NHI ring, verified
 * by config-space readback) but NOT the reverse one, so our TX frames -
 * entering the router at the NHI adapter with HopID == ring number - hit an
 * empty table and die inside our own router (and E2E credits never reach the
 * peer either).  Write it ourselves via CFG_WRITE (this is the one SW-CM
 * duty ICM mode still leaves us): NHI adapter HOPS[tx ring] -> out_port =
 * first hop of the session route, next_hop = our announced TX HopID.  Entry
 * style copied from the fw-programmed RX entry (credits 14, weight 1,
 * priority 5, counters+ingress_fc bits = 0x0180c501).
 */
int
nhi_install_tx_hop(struct nhi_softc *sc)
{
	uint8_t req[20];
	uint8_t resp[NHI_CTL_FRAME_SIZE];
	uint8_t pdf;
	uint32_t addr;
	u_int rlen, out_port, i;

	out_port = sc->peer_route_lo & 0x3f;	/* first hop byte = lane port */
	if (out_port == 0)
		out_port = 1;
	bzero(req, sizeof(req));
	addr = ((NHI_DATA_TX_RING * 2) & 0x1fff) |	/* HOPS entry = ring */
	    (2u << 13) |		/* 2 dwords */
	    (7u << 19) |		/* MTL NHI adapter = port 7 (portdump) */
	    (0u << 25) | (1u << 27);	/* space HOPS, seq */
	le32enc(req + 8, addr);
	le32enc(req + 12, (sc->local_tx_hopid & 0x7ff) | (out_port << 11) |
	    (14u << 17) | (1u << 31));	/* next_hop | out_port | credits | EN */
	le32enc(req + 16, 0x0180c501);
	if (nhi_ctl_tx(sc, 2 /* TB_CFG_PKG_WRITE */, req, sizeof(req)) != 0)
		return (EIO);
	for (i = 0; i < 8; i++) {
		rlen = sizeof(resp);
		if (nhi_ctl_rx(sc, resp, &rlen, &pdf, 300) != 0)
			break;
		if (pdf == 2) {		/* write ack */
			device_printf(sc->dev, "TX hop installed: NHI(7) "
			    "HOPS[%d] -> port %u hop %u\n", NHI_DATA_TX_RING,
			    out_port, sc->local_tx_hopid);
			return (0);
		}
	}
	device_printf(sc->dev, "TX hop install: no write ack\n");
	return (ETIMEDOUT);
}

/*
 * APPROVE_XDOMAIN_PATHS - set up the bidirectional DMA tunnel for the network
 * service (the ICM firmware protocol, TR variant for integrated controllers).  We chose the
 * NHI ring indices and HopIDs already: our TX HopID + data TX ring, and the
 * peer's TX HopID (its login transmit_path) as our receive path + data RX ring.
 * Request layout (icm_tr_pkg_approve_xdomain, 40 bytes; response echoes it):
 *   hdr(4) route_hi(4) route_lo(4) remote_uuid(16) transmit_path(u16)
 *   transmit_ring(u16) receive_path(u16) receive_ring(u16) reserved(4)
 * Called from the ring-0 event loop (no concurrent reaper), so the synchronous
 * nhi_ctl_rx for the response is safe here.
 */
int
nhi_icm_approve_xdomain_paths(struct nhi_softc *sc)
{
	uint8_t req[36];	/* struct icm_tr_pkg_approve_xdomain (ICM firmware protocol) */
	uint8_t resp[NHI_CTL_FRAME_SIZE];
	uint8_t code, flags, pdf;
	u_int rlen;
	int error, attempt, i;

	bzero(req, sizeof(req));
	req[0] = NHI_ICM_APPROVE_XDOMAIN;
	le32enc(req + 4, sc->peer_route_hi);
	le32enc(req + 8, sc->peer_route_lo);
	memcpy(req + 12, sc->peer_uuid, 16);
	le16enc(req + 28, (uint16_t)sc->local_tx_hopid);	/* transmit_path */
	le16enc(req + 30, NHI_DATA_TX_RING);			/* transmit_ring */
	le16enc(req + 32, (uint16_t)sc->remote_tx_hopid);	/* receive_path */
	le16enc(req + 34, NHI_DATA_RX_RING);			/* receive_ring */

	/*
	 * The ICM request flow: send the command, then match a reply by
	 * (pdf == ICM_RESP && code == request code) - icm_match(); async ICM
	 * events (DP_CONFIG_CHANGED=0x08, RTD3_VETO=0x0a) share this ring-0 RX so
	 * skip anything that isn't our reply.  Retry ICM_RETRIES(=3) times.
	 */
	code = flags = pdf = 0;
	for (attempt = 0; attempt < 3; attempt++) {
		error = nhi_ctl_tx(sc, NHI_PDF_ICM_CMD, req, sizeof(req));
		if (error != 0) {
			device_printf(sc->dev,
			    "APPROVE_XDOMAIN tx failed: %d\n", error);
			return (error);
		}
		for (i = 0; i < 10; i++) {
			rlen = sizeof(resp);
			if (nhi_ctl_rx(sc, resp, &rlen, &pdf, 400) != 0)
				break;		/* timeout: re-send */
			code = resp[0];
			flags = resp[1];
			if (pdf == NHI_PDF_ICM_RESP &&
			    code == NHI_ICM_APPROVE_XDOMAIN)
				goto matched;
		}
		device_printf(sc->dev, "APPROVE_XDOMAIN: no reply, retry %d\n",
		    attempt + 1);
	}
	device_printf(sc->dev, "APPROVE_XDOMAIN: no ICM_RESP after retries\n");
	return (ETIMEDOUT);
matched:
	if ((flags & NHI_ICM_FLAGS_ERROR) != 0) {
		device_printf(sc->dev,
		    "APPROVE_XDOMAIN rejected (flags=0x%02x)\n", flags);
		return (EIO);
	}
	/*
	 * Parse the echoed fields - the fw may return the hopids/rings it
	 * ACTUALLY programmed (we previously ignored everything but the code).
	 */
	device_printf(sc->dev, "APPROVE_XDOMAIN ok: reply len=%u route %x:%x "
	    "tx_path=%u tx_ring=%u rx_path=%u rx_ring=%u (we asked tx %u/%d "
	    "rx %u/%d)\n", rlen,
	    le32dec(resp + 4), le32dec(resp + 8),
	    le16dec(resp + 28), le16dec(resp + 30),
	    le16dec(resp + 32), le16dec(resp + 34),
	    sc->local_tx_hopid, NHI_DATA_TX_RING,
	    sc->remote_tx_hopid, NHI_DATA_RX_RING);
	return (0);
}

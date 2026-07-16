/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tb_icm.c - ICM-mode connection manager + ThunderboltIP login, on the in-tree
 * NHI transport (ROADMAP.md Step 3).  This is the capstone that ties the
 * transport (Step 1a), the ThunderboltIP net driver (Step 1b, if_tbt), and the
 * XDomain responder (Step 2, tb_xdomain) into a loadable ICM stack.
 *
 * ICM = firmware connection manager: the OS sends high-level commands
 * (DRIVER_READY, APPROVE_XDOMAIN_PATHS) and receives notifications on ring 0; it
 * does NOT walk routers (that is the HCM path, router.c/hcm.c).  This module is
 * selected at attach when the controller firmware reports CM mode, beside the
 * in-tree HCM.
 *
 * Ported from the standalone nhi_icm driver (sys/dev/nhi/nhi_icm.c + nhi_tbip.c).
 * The protocol logic (DRIVER_READY, event decode, APPROVE, TX-hop install, the
 * ThunderboltIP login state machine, MAC derivation) is faithful; the CONCURRENCY
 * model is new.  nhi_icm ran everything in a blocking ring-0 event-loop kthread;
 * the in-tree transport is interrupt-callback driven, so:
 *   - ring-0 RX arrives in nhi_register_pdf callbacks (interrupt context).
 *   - blocking ICM req/resp (DRIVER_READY, APPROVE) run in a taskqueue thread,
 *     woken by the CM_RESP callback (msleep/wakeup).
 *   - login retry runs on a callout.
 * Frame wire I/O (native <-> big-endian dword + CRC) is done here, as router.c
 * does for config packets and tb_xdomain does for discovery.
 *
 * UNTESTED: compiles clean; not run (loading conflicts with the standalone
 * nhi_icm on the same NHI, and needs the fw-opmode attach glue).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/endian.h>
#include <sys/gsb_crc32.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <machine/bus.h>

#include <dev/pci/pcivar.h>
#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/tb_reg.h>
#include <dev/thunderbolt/tb_var.h>
#include <dev/thunderbolt/tb_debug.h>
#include <dev/thunderbolt/tbcfg_reg.h>
#include <dev/thunderbolt/router_var.h>
#include <dev/thunderbolt/tb_xdomain.h>
#include <dev/thunderbolt/if_tbt.h>
#include <dev/thunderbolt/tb_icm.h>

/* ICM protocol (firmware connection manager); not in the in-tree headers. */
#define	ICM_DRIVER_READY		0x03
/* #42: bound on post-DRIVER_READY software-replug kicks (x2s = ~30s window) */
#define	NHI_ICM_XDKICK_MAX		15
#define	 ICM_FLAGS_ERROR		(1u << 0)
#define	 ICM_TR_SLEVEL_MASK		0x0007
#define	 ICM_TR_PROTO_VER_SHIFT		4
#define	 ICM_TR_PROTO_VER_MASK		0x0070
#define	ICM_EVENT_DEVICE_CONNECTED	0x03
#define	ICM_EVENT_XDOMAIN_CONNECTED	0x06
#define	ICM_EVENT_XDOMAIN_DISCONNECTED	0x07
#define	ICM_APPROVE_XDOMAIN		0x10
#define	ICM_DISCONNECT_XDOMAIN		0x11

/* ThunderboltIP login (Apple protocol; frames ride PDF_XDOMAIN_RESP). */
#define	TBIP_HDR_LEN		68
#define	 TBIP_OFF_UUID		12
#define	 TBIP_OFF_INITIATOR	28
#define	 TBIP_OFF_TARGET	44
#define	 TBIP_OFF_TYPE		60
#define	 TBIP_OFF_COMMAND_ID	64
#define	TBIP_LOGIN		0
#define	TBIP_LOGIN_RESPONSE	1
#define	TBIP_LOGOUT		2
#define	TBIP_STATUS		3
#define	TBIP_PROTO_VERSION	1
#define	TBIP_SVC_UUID	{ 0x79,0x8f,0x58,0x9e, 0x36,0x16, 0x8a,0x47, \
			  0x97,0xc6, 0x56,0x64,0xa9,0x20,0xc8,0xdd }
#define	XDP_LENGTH_MASK	0x0000003f
#define	XDP_SN_MASK	0x18000000

#define	ICM_DATA_TX_RING	1	/* if_tbt local TX ring (must match if_tbt.c) */
#define	ICM_DATA_RX_RING	2	/* if_tbt local RX ring (separate for E2E) */
#define	ICM_FRAME_SIZE		256

static const uint8_t tbip_svc_uuid[16] = TBIP_SVC_UUID;

struct tb_icm {
	struct nhi_softc	*nsc;
	struct nhi_ring_pair	*ring0;
	struct tb_xdomain	*xd;
	struct tbnet_softc	*net;
	struct mtx		lock;
	struct taskqueue	*tq;
	struct task		bringup_task;
	struct task		teardown_task;
	struct callout		login_co;
	bool			dying;		/* teardown: stop callout re-arm */
	bool			native;		/* native USB4 (SW-CM), no ICM fw */

	/* synchronous ICM req/resp capture (waiter <- CM_RESP callback) */
	bool			waiting;
	uint8_t			want_code;
	uint8_t			resp[ICM_FRAME_SIZE];
	u_int			resp_len;

	/* session state (from nhi_softc in the standalone driver) */
	bool			has_peer;
	bool			paths_approved;
	uint8_t			peer_uuid[16];
	uint8_t			local_uuid[16];
	uint32_t		peer_route_hi, peer_route_lo;
	uint8_t			tbt_mac[6];
	uint8_t			peer_mac[6];
	u_int			local_tx_hopid;
	u_int			remote_tx_hopid;
	bool			login_sent, login_received;
	u_int			login_tries;
	u_int			logout_kicks;

	/* #42: auto software-replug when the ICM skips XDOMAIN_CONNECTED */
	struct task		xdkick_task;
	u_int			xdkick_tries;
};

static void tb_icm_login_timer(void *);
static void tbip_send_logout(struct tb_icm *);

/* ---- ring-0 control frame wire I/O (native <-> big-endian dword + CRC) ---- */

static int
tb_icm_ctl_tx(struct tb_icm *icm, uint8_t pdf, const uint8_t *data, u_int len)
{
	struct nhi_cmd_frame *cmd;
	uint8_t *fbuf;
	uint32_t crc;
	u_int i;

	if (len + 4 > ICM_FRAME_SIZE)
		return (EINVAL);
	cmd = nhi_alloc_tx_frame(icm->ring0);
	if (cmd == NULL)
		return (ENOBUFS);
	fbuf = (uint8_t *)cmd->data;
	for (i = 0; i < len; i += 4)
		be32enc(fbuf + i, le32dec(data + i));
	crc = ~calculate_crc32c(~0U, fbuf, len);
	be32enc(fbuf + len, crc);
	cmd->req_len = len + 4;
	cmd->pdf = pdf;
	cmd->sof = 0;
	if (nhi_tx_schedule(icm->ring0, cmd) != 0) {
		nhi_free_tx_frame(icm->ring0, cmd);
		return (EIO);
	}
	return (0);
}

/*
 * Send an ICM command (PDF_CM_REQ) and block until the matching CM_RESP arrives
 * (captured by tb_icm_resp_cb).  MUST run in a sleepable context (the taskqueue
 * thread or attach), never a callback.
 */
static int
tb_icm_request(struct tb_icm *icm, uint8_t code, const uint8_t *req, u_int len,
    uint8_t *resp, u_int *resplen, int timo_ms)
{
	int error;

	mtx_lock(&icm->lock);
	icm->waiting = true;
	icm->want_code = code;
	icm->resp_len = 0;
	mtx_unlock(&icm->lock);

	if ((error = tb_icm_ctl_tx(icm, PDF_CM_REQ, req, len)) != 0) {
		mtx_lock(&icm->lock);
		icm->waiting = false;
		mtx_unlock(&icm->lock);
		return (error);
	}

	mtx_lock(&icm->lock);
	while (icm->waiting && !icm->dying)
		if (msleep(icm, &icm->lock, 0, "tbicm", timo_ms * hz / 1000) != 0)
			break;			/* timeout */

	/*
	 * Timed out with the interrupt path.  The ring0 MSI-X vector may
	 * simply not be firing (#26: MTL NHIs have lost per-ring vector
	 * delivery before) - drain ring 0 by hand once.  If the response was
	 * sitting in the ring, the interrupt path is dead, not the ICM:
	 * switch ring 0 over to the same poll that services the data rings
	 * (Apple's own integrated HAL falls back to a poll timer too) and
	 * carry on.
	 */
	if (icm->waiting && !icm->dying) {
		mtx_unlock(&icm->lock);
		nhi_ring_poll(icm->ring0);
		mtx_lock(&icm->lock);
		if (!icm->waiting) {
			mtx_unlock(&icm->lock);
			device_printf(icm->nsc->dev, "ICM response recovered "
			    "by manual drain - ring0 interrupt not firing; "
			    "switching ring0 to polling\n");
			nhi_ring_force_poll(icm->ring0);
			mtx_lock(&icm->lock);
		}
	}
	if (icm->waiting) {			/* timed out or dying */
		icm->waiting = false;
		mtx_unlock(&icm->lock);
		return (ETIMEDOUT);
	}
	if (resp != NULL && resplen != NULL) {
		*resplen = MIN(*resplen, icm->resp_len);
		memcpy(resp, icm->resp, *resplen);
	}
	mtx_unlock(&icm->lock);
	return (0);
}

/* rxpdf[CM_RESP] callback (interrupt): capture the reply for a waiter. */
static void
tb_icm_resp_cb(void *context, union nhi_ring_desc *rdesc,
    struct nhi_cmd_frame *cmd)
{
	struct tb_icm *icm = context;
	struct nhi_rx_post_desc *desc = (struct nhi_rx_post_desc *)rdesc;
	const uint8_t *src = (const uint8_t *)cmd->data;
	uint8_t f[ICM_FRAME_SIZE];
	u_int len, ndw, j;

	len = desc->eof_len & RX_BUFFER_DESC_LEN_MASK;
	if (len < 4 || len > sizeof(f))
		return;
	len -= 4;
	ndw = len / 4;
	for (j = 0; j < ndw; j++)
		le32enc(f + j * 4, be32dec(src + j * 4));

	mtx_lock(&icm->lock);
	if (icm->waiting && f[0] == icm->want_code) {
		memcpy(icm->resp, f, len);
		icm->resp_len = len;
		icm->waiting = false;
		wakeup(icm);
	}
	mtx_unlock(&icm->lock);
}

/* ---- DRIVER_READY -------------------------------------------------------- */

static int
tb_icm_driver_ready(struct tb_icm *icm)
{
	uint8_t req[4], resp[ICM_FRAME_SIZE];
	u_int rlen;
	int attempt, error;

	/*
	 * Retry like Linux icm.c (ICM_RETRIES=3): right after force-power the
	 * fw can eat the first request.  Each attempt also runs the
	 * interrupt-vs-poll probe in tb_icm_request.  On failure, log the TX
	 * and RX PICI registers: TX PICI shows whether the fw consumed our
	 * request at all (index advanced), RX PICI whether it produced a
	 * response we failed to see.
	 */
	error = ETIMEDOUT;
	for (attempt = 1; attempt <= 3; attempt++) {
		bzero(req, sizeof(req));
		req[0] = ICM_DRIVER_READY;
		rlen = sizeof(resp);
		error = tb_icm_request(icm, ICM_DRIVER_READY, req, sizeof(req),
		    resp, &rlen, 1000);
		if (error == 0)
			break;
		device_printf(icm->nsc->dev,
		    "DRIVER_READY: no ICM response (try %d/3) "
		    "tx_pici=0x%08x rx_pici=0x%08x\n", attempt,
		    nhi_read_reg(icm->nsc, icm->ring0->tx_pici_reg),
		    nhi_read_reg(icm->nsc, icm->ring0->rx_pici_reg));
	}
	if (error != 0)
		return (error);
	if ((resp[1] & ICM_FLAGS_ERROR) != 0) {
		device_printf(icm->nsc->dev, "DRIVER_READY rejected\n");
		return (EIO);
	}
	device_printf(icm->nsc->dev, "DRIVER_READY ok (len=%u)\n", rlen);
	return (0);
}

/* ---- native (SW-CM) data path -------------------------------------------- */

/*
 * Program the session's hop entries on our own router - the one duty the ICM's
 * APPROVE_XDOMAIN used to perform (and half of it it didn't - see
 * tb_icm_install_tx_hop).  Native USB4 mode: no ICM, so both directions are
 * ours via ring-0 config writes.  Entry format validated against Apple's
 * thunderbolttool decode and the MTL ICM's own RX entry (lane HOPS[8] =
 * 0x801c3802/0x0180c501): DW0 = HopID[10:0] | OutPort[16:11] | Credits[24:17]
 * | Valid[31]; DW1 = WRR/pri/flags = 0x0180c501.  The NHI adapter is port 7 on
 * MTL (Router upstream_port=7, live portdump).
 */
#define	TB_SWCM_NHI_PORT	7
#define	TB_SWCM_HOP_CREDITS	14
#define	TB_SWCM_HOP_DW1		0x0180c501
/*
 * TX hop DW1: the ICM's 0x0180c501 is its *RX* (lane->NHI) entry - weight 1,
 * priority 5, counter_enable, ingress_fc, but egress_fc=0 (bit 25).  Copied
 * verbatim onto the TX (NHI->lane) hop that leaves the egress toward the wire
 * un-flow-controlled: a slow peer's full ingress buffer makes our lane
 * adapter drop frames silently (Titan Ridge Mac: 5700+ TCP rexmits, SMB read
 * collapse).  Linux tb_dma_init_path enables egress FC on the TX DMA path
 * (drivers/thunderbolt/tunnel.c); Apple's PathSetup carries an explicit
 * egress-FC toggle (research: swcm-pathsetup).  So: TX hop gets egress_fc.
 */
#define	TB_SWCM_HOP_DW1_TX	(TB_SWCM_HOP_DW1 | (1u << 25))

static int
tb_icm_program_paths(struct tb_icm *icm)
{
	struct router_softc *rsc = icm->nsc->root_rsc;
	uint32_t buf[2];
	u_int lane;
	int error;

	if (rsc == NULL)
		return (ENXIO);
	lane = icm->peer_route_lo & 0x3f;
	if (lane == 0)
		lane = 1;

	/* RX: lane HOPS[peer's TX HopID] -> NHI(7), HopID = data RX ring. */
	buf[0] = (ICM_DATA_RX_RING & 0x7ff) | (TB_SWCM_NHI_PORT << 11) |
	    (TB_SWCM_HOP_CREDITS << 17) | (1u << 31);
	buf[1] = TB_SWCM_HOP_DW1;
	error = tb_config_write(rsc, TB_CFG_CS_PATH, lane,
	    icm->remote_tx_hopid * 2, 2, buf);
	if (error != 0) {
		device_printf(icm->nsc->dev,
		    "path RX hop write failed: %d\n", error);
		return (error);
	}

	/* TX: NHI(7) HOPS[data TX ring] -> lane, HopID = our announced one. */
	buf[0] = (icm->local_tx_hopid & 0x7ff) | (lane << 11) |
	    (TB_SWCM_HOP_CREDITS << 17) | (1u << 31);
	buf[1] = TB_SWCM_HOP_DW1_TX;
	error = tb_config_write(rsc, TB_CFG_CS_PATH, TB_SWCM_NHI_PORT,
	    ICM_DATA_TX_RING * 2, 2, buf);
	if (error != 0) {
		device_printf(icm->nsc->dev,
		    "path TX hop write failed: %d\n", error);
		return (error);
	}

	device_printf(icm->nsc->dev, "SW-CM paths: lane%u HOPS[%u] -> "
	    "NHI ring%d; NHI HOPS[%d] -> lane%u hop %u\n", lane,
	    icm->remote_tx_hopid, ICM_DATA_RX_RING, ICM_DATA_TX_RING, lane,
	    icm->local_tx_hopid);
	return (0);
}

/* Invalidate the session's hop entries (Valid=0) - the inverse of
 * tb_icm_program_paths; Apple deactivates the hop out of the table before
 * tunnel deallocation.  Sleepable context. */
static void
tb_icm_invalidate_paths(struct tb_icm *icm)
{
	struct router_softc *rsc = icm->nsc->root_rsc;
	uint32_t buf[2] = { 0, 0 };
	u_int lane;

	if (rsc == NULL || nhi_dead(icm->nsc))
		return;
	lane = icm->peer_route_lo & 0x3f;
	if (lane == 0)
		lane = 1;
	(void)tb_config_write(rsc, TB_CFG_CS_PATH, lane,
	    icm->remote_tx_hopid * 2, 2, buf);
	buf[0] = buf[1] = 0;
	(void)tb_config_write(rsc, TB_CFG_CS_PATH, TB_SWCM_NHI_PORT,
	    ICM_DATA_TX_RING * 2, 2, buf);
}

/*
 * Ordered link-down teardown (unplug / controller loss), Apple-style:
 * carrier down + ring teardown (async on our taskqueue), hop entries
 * invalidated, session state reset so a future plug re-bootstraps cleanly.
 * Called from the HCM link-event task (sleepable).
 */
void
tb_icm_link_down(struct tb_icm *icm)
{
	bool had_paths;

	if (icm == NULL || icm->dying)
		return;
	callout_stop(&icm->login_co);
	had_paths = icm->paths_approved;
	taskqueue_enqueue(icm->tq, &icm->teardown_task);
	if (had_paths)
		tb_icm_invalidate_paths(icm);
	icm->has_peer = false;
	icm->login_sent = icm->login_received = false;
	icm->login_tries = icm->logout_kicks = 0;
	device_printf(icm->nsc->dev, "link down: session torn down\n");
}

/* A plug happened: restart the discovery nudge loop so the peer re-initiates. */
void
tb_icm_link_up_hint(struct tb_icm *icm)
{
	if (icm == NULL || icm->dying)
		return;
	icm->xdkick_tries = 0;
	callout_reset(&icm->login_co, hz, tb_icm_login_timer, icm);
}

/* Peer session bootstrapped by the XDomain responder (native mode: no
 * XDOMAIN_CONNECTED event will ever come from firmware).  Interrupt context. */
static void
tb_icm_peer_discovered(void *arg, const uint8_t *peer_uuid,
    const uint8_t *local_uuid, uint32_t rhi, uint32_t rlo)
{
	struct tb_icm *icm = arg;

	memcpy(icm->peer_uuid, peer_uuid, 16);
	memcpy(icm->local_uuid, local_uuid, 16);
	icm->peer_route_hi = rhi;
	icm->peer_route_lo = rlo;
	icm->has_peer = true;
	icm->login_sent = icm->login_received = icm->paths_approved = false;
	icm->login_tries = icm->logout_kicks = 0;
	tbip_send_logout(icm);		/* reset a stale peer session */
	if (!icm->dying)
		callout_reset(&icm->login_co, hz / 2, tb_icm_login_timer, icm);
	device_printf(icm->nsc->dev, "xdomain peer (native): route %x:%x\n",
	    rhi, rlo);
}

/* ---- APPROVE_XDOMAIN + TX-hop install (deferred bring-up task) ----------- */

static int
tb_icm_approve_xdomain(struct tb_icm *icm)
{
	uint8_t req[36], resp[ICM_FRAME_SIZE];
	u_int rlen = sizeof(resp);
	int error;

	bzero(req, sizeof(req));
	req[0] = ICM_APPROVE_XDOMAIN;
	le32enc(req + 4, icm->peer_route_hi);
	le32enc(req + 8, icm->peer_route_lo);
	memcpy(req + 12, icm->peer_uuid, 16);
	le16enc(req + 28, (uint16_t)icm->local_tx_hopid);	/* transmit_path */
	le16enc(req + 30, ICM_DATA_TX_RING);			/* transmit_ring */
	le16enc(req + 32, (uint16_t)icm->remote_tx_hopid);	/* receive_path */
	le16enc(req + 34, ICM_DATA_RX_RING);			/* receive_ring */

	error = tb_icm_request(icm, ICM_APPROVE_XDOMAIN, req, sizeof(req),
	    resp, &rlen, 400);
	if (error != 0)
		return (error);
	if ((resp[1] & ICM_FLAGS_ERROR) != 0)
		return (EIO);
	device_printf(icm->nsc->dev, "APPROVE_XDOMAIN ok (tx_path=%u rx_path=%u)\n",
	    icm->local_tx_hopid, icm->remote_tx_hopid);
	return (0);
}

/*
 * Install the NHI-side TX hop the MTL ICM leaves empty (nhi_icm.c
 * nhi_install_tx_hop): CFG_WRITE (PDF_WRITE) NHI adapter (port 7) HOPS[tx ring]
 * -> out_port = first lane hop, next_hop = our TX HopID.  Fire-and-forget + a
 * short settle (the standalone driver waited for the write ack; here we skip it
 * to avoid a second config-PDF waiter - the link fails visibly if it did not
 * land).
 */
static void
tb_icm_install_tx_hop(struct tb_icm *icm)
{
	uint8_t req[20];
	uint32_t addr;
	u_int out_port;

	out_port = icm->peer_route_lo & 0x3f;
	if (out_port == 0)
		out_port = 1;
	bzero(req, sizeof(req));
	addr = ((ICM_DATA_TX_RING * 2) & 0x1fff) | (2u << 13) | (7u << 19) |
	    (0u << 25) | (1u << 27);
	le32enc(req + 8, addr);
	le32enc(req + 12, (icm->local_tx_hopid & 0x7ff) | (out_port << 11) |
	    (14u << 17) | (1u << 31));
	le32enc(req + 16, 0x0180c501);
	tb_icm_ctl_tx(icm, PDF_WRITE, req, sizeof(req));
	DELAY(5000);
	device_printf(icm->nsc->dev, "TX hop installed: NHI(7) HOPS[%d] -> "
	    "port %u hop %u\n", ICM_DATA_TX_RING, out_port, icm->local_tx_hopid);
}

/* Taskqueue: no XDOMAIN_CONNECTED arrived (integrated MTL often skips it after a
 * driver reload while the peer still believes the old session is up).  Send a
 * software replug - PROPERTIES_CHANGED x2 to the directly-attached route - so the
 * peer re-initiates and the ensuing XDOMAIN_CONNECTED / PROPERTIES_REQUEST
 * bootstraps the session.  Runs in the sleepable taskqueue (the kick busy-waits
 * ~40ms between packets, too long for the callout/softclock). */
static void
tb_icm_xdkick(void *arg, int pending __unused)
{
	struct tb_icm *icm = arg;

	if (!icm->dying && !icm->has_peer)
		tb_xdomain_kick(icm->xd, 1);
}

/* Taskqueue: LOGIN COMPLETE -> approve the tunnel, install the hop, bring up
 * the net interface.  Runs in a sleepable thread (APPROVE blocks). */
static void
tb_icm_bringup(void *arg, int pending __unused)
{
	struct tb_icm *icm = arg;

	if (icm->dying || icm->paths_approved || !icm->has_peer) {
		if (bootverbose)
			device_printf(icm->nsc->dev, "bring-up skipped: "
			    "dying=%d approved=%d has_peer=%d\n", icm->dying,
			    icm->paths_approved, icm->has_peer);
		return;
	}
	if (icm->native) {
		/* No ICM: program both hop entries ourselves. */
		if (tb_icm_program_paths(icm) != 0) {
			device_printf(icm->nsc->dev,
			    "bring-up: SW-CM path programming failed\n");
			return;
		}
	} else {
		if (tb_icm_approve_xdomain(icm) != 0) {
			device_printf(icm->nsc->dev, "bring-up: APPROVE failed\n");
			return;
		}
		tb_icm_install_tx_hop(icm);
	}
	if (icm->net == NULL || tbnet_connect(icm->net) != 0) {
		device_printf(icm->nsc->dev, "bring-up: tbnet_connect failed\n");
		return;
	}
	icm->paths_approved = true;
	device_printf(icm->nsc->dev, "tbt: link UP\n");
}

/* Taskqueue: peer went away.  tbnet_disconnect does DMA/ring
 * teardown, which must not run in the interrupt event callback - defer here.
 * Serialized with tb_icm_bringup on the single-thread taskqueue, so icm->net
 * needs no extra lock between them. */
static void
tb_icm_teardown(void *arg, int pending __unused)
{
	struct tb_icm *icm = arg;

	if (icm->net != NULL)
		tbnet_disconnect(icm->net);	/* keep the resident interface */
	icm->paths_approved = false;
}

/* ---- ThunderboltIP login (the XDomain service handler) ------------------- */

static inline uint32_t
tbip_rol32(uint32_t x, int n)
{
	return ((x << n) | (x >> (32 - n)));
}

static uint32_t
tbip_jhash2(const uint32_t *k, uint32_t length, uint32_t initval)
{
	uint32_t a, b, c;

	a = b = c = 0xdeadbeef + (length << 2) + initval;
	while (length > 3) {
		a += k[0]; b += k[1]; c += k[2];
		a -= c; a ^= tbip_rol32(c, 4);  c += b;
		b -= a; b ^= tbip_rol32(a, 6);  a += c;
		c -= b; c ^= tbip_rol32(b, 8);  b += a;
		a -= c; a ^= tbip_rol32(c, 16); c += b;
		b -= a; b ^= tbip_rol32(a, 19); a += c;
		c -= b; c ^= tbip_rol32(b, 4);  b += a;
		length -= 3; k += 3;
	}
	switch (length) {
	case 3: c += k[2]; /* FALLTHROUGH */
	case 2: b += k[1]; /* FALLTHROUGH */
	case 1: a += k[0];
		c ^= b; c -= tbip_rol32(b, 14);
		a ^= c; a -= tbip_rol32(c, 11);
		b ^= a; b -= tbip_rol32(a, 25);
		c ^= b; c -= tbip_rol32(b, 16);
		a ^= c; a -= tbip_rol32(c, 4);
		b ^= a; b -= tbip_rol32(a, 14);
		c ^= b; c -= tbip_rol32(b, 24);
		/* FALLTHROUGH */
	case 0:
		break;
	}
	return (c);
}

static void
tb_icm_gen_mac(struct tb_icm *icm)
{
	device_t dev = icm->nsc->dev;
	uint32_t k[2], hash;

	/*
	 * Stable, peer-independent MAC from the NHI's PCI identity.  We need it
	 * at driver load (the interface is resident, before any peer, so we do
	 * not yet know our XDomain UUID); the peer discovers it via ARP, so it
	 * need not be UUID-derived.
	 */
	k[0] = pci_get_devid(dev);
	k[1] = (pci_get_bus(dev) << 16) | (pci_get_slot(dev) << 8) |
	    pci_get_function(dev);
	hash = tbip_jhash2(k, 2, 0);
	icm->tbt_mac[0] = 0x02;			/* locally administered, unicast */
	icm->tbt_mac[1] = hash & 0xff;
	icm->tbt_mac[2] = (hash >> 8) & 0xff;
	icm->tbt_mac[3] = (hash >> 16) & 0xff;
	icm->tbt_mac[4] = (hash >> 24) & 0xff;
	hash = tbip_jhash2(k, 2, hash);
	icm->tbt_mac[5] = hash & 0xff;
	icm->local_tx_hopid = 8;
}

static void
tbip_hdr(struct tb_icm *icm, uint8_t *r, uint32_t length_sn, uint32_t type,
    uint32_t command_id, const uint8_t *initiator, const uint8_t *target,
    u_int total)
{
	u_int dwords = (total - 12) / 4;

	le32enc(r + 0, icm->peer_route_hi);
	le32enc(r + 4, icm->peer_route_lo);
	le32enc(r + 8, (dwords & XDP_LENGTH_MASK) | (length_sn & XDP_SN_MASK));
	memcpy(r + TBIP_OFF_UUID, tbip_svc_uuid, 16);
	memcpy(r + TBIP_OFF_INITIATOR, initiator, 16);
	memcpy(r + TBIP_OFF_TARGET, target, 16);
	le32enc(r + TBIP_OFF_TYPE, type);
	le32enc(r + TBIP_OFF_COMMAND_ID, command_id);
}

static void
tbip_send_login(struct tb_icm *icm)
{
	uint8_t r[ICM_FRAME_SIZE];
	u_int total = TBIP_HDR_LEN + 4 + 4 + 16;	/* 92 */

	bzero(r, total);
	tbip_hdr(icm, r, 0, TBIP_LOGIN, 0, icm->local_uuid, icm->peer_uuid, total);
	le32enc(r + TBIP_HDR_LEN + 0, TBIP_PROTO_VERSION);
	le32enc(r + TBIP_HDR_LEN + 4, icm->local_tx_hopid);
	tb_icm_ctl_tx(icm, PDF_XDOMAIN_RESP, r, total);
}

static void
tbip_send_logout(struct tb_icm *icm)
{
	uint8_t r[ICM_FRAME_SIZE];

	bzero(r, TBIP_HDR_LEN);
	tbip_hdr(icm, r, 0, TBIP_LOGOUT, 0, icm->local_uuid, icm->peer_uuid,
	    TBIP_HDR_LEN);
	tb_icm_ctl_tx(icm, PDF_XDOMAIN_RESP, r, TBIP_HDR_LEN);
}

/* XDomain service handler: ThunderboltIP login frames (interrupt context). */
static void
tb_icm_tbip_handle(void *ctx, const uint8_t *f, u_int len)
{
	struct tb_icm *icm = ctx;
	uint8_t r[ICM_FRAME_SIZE];
	uint32_t type, command_id, length_sn;
	u_int total;

	if (len < TBIP_HDR_LEN)
		return;
	length_sn = le32dec(f + 8);
	type = le32dec(f + TBIP_OFF_TYPE);
	command_id = le32dec(f + TBIP_OFF_COMMAND_ID);

	/* Self-heal the peer UUID from a request's initiator (peer may have
	 * rebooted with a new UUID without a fresh XDOMAIN_CONNECTED). */
	if ((type == TBIP_LOGIN || type == TBIP_LOGOUT) &&
	    memcmp(icm->peer_uuid, f + 28, 16) != 0) {
		memcpy(icm->peer_uuid, f + 28, 16);
		icm->login_sent = false;
		icm->login_tries = 0;
		icm->logout_kicks = 0;
	}

	/* Adopt the whole session from a LOGIN when we lost ours across a
	 * link flap but the peer kept its: it then skips the property
	 * exchange, so the responder bootstrap never runs, has_peer stays
	 * false, and bring-up would silently skip after LOGIN COMPLETE. */
	if (type == TBIP_LOGIN && !icm->has_peer) {
		icm->peer_route_hi = le32dec(f + 0) & 0x7fffffff;
		icm->peer_route_lo = le32dec(f + 4);
		icm->has_peer = true;
		icm->paths_approved = false;
		icm->login_tries = icm->logout_kicks = 0;
		device_printf(icm->nsc->dev,
		    "tbip: session adopted from peer LOGIN, route %x:%x\n",
		    icm->peer_route_hi, icm->peer_route_lo);
	}

	switch (type) {
	case TBIP_LOGIN:
		if (len >= TBIP_HDR_LEN + 8)
			icm->remote_tx_hopid = le32dec(f + TBIP_HDR_LEN + 4);
		total = TBIP_HDR_LEN + 4 + 8 + 4 + 16;		/* 100 */
		bzero(r, total);
		tbip_hdr(icm, r, length_sn, TBIP_LOGIN_RESPONSE, command_id,
		    icm->local_uuid, icm->peer_uuid, total);
		le32enc(r + TBIP_HDR_LEN + 0, 0);		/* status */
		memcpy(r + TBIP_HDR_LEN + 4, icm->tbt_mac, 6);
		le32enc(r + TBIP_HDR_LEN + 12, 6);		/* mac len */
		tb_icm_ctl_tx(icm, PDF_XDOMAIN_RESP, r, total);
		icm->login_received = true;
		if (!icm->login_sent)
			tbip_send_login(icm);
		break;
	case TBIP_LOGIN_RESPONSE:
		if (len >= TBIP_HDR_LEN + 12)
			memcpy(icm->peer_mac, f + TBIP_HDR_LEN + 4, 6);
		icm->login_sent = true;
		if (!icm->login_received && icm->logout_kicks < 3) {
			icm->logout_kicks++;
			tbip_send_logout(icm);
			icm->login_sent = false;
			icm->login_tries = 0;
		}
		break;
	case TBIP_LOGOUT:
		total = TBIP_HDR_LEN + 4;
		bzero(r, total);
		tbip_hdr(icm, r, length_sn, TBIP_STATUS, command_id,
		    icm->local_uuid, icm->peer_uuid, total);
		le32enc(r + TBIP_HDR_LEN, 0);
		tb_icm_ctl_tx(icm, PDF_XDOMAIN_RESP, r, total);
		icm->login_sent = icm->login_received = false;
		icm->login_tries = 0;
		break;
	case TBIP_STATUS:
		break;
	}

	if (icm->login_sent && icm->login_received && !icm->paths_approved) {
		device_printf(icm->nsc->dev, "tbip: LOGIN COMPLETE - bringing up\n");
		taskqueue_enqueue(icm->tq, &icm->bringup_task);
	}
}

/* Callout: proactively (re)send our LOGIN while a peer is up but not approved. */
static void
tb_icm_login_timer(void *arg)
{
	struct tb_icm *icm = arg;

	if (icm->dying)				/* teardown: never re-arm */
		return;
	if (!icm->has_peer) {
		/* #42: no XDOMAIN_CONNECTED yet - software-replug the peer a
		 * bounded number of times, then give up (a later fresh plug
		 * notifies normally and takes the has_peer path below). */
		if (icm->xdkick_tries < NHI_ICM_XDKICK_MAX) {
			icm->xdkick_tries++;
			taskqueue_enqueue(icm->tq, &icm->xdkick_task);
			callout_reset(&icm->login_co, 2 * hz,
			    tb_icm_login_timer, icm);
		}
		return;
	}
	if (icm->has_peer && !icm->paths_approved && !icm->login_sent &&
	    icm->login_tries < 60) {
		icm->login_tries++;
		/* Re-nudge the peer's directory read while it stays silent. */
		if ((icm->login_tries % 5) == 1)
			tb_xdomain_notify_changed(icm->xd);
		tbip_send_login(icm);
	}
	if (icm->has_peer && !icm->paths_approved)
		callout_reset(&icm->login_co, 2 * hz, tb_icm_login_timer, icm);
}

/* ---- ICM events (rxpdf[CM_EVENT], interrupt context) --------------------- */

static void
tb_icm_event_cb(void *context, union nhi_ring_desc *rdesc,
    struct nhi_cmd_frame *cmd)
{
	struct tb_icm *icm = context;
	struct nhi_rx_post_desc *desc = (struct nhi_rx_post_desc *)rdesc;
	const uint8_t *src = (const uint8_t *)cmd->data;
	uint8_t f[ICM_FRAME_SIZE];
	u_int len, ndw, j;

	len = desc->eof_len & RX_BUFFER_DESC_LEN_MASK;
	if (len < 4 || len > sizeof(f))
		return;
	len -= 4;
	ndw = len / 4;
	for (j = 0; j < ndw; j++)
		le32enc(f + j * 4, be32dec(src + j * 4));

	switch (f[0]) {
	case ICM_EVENT_XDOMAIN_CONNECTED:
		if (len < 48)
			break;
		memcpy(icm->peer_uuid, f + 8, 16);
		memcpy(icm->local_uuid, f + 24, 16);
		icm->peer_route_hi = le32dec(f + 40);
		icm->peer_route_lo = le32dec(f + 44);
		icm->has_peer = true;
		icm->login_sent = icm->login_received = icm->paths_approved = false;
		icm->login_tries = icm->logout_kicks = icm->xdkick_tries = 0;
		tb_xdomain_set_peer(icm->xd, icm->peer_uuid, icm->local_uuid,
		    icm->peer_route_hi, icm->peer_route_lo);
		tbip_send_logout(icm);		/* reset a stale peer session */
		/*
		 * Nudge the peer to (re)read our property directory - the
		 * standalone driver did this on every XDOMAIN_CONNECTED
		 * (nhi_xdomain_notify_changed); without it macOS never
		 * learns about our network service in ICM mode and ignores
		 * our LOGINs (#76).
		 */
		tb_xdomain_notify_changed(icm->xd);
		if (!icm->dying)
			callout_reset(&icm->login_co, hz / 2,
			    tb_icm_login_timer, icm);
		device_printf(icm->nsc->dev, "xdomain connected: route %x:%x\n",
		    icm->peer_route_hi, icm->peer_route_lo);
		break;
	case ICM_EVENT_XDOMAIN_DISCONNECTED:
		icm->has_peer = false;
		callout_stop(&icm->login_co);
		/* Defer the net teardown to a thread; we are in interrupt ctx. */
		if (!icm->dying)
			taskqueue_enqueue(icm->tq, &icm->teardown_task);
		device_printf(icm->nsc->dev, "xdomain disconnected\n");
		break;
	case ICM_EVENT_DEVICE_CONNECTED:
		device_printf(icm->nsc->dev, "ICM device connected: PCIe "
		    "tunnelling not implemented (Phase 5); ignored\n");
		break;
	default:
		break;
	}
}

/* ---- lifecycle ----------------------------------------------------------- */

int
tb_icm_init(struct nhi_softc *nsc, struct nhi_ring_pair *ring0,
    struct tb_icm **icmp)
{
	struct nhi_dispatch txd[1], rxd[3];
	struct tb_icm *icm;
	int error;

	icm = malloc(sizeof(*icm), M_NHI, M_NOWAIT | M_ZERO);
	if (icm == NULL)
		return (ENOMEM);
	icm->nsc = nsc;
	icm->ring0 = ring0;
	/* Native USB4 (SW-CM) mode: the HCM path attached a root router. */
	icm->native = (nsc->root_rsc != NULL);
	mtx_init(&icm->lock, "tbicm", NULL, MTX_DEF);
	callout_init(&icm->login_co, 1);
	TASK_INIT(&icm->bringup_task, 0, tb_icm_bringup, icm);
	TASK_INIT(&icm->teardown_task, 0, tb_icm_teardown, icm);
	TASK_INIT(&icm->xdkick_task, 0, tb_icm_xdkick, icm);
	icm->tq = taskqueue_create("tbicm", M_NOWAIT, taskqueue_thread_enqueue,
	    &icm->tq);
	if (icm->tq == NULL) {
		error = ENOMEM;
		goto fail;
	}
	taskqueue_start_threads(&icm->tq, 1, PI_NET, "tbicm taskq");

	/* XDomain discovery on ring 0, with us as the ThunderboltIP service. */
	if ((error = tb_xdomain_init(nsc, ring0, &icm->xd)) != 0)
		goto fail_tq;
	tb_xdomain_set_service_handler(icm->xd, tb_icm_tbip_handle, icm);
	tb_xdomain_set_peer_handler(icm->xd, tb_icm_peer_discovered, icm);

	/* ICM command responses + notifications on ring 0. */
	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = PDF_CM_RESP;  rxd[0].cb = tb_icm_resp_cb;  rxd[0].context = icm;
	rxd[1].pdf = PDF_CM_EVENT; rxd[1].cb = tb_icm_event_cb; rxd[1].context = icm;
	rxd[2].pdf = 0; rxd[2].cb = NULL; rxd[2].context = NULL;
	if ((error = nhi_register_pdf(ring0, txd, rxd)) != 0)
		goto fail_xd;

	if (icm->native) {
		device_printf(nsc->dev, "native USB4 (SW-CM): XDomain "
		    "responder + ThunderboltIP active, no ICM firmware\n");
	} else if ((error = tb_icm_driver_ready(icm)) != 0)
		goto fail_pdf;

	/*
	 * Bring up the resident ThunderboltIP interface now, at load - present
	 * with a stable MAC and link DOWN, independent of any peer (macOS-style
	 * Thunderbolt Bridge).  Carrier is raised in tb_icm_bringup() once a peer
	 * logs in; DHCP (client or server) can bind to tbtN immediately.
	 */
	tb_icm_gen_mac(icm);
	if ((error = tbnet_create(nsc, icm->tbt_mac, &icm->net)) != 0)
		goto fail_pdf;

	/*
	 * #42: start the software-replug loop.  On a fresh cable plug the ICM
	 * sends XDOMAIN_CONNECTED and the login timer takes the has_peer path;
	 * on a driver reload where the ICM stays silent, the timer kicks the
	 * peer (bounded) until it re-initiates - no physical replug needed.
	 */
	callout_reset(&icm->login_co, hz, tb_icm_login_timer, icm);

	*icmp = icm;
	return (0);

fail_pdf:
	nhi_deregister_pdf(ring0, txd, rxd);
fail_xd:
	tb_xdomain_fini(icm->xd);
fail_tq:
	taskqueue_free(icm->tq);
fail:
	callout_drain(&icm->login_co);
	mtx_destroy(&icm->lock);
	free(icm, M_NHI);
	return (error);
}

void
tb_icm_fini(struct tb_icm *icm)
{
	struct nhi_dispatch txd[1], rxd[3];

	/*
	 * Teardown order matters.  (1) Mark dying and wake any req/resp waiter
	 * so the login callout and the bring-up task stop re-arming/blocking -
	 * without this, callout_drain() on a self-rescheduling callout hangs
	 * forever (it hung a reboot).  (2) Stop ring-0 dispatch (CM_RESP/EVENT
	 * and XDomain) so no callback fires during teardown.  (3) Drain the
	 * callout and the bring-up task.  (4) Tear down the net + free.
	 */
	mtx_lock(&icm->lock);
	icm->dying = true;
	wakeup(icm);
	mtx_unlock(&icm->lock);

	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = PDF_CM_RESP;  rxd[0].cb = tb_icm_resp_cb;  rxd[0].context = icm;
	rxd[1].pdf = PDF_CM_EVENT; rxd[1].cb = tb_icm_event_cb; rxd[1].context = icm;
	rxd[2].pdf = 0; rxd[2].cb = NULL; rxd[2].context = NULL;
	nhi_deregister_pdf(icm->ring0, txd, rxd);
	tb_xdomain_fini(icm->xd);

	callout_drain(&icm->login_co);
	taskqueue_drain(icm->tq, &icm->bringup_task);
	taskqueue_drain(icm->tq, &icm->teardown_task);
	taskqueue_drain(icm->tq, &icm->xdkick_task);
	taskqueue_free(icm->tq);

	if (icm->net != NULL)
		tbnet_destroy(icm->net);
	mtx_destroy(&icm->lock);
	free(icm, M_NHI);
}

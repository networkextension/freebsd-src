/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_tbip.c - Apple ThunderboltIP networking login (Phase 3a).
 *
 * Once the peer matches our advertised "network" service, ThunderboltIP runs a
 * login handshake over ring 0: login / login-response / logout / status frames.
 * They ride the XDomain control path (PDF XDOMAIN_REQ/RESP, same transport as
 * discovery), but the protocol UUID in the header is the ThunderboltIP *service*
 * UUID (798f589e-...) rather than the discovery UUID, and the header is the
 * 68-byte thunderbolt_ip_header.  Both directions must log in; then the OS
 * allocates a data-ring pair and issues APPROVE_XDOMAIN_PATHS (next step).
 *
 * Cross-checked against Linux drivers/net/thunderbolt/main.c.  Endianness: like
 * all ring-0 frames these are byteswapped per dword by nhi_ctl_tx/rx, so build
 * and parse in native order (UUIDs/MAC memcpy'd as bytes round-trip the swap).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>	/* ticks, hz */
#include <machine/bus.h>

#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

static const uint8_t tbip_svc_uuid[16] = NHI_TBIP_SVC_UUID;

/*
 * Jenkins hash (Linux lib/jhash.h jhash2) - used only to derive our synthetic
 * MAC so it is byte-identical to the one Linux tbnet would generate from the
 * same UUID.  Not on any wire path (the peer learns our MAC via LOGIN_RESPONSE).
 */
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

/*
 * Module-wide registry of LOCAL domain UUIDs (one per NHI instance) so each
 * instance can reject frames that originate from this same machine - the
 * self/sibling loop guard in nhi_tbip_handle.
 */
static uint8_t	nhi_tbip_local_uuids[4][16];
static int	nhi_tbip_local_cnt;

static void
nhi_tbip_register_local(const uint8_t *uuid)
{
	int i;

	for (i = 0; i < nhi_tbip_local_cnt; i++)
		if (memcmp(nhi_tbip_local_uuids[i], uuid, 16) == 0)
			return;
	if (nhi_tbip_local_cnt < 4)
		memcpy(nhi_tbip_local_uuids[nhi_tbip_local_cnt++], uuid, 16);
}

bool
nhi_tbip_uuid_is_local(const uint8_t *uuid)
{
	int i;

	for (i = 0; i < nhi_tbip_local_cnt; i++)
		if (memcmp(nhi_tbip_local_uuids[i], uuid, 16) == 0)
			return (true);
	return (false);
}

void
nhi_tbip_init(struct nhi_softc *sc)
{
	uint32_t k[4], hash;
	int i;

	nhi_tbip_register_local(sc->local_uuid);
	for (i = 0; i < 4; i++)
		k[i] = le32dec(sc->local_uuid + i * 4);
	sc->tbt_mac[0] = (0 << 4) | 0x02;	/* locally administered, unicast */
	hash = tbip_jhash2(k, 4, 0);
	sc->tbt_mac[1] = hash & 0xff;
	sc->tbt_mac[2] = (hash >> 8) & 0xff;
	sc->tbt_mac[3] = (hash >> 16) & 0xff;
	sc->tbt_mac[4] = (hash >> 24) & 0xff;
	hash = tbip_jhash2(k, 4, hash);
	sc->tbt_mac[5] = hash & 0xff;
	sc->local_tx_hopid = 8;		/* fixed for our single service */
}

/* Build the 68-byte ThunderboltIP header. */
static void
tbip_hdr(struct nhi_softc *sc, uint8_t *r, uint32_t length_sn, uint32_t type,
    uint32_t command_id, const uint8_t *initiator, const uint8_t *target,
    u_int total)
{
	u_int dwords = (total - 12) / 4;	/* len excludes route_hi/lo/length_sn */

	le32enc(r + 0, sc->peer_route_hi);
	le32enc(r + 4, sc->peer_route_lo);
	le32enc(r + 8,
	    (dwords & NHI_XDP_LENGTH_MASK) | (length_sn & NHI_XDP_SN_MASK));
	memcpy(r + NHI_TBIP_OFF_UUID, tbip_svc_uuid, 16);
	memcpy(r + NHI_TBIP_OFF_INITIATOR, initiator, 16);
	memcpy(r + NHI_TBIP_OFF_TARGET, target, 16);
	le32enc(r + NHI_TBIP_OFF_TYPE, type);
	le32enc(r + NHI_TBIP_OFF_COMMAND_ID, command_id);
}

/* Initiate our own login (carries our TX HopID). */
static void
tbip_send_login(struct nhi_softc *sc)
{
	uint8_t r[NHI_CTL_FRAME_SIZE];
	u_int total = NHI_TBIP_HDR_LEN + 4 + 4 + 16;	/* proto+txpath+rsvd = 92 */

	bzero(r, total);
	tbip_hdr(sc, r, 0, NHI_TBIP_LOGIN, 0, sc->local_uuid, sc->peer_uuid,
	    total);
	le32enc(r + NHI_TBIP_HDR_LEN + 0, NHI_TBIP_PROTO_VERSION);
	le32enc(r + NHI_TBIP_HDR_LEN + 4, sc->local_tx_hopid);
	/* macOS AppleThunderboltIP sends/expects ThunderboltIP frames as
	 * XDOMAIN_RESP (pdf=7), same as our LOGIN_RESPONSE. */
	nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, total);
}

/*
 * Active P2P side: proactively (re)send our LOGIN while we have a peer but the
 * paths aren't approved yet.  ThunderboltIP is symmetric - both ends initiate,
 * so don't only react to the peer's LOGIN (Linux tbnet retries ~every 4.5 s).
 * Throttled to ~2 s; driven from the ring-0 event loop.
 */
void
nhi_tbip_start_login(struct nhi_softc *sc)
{
	if (!sc->has_peer || sc->paths_approved || sc->tbip_login_sent)
		return;
	/*
	 * Cap retries (Linux tbnet TBNET_LOGIN_RETRIES = 60): a peer that went
	 * away or rebooted has a new domain UUID and will never accept these -
	 * without a cap we spam the ring + dmesg forever if the firmware fails
	 * to send XDOMAIN_DISCONNECTED (observed on integrated MTL).
	 */
	if (sc->login_tries >= 60) {
		if (sc->login_tries == 60) {
			sc->login_tries++;
			device_printf(sc->dev,
			    "tbip: out of login retries, giving up\n");
		}
		return;
	}
	if (sc->login_last != 0 && (ticks - sc->login_last) < 2 * hz)
		return;
	sc->login_last = ticks;
	sc->login_tries++;
	if (sc->login_tries == 1 || (sc->login_tries % 15) == 0)
		device_printf(sc->dev, "tbip: LOGIN attempt %u (tx_hopid=%u)\n",
		    sc->login_tries, sc->local_tx_hopid);
	tbip_send_login(sc);
}

/*
 * Send a ThunderboltIP LOGOUT so the peer tears down any stale session for us -
 * used on (re)connect to reset a peer that still thinks it is logged in from a
 * previous cycle (e.g. after we reload the driver).  Header only (68 bytes).
 */
void
nhi_tbip_logout(struct nhi_softc *sc)
{
	uint8_t r[NHI_CTL_FRAME_SIZE];

	if (!sc->has_peer)
		return;
	bzero(r, NHI_TBIP_HDR_LEN);
	tbip_hdr(sc, r, 0, NHI_TBIP_LOGOUT, 0, sc->local_uuid, sc->peer_uuid,
	    NHI_TBIP_HDR_LEN);
	nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, NHI_TBIP_HDR_LEN);
	device_printf(sc->dev, "tbip: sent LOGOUT (reset peer session)\n");
}

void
nhi_tbip_handle(struct nhi_softc *sc, const uint8_t *f, u_int len)
{
	uint8_t r[NHI_CTL_FRAME_SIZE];
	uint32_t type, command_id, length_sn;
	u_int total;

	if (len < NHI_TBIP_HDR_LEN)
		return;
	/*
	 * Self-frame guard: the fabric can loop our own or our SIBLING NHI's
	 * frames back at us; without this we complete a login with ourselves
	 * (observed: nhi0's proactive LOGINs arriving at nhi1, "LOGIN
	 * COMPLETE" and a tbt interface talking to a mirror).  Check the
	 * initiator against every local domain UUID this module has seen.
	 */
	if (nhi_tbip_uuid_is_local(f + 28))
		return;
	length_sn = le32dec(f + 8);
	type = le32dec(f + NHI_TBIP_OFF_TYPE);
	command_id = le32dec(f + NHI_TBIP_OFF_COMMAND_ID);

	/*
	 * Self-heal the peer UUID from the frame's initiator field: the peer
	 * may have rebooted (new domain UUID) without the firmware raising a
	 * new XDOMAIN_CONNECTED - replies addressed to the stale UUID are
	 * silently dropped by macOS.  Always answer the UUID that asked.
	 * Only for REQUEST types (initiator == sender); in responses the
	 * initiator field echoes us.
	 */
	if ((type == NHI_TBIP_LOGIN || type == NHI_TBIP_LOGOUT) &&
	    memcmp(sc->peer_uuid, f + 28, 16) != 0) {
		device_printf(sc->dev,
		    "tbip: peer UUID changed (%02x%02x%02x%02x-... -> "
		    "%02x%02x%02x%02x-...), adopting\n",
		    sc->peer_uuid[0], sc->peer_uuid[1], sc->peer_uuid[2],
		    sc->peer_uuid[3], f[28], f[29], f[30], f[31]);
		memcpy(sc->peer_uuid, f + 28, 16);
		/* new peer session: restart our side of the handshake */
		sc->tbip_login_sent = false;
		sc->login_tries = 0;
		sc->login_last = 0;
		sc->logout_kicks = 0;
	}

	switch (type) {
	case NHI_TBIP_LOGIN:
		/* req body: proto_version@hdr, transmit_path@hdr+4 */
		if (len >= NHI_TBIP_HDR_LEN + 8)
			sc->remote_tx_hopid = le32dec(f + NHI_TBIP_HDR_LEN + 4);
		/*
		 * Reply LOGIN_RESPONSE with our MAC.  initiator = OUR uuid,
		 * target = the peer - tbnet_login_response passes
		 * (xd->local_uuid, xd->remote_uuid) and macOS validates
		 * initiator==remote && target==local, silently dropping
		 * anything else (we had these swapped: every response we ever
		 * sent was discarded and the Mac retried login forever).
		 */
		total = NHI_TBIP_HDR_LEN + 4 + 8 + 4 + 16;	/* = 100 */
		bzero(r, total);
		tbip_hdr(sc, r, length_sn, NHI_TBIP_LOGIN_RESPONSE, command_id,
		    sc->local_uuid, sc->peer_uuid, total);
		le32enc(r + NHI_TBIP_HDR_LEN + 0, 0);		/* status */
		memcpy(r + NHI_TBIP_HDR_LEN + 4, sc->tbt_mac, 6);
		le32enc(r + NHI_TBIP_HDR_LEN + 12, 6);	/* receiver_mac_len */
		nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, total);
		sc->tbip_login_received = true;
		device_printf(sc->dev, "tbip: LOGIN (peer tx_hopid=%u) -> "
		    "LOGIN_RESPONSE mac %6D\n", sc->remote_tx_hopid,
		    sc->tbt_mac, ":");
		if (!sc->tbip_login_sent)
			tbip_send_login(sc);
		break;
	case NHI_TBIP_LOGIN_RESPONSE:
		if (len >= NHI_TBIP_HDR_LEN + 12)
			memcpy(sc->peer_mac, f + NHI_TBIP_HDR_LEN + 4, 6);
		sc->tbip_login_sent = true;
		device_printf(sc->dev, "tbip: LOGIN_RESPONSE peer mac %6D\n",
		    sc->peer_mac, ":");
		/*
		 * Half-open: the peer answers our LOGIN but never sends its
		 * own (it still holds a session from before we reloaded, so
		 * we don't know its transmit_path).  Kick it with a LOGOUT -
		 * it drops the stale session and re-LOGINs, giving us its
		 * tx HopID.  Rate-limited to avoid a logout/response loop.
		 */
		if (!sc->tbip_login_received && sc->logout_kicks < 3) {
			sc->logout_kicks++;
			device_printf(sc->dev,
			    "tbip: half-open (peer never logged in) - "
			    "LOGOUT kick %u\n", sc->logout_kicks);
			nhi_tbip_logout(sc);
			sc->tbip_login_sent = false;
			sc->login_tries = 0;
			sc->login_last = 0;
		}
		break;
	case NHI_TBIP_LOGOUT:
		/* Ack with STATUS(0) - tbnet replies so the peer's logout
		 * retries stop - then clear our session state.  Same uuid
		 * direction as all tbnet frames: initiator=us, target=peer. */
		total = NHI_TBIP_HDR_LEN + 4;
		bzero(r, total);
		tbip_hdr(sc, r, length_sn, NHI_TBIP_STATUS, command_id,
		    sc->local_uuid, sc->peer_uuid, total);
		le32enc(r + NHI_TBIP_HDR_LEN, 0);	/* status = ok */
		nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, total);
		sc->tbip_login_sent = false;
		sc->tbip_login_received = false;
		sc->login_tries = 0;
		device_printf(sc->dev, "tbip: LOGOUT -> STATUS ack\n");
		break;
	case NHI_TBIP_STATUS:
		break;			/* peer acking our LOGOUT */
	default:
		device_printf(sc->dev, "tbip: type=%u len=%u\n", type, len);
		break;
	}

	if (sc->tbip_login_sent && sc->tbip_login_received && !sc->paths_approved) {
		device_printf(sc->dev, "tbip: LOGIN COMPLETE (both directions) - "
		    "approving paths + bringing up tbt%d\n",
		    device_get_unit(sc->dev));
		nhi_tbt_connect(sc);
	}
}

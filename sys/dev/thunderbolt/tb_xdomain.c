/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tb_xdomain.c - Thunderbolt XDomain discovery responder, as a shared layer on
 * the in-tree NHI transport (ROADMAP.md Step 2).
 *
 * XDomain is the host-to-host discovery/property protocol that rides ring 0
 * (PDF XDOMAIN_REQ/RESP): a peer (e.g. a Mac) asks for our UUID and reads our
 * property directory to identify the host and its network service, which is what
 * brings up "Thunderbolt Bridge".  It sits ABOVE whichever connection manager
 * (HCM router.c | ICM tb_icm) approved the link: the CM tells us the peer/local
 * UUIDs + route (tb_xdomain_set_peer); we serve discovery here.
 *
 * Ported from the standalone nhi_icm driver (sys/dev/nhi/nhi_xdomain.c).  The
 * frame builders and the dispatch are unchanged; only the wire I/O differs.  The
 * in-tree transport does not byteswap ring-0 payloads (unlike our nhi_ctl_tx),
 * so - exactly as router.c does for config packets - we format the frame
 * ourselves: native dwords -> big-endian per dword + CRC32C, then
 * nhi_tx_schedule; and be32->native on RX.  Wire formats: USB4 Inter-Domain
 * Service Spec 2.0.
 *
 * UNTESTED: written against the API, not yet compiled or run (board offline).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/gsb_crc32.h>
#include <sys/jail.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <machine/bus.h>

#include <dev/thunderbolt/nhi_reg.h>
#include <dev/thunderbolt/nhi_var.h>
#include <dev/thunderbolt/tb_xdomain.h>
#include <dev/thunderbolt/tb_debug.h>

/*
 * XDomain / property-directory protocol constants (Apple/USB4 Inter-Domain
 * Service Spec; not in the in-tree NHI headers).  Kept with the same names as
 * the nhi_icm source so the ported builders are verbatim.
 */
#define	XDP_HDR_LEN	32	/* route_hi(4)+route_lo(4)+length_sn(4)+uuid(16)+type(4) */
#define	NHI_XDP_LENGTH_MASK	0x0000003f
#define	NHI_XDP_SN_MASK		0x18000000
#define	NHI_XDP_UUID	{ 0xb6,0x38,0xd7,0x0e, 0x42,0xff, 0x40,0xbb, \
			  0x97,0xc2, 0x90,0xe2,0xc0,0xb2,0xff,0x07 }
#define	NHI_XDP_UUID_REQUEST			12
#define	NHI_XDP_UUID_RESPONSE			2
#define	NHI_XDP_PROPERTIES_REQUEST		3
#define	NHI_XDP_PROPERTIES_RESPONSE		4
#define	NHI_XDP_PROPERTIES_CHANGED_REQUEST	5
#define	NHI_XDP_PROPERTIES_CHANGED_RESPONSE	6
#define	NHI_XDP_PROP_CHUNK_DWORDS		40
#define	NHI_PROP_MAGIC		0x55584401u
#define	NHI_PROP_TYPE_VALUE	0x76
#define	NHI_PROP_TYPE_TEXT	0x74
#define	NHI_PROP_TYPE_DIRECTORY	0x44
#define	NHI_TBNET_PRTCID	1
#define	NHI_TBNET_PRTCVERS	1
#define	NHI_TBNET_PRTCREVS	1
#define	NHI_TBNET_PRTCSTNS	(0x1 | 0x2 | 0x4)	/* E2E|MATCH_FRAGS_ID|64K */
#define	NHI_NET_DIR_UUID { 0xc6,0x61,0x89,0xca, 0x1c,0xce, 0x41,0x95, \
			   0xbd,0xb8, 0x49,0x59,0x2e,0x5f,0x5a,0x4f }
#define	NHI_TBIP_SVC_UUID { 0x79,0x8f,0x58,0x9e, 0x36,0x16, 0x8a,0x47, \
			    0x97,0xc6, 0x56,0x64,0xa9,0x20,0xc8,0xdd }
#define	XDP_FRAME_SIZE	256

/* PDF_XDOMAIN_REQ / PDF_XDOMAIN_RESP (0x06 / 0x07) come from nhi_reg.h.  Frames
 * on ring 0 use SOF==EOF==pdf (control), like the CM's config packets. */

/* tb_xdomain_service_cb (for non-discovery service-UUID frames) is in the
 * public header. */

struct tb_xdomain {
	struct nhi_softc	*nsc;
	struct nhi_ring_pair	*ring0;		/* transport control ring */

	bool			has_peer;
	uint8_t			peer_uuid[16];
	uint8_t			local_uuid[16];
	uint32_t		local_route_hi, local_route_lo;
	uint32_t		peer_route_hi, peer_route_lo;
	uint32_t		kick_route;

	uint32_t		prop_block[128];
	u_int			prop_dwords;
	u_int			prop_generation;

	tb_xdomain_service_cb	*service_cb;	/* service-UUID frames (tbip login) */
	void			*service_ctx;
	tb_xdomain_peer_cb	*peer_cb;	/* session bootstrapped (native CM) */
	void			*peer_ctx;
};

static void tb_xdp_dispatch(struct tb_xdomain *, const uint8_t *, u_int);

/*
 * Send an XDomain control frame on ring 0.  The frame is built by the caller in
 * native dword order (le32enc into a byte buffer); here we byteswap each dword
 * to big-endian, append the big-endian CRC32C, and schedule it - the in-tree
 * transport DMAs the payload raw, so the framing is our responsibility (same as
 * router.c for config packets).  Control frame => SOF==EOF==pdf.
 */
static int
tb_xdp_ctl_tx(struct tb_xdomain *xd, uint8_t pdf, const uint8_t *data, u_int len)
{
	struct nhi_cmd_frame *cmd;
	uint8_t *fbuf;
	uint32_t crc;
	u_int i;

	if (len + 4 > XDP_FRAME_SIZE)
		return (EINVAL);
	cmd = nhi_alloc_tx_frame(xd->ring0);
	if (cmd == NULL)
		return (ENOBUFS);
	fbuf = (uint8_t *)cmd->data;

	/* native (le) dwords -> big-endian on the wire */
	for (i = 0; i < len; i += 4)
		be32enc(fbuf + i, le32dec(data + i));
	crc = ~calculate_crc32c(~0U, fbuf, len);
	be32enc(fbuf + len, crc);

	cmd->req_len = len + 4;
	cmd->pdf = pdf;		/* EOF; sof defaults to pdf (control frame) */
	cmd->sof = 0;
	if (nhi_tx_schedule(xd->ring0, cmd) != 0) {
		nhi_free_tx_frame(xd->ring0, cmd);
		return (EIO);
	}
	return (0);
}

/* ---- property directory + response builders (verbatim from nhi_xdomain.c) -- */

static uint32_t
xdp_word(const char *s, u_int slen, u_int off)
{
	uint32_t v = 0;
	u_int i;

	for (i = 0; i < 4; i++) {
		uint8_t c = (off + i < slen) ? (uint8_t)s[off + i] : 0;
		v = (v << 8) | c;
	}
	return (v);
}

static void
xdp_put_entry(uint32_t *b, u_int e, const char *key, u_int length, u_int type,
    uint32_t value)
{
	u_int klen = strlen(key);

	b[e + 0] = xdp_word(key, klen, 0);
	b[e + 1] = xdp_word(key, klen, 4);
	b[e + 2] = (uint32_t)(length & 0xffff) | ((uint32_t)type << 24);
	b[e + 3] = value;
}

static void
xdp_put_text(uint32_t *b, u_int off, const char *s)
{
	u_int dw = howmany(strlen(s) + 1, 4), i;

	for (i = 0; i < dw; i++)
		b[off + i] = xdp_word(s, strlen(s), i * 4);
}

static void
tb_xdomain_build_dir(struct tb_xdomain *xd)
{
	static const uint8_t net_uuid[16] = NHI_NET_DIR_UUID;
	uint32_t *b = xd->prop_block;
	static const char vname[] = "FreeBSD";
	char host[24];
	u_int e, fb_off, hn_off, net_off, fb_dw, hn_dw, net_len, total;

	getcredhostname(curthread->td_ucred, host, sizeof(host));
	if (host[0] == '\0')
		strlcpy(host, "freebsd", sizeof(host));

	fb_dw = howmany(strlen(vname) + 1, 4);
	hn_dw = howmany(strlen(host) + 1, 4);
	net_len = 4 + 4 * 4;

	fb_off = 30;
	hn_off = fb_off + fb_dw;
	net_off = hn_off + hn_dw;
	total = net_off + net_len;

	bzero(b, sizeof(xd->prop_block));
	b[0] = NHI_PROP_MAGIC;
	b[1] = fb_off - 2;

	e = 2;
	xdp_put_entry(b, e, "vendorid", 1, NHI_PROP_TYPE_VALUE, 0x1d6b); e += 4;
	xdp_put_entry(b, e, "deviceid", 1, NHI_PROP_TYPE_VALUE, 0x1); e += 4;
	xdp_put_entry(b, e, "devicerv", 1, NHI_PROP_TYPE_VALUE, 0x80000100); e += 4;
	xdp_put_entry(b, e, "maxhopid", 1, NHI_PROP_TYPE_VALUE, 0x8); e += 4;
	xdp_put_entry(b, e, "vendorid", fb_dw, NHI_PROP_TYPE_TEXT, fb_off); e += 4;
	xdp_put_entry(b, e, "deviceid", hn_dw, NHI_PROP_TYPE_TEXT, hn_off); e += 4;
	xdp_put_entry(b, e, "network", net_len, NHI_PROP_TYPE_DIRECTORY, net_off);

	xdp_put_text(b, fb_off, vname);
	xdp_put_text(b, hn_off, host);

	memcpy(&b[net_off], net_uuid, sizeof(net_uuid));
	e = net_off + 4;
	xdp_put_entry(b, e, "prtcid", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCID); e += 4;
	xdp_put_entry(b, e, "prtcvers", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCVERS); e += 4;
	xdp_put_entry(b, e, "prtcrevs", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCREVS); e += 4;
	xdp_put_entry(b, e, "prtcstns", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCSTNS);

	xd->prop_dwords = total;
	xd->prop_generation = 1;
	device_printf(xd->nsc->dev,
	    "xdomain: property dir built (%u dwords, host=\"%s\", +network svc)\n",
	    total, host);
}

static void
xdp_hdr(uint8_t *r, uint32_t rhi, uint32_t rlo, uint32_t length_sn,
    u_int total, uint32_t type)
{
	static const uint8_t xdp_uuid[16] = NHI_XDP_UUID;
	u_int dwords = (total - 12) / 4;

	le32enc(r + 0, rhi);
	le32enc(r + 4, rlo);
	le32enc(r + 8, (dwords & NHI_XDP_LENGTH_MASK) | (length_sn & NHI_XDP_SN_MASK));
	memcpy(r + 12, xdp_uuid, 16);
	le32enc(r + 28, type);
}

static u_int
xdp_uuid_response(struct tb_xdomain *xd, uint8_t *r, uint32_t rhi, uint32_t rlo,
    uint32_t length_sn)
{
	u_int total = XDP_HDR_LEN + 16 + 8;

	xdp_hdr(r, rhi, rlo, length_sn, total, NHI_XDP_UUID_RESPONSE);
	memcpy(r + 32, xd->local_uuid, 16);
	le32enc(r + 48, xd->local_route_hi);
	le32enc(r + 52, xd->local_route_lo);
	return (total);
}

static u_int
xdp_props_response(struct tb_xdomain *xd, uint8_t *r, uint32_t rhi, uint32_t rlo,
    uint32_t length_sn, const uint8_t *peer_uuid, u_int off)
{
	u_int chunk, total, i;

	chunk = (off < xd->prop_dwords) ?
	    MIN(xd->prop_dwords - off, NHI_XDP_PROP_CHUNK_DWORDS) : 0;
	total = XDP_HDR_LEN + 16 + 16 + 8 + chunk * 4;

	xdp_hdr(r, rhi, rlo, length_sn, total, NHI_XDP_PROPERTIES_RESPONSE);
	memcpy(r + 32, xd->local_uuid, 16);
	memcpy(r + 48, peer_uuid, 16);
	le32enc(r + 64, (off & 0xffff) | ((xd->prop_dwords & 0xffff) << 16));
	le32enc(r + 68, xd->prop_generation);
	for (i = 0; i < chunk; i++)
		le32enc(r + 72 + i * 4, xd->prop_block[off + i]);
	return (total);
}

/* ---- dispatch (verbatim from nhi_xdomain_handle) ------------------------- */

static void
tb_xdp_dispatch(struct tb_xdomain *xd, const uint8_t *f, u_int len)
{
	static const uint8_t tbip_uuid[16] = NHI_TBIP_SVC_UUID;
	uint8_t r[XDP_FRAME_SIZE];
	uint32_t req_rhi, req_rlo, rhi, rlo, length_sn, type;
	u_int rlen, off;

	if (len < XDP_HDR_LEN)
		return;

	/* Service-UUID frames (ThunderboltIP login) use a different header and
	 * are handed to the registered service handler, not decoded here. */
	if (memcmp(f + 12, tbip_uuid, 16) == 0) {
		if (xd->service_cb != NULL)
			xd->service_cb(xd->service_ctx, f, len);
		return;
	}
	req_rhi = le32dec(f + 0);
	req_rlo = le32dec(f + 4);
	length_sn = le32dec(f + 8);
	type = le32dec(f + 28);

	rhi = req_rhi & 0x7fffffff;
	rlo = req_rlo;
	tb_debug(xd->nsc, DBG_INIT,
	    "xdomain rx type=%u len=%u reqroute=%x:%x peerroute=%x:%x\n",
	    type, len, req_rhi, req_rlo, rhi, rlo);

	switch (type) {
	case NHI_XDP_UUID_REQUEST:
		rlen = xdp_uuid_response(xd, r, rhi, rlo, length_sn);
		tb_xdp_ctl_tx(xd, PDF_XDOMAIN_RESP, r, rlen);
		break;
	case NHI_XDP_PROPERTIES_REQUEST:
		if (len < 68)
			break;
		if (!xd->has_peer) {
			/* Bootstrap the session from the request (integrated MTL
			 * often skips XDOMAIN_CONNECTED after a reload): src =
			 * peer UUID, dst = our UUID, over the answer route. */
			memcpy(xd->peer_uuid, f + 32, 16);
			memcpy(xd->local_uuid, f + 48, 16);
			xd->peer_route_hi = rhi;
			xd->peer_route_lo = rlo;
			xd->local_route_hi = rhi;
			xd->local_route_lo = rlo;
			xd->has_peer = true;
			tb_xdomain_build_dir(xd);
			device_printf(xd->nsc->dev,
			    "xdomain: session bootstrapped from peer request\n");
			if (xd->peer_cb != NULL)
				xd->peer_cb(xd->peer_ctx, xd->peer_uuid,
				    xd->local_uuid, rhi, rlo);
		}
		off = le32dec(f + 64) & 0xffff;
		rlen = xdp_props_response(xd, r, rhi, rlo, length_sn, f + 32, off);
		tb_xdp_ctl_tx(xd, PDF_XDOMAIN_RESP, r, rlen);
		tb_debug(xd->nsc, DBG_INIT,
		    "xdomain: PROPERTIES_REQUEST off=%u -> %u bytes\n", off, rlen);
		break;
	case NHI_XDP_PROPERTIES_CHANGED_REQUEST:
		xdp_hdr(r, rhi, rlo, length_sn, XDP_HDR_LEN,
		    NHI_XDP_PROPERTIES_CHANGED_RESPONSE);
		tb_xdp_ctl_tx(xd, PDF_XDOMAIN_RESP, r, XDP_HDR_LEN);
		break;
	case NHI_XDP_PROPERTIES_CHANGED_RESPONSE:
		break;
	default:
		tb_debug(xd->nsc, DBG_INIT, "xdomain: unhandled type=%u len=%u\n",
		    type, len);
		break;
	}
}

/*
 * RX PDF callback (ring 0, interrupt context): the transport hands us the raw
 * big-endian frame in cmd->data.  Un-byteswap into a native buffer, strip the
 * 4-byte CRC, and dispatch.
 */
static void
tb_xdp_rx_cb(void *context, union nhi_ring_desc *rdesc,
    struct nhi_cmd_frame *cmd)
{
	struct tb_xdomain *xd = context;
	struct nhi_rx_post_desc *desc = (struct nhi_rx_post_desc *)rdesc;
	uint8_t f[XDP_FRAME_SIZE];
	const uint8_t *src = (const uint8_t *)cmd->data;
	u_int len, ndw, j;

	len = desc->eof_len & RX_BUFFER_DESC_LEN_MASK;
	if (len < 4 || len > sizeof(f))
		return;
	len -= 4;			/* drop the CRC footer */
	ndw = len / 4;
	for (j = 0; j < ndw; j++)	/* big-endian -> native per dword */
		le32enc(f + j * 4, be32dec(src + j * 4));
	tb_xdp_dispatch(xd, f, len);
}

/* ---- active requests (verbatim logic, tb_xdp_ctl_tx wire I/O) ------------ */

void
tb_xdomain_notify_changed(struct tb_xdomain *xd)
{
	uint8_t r[64];
	u_int total = XDP_HDR_LEN + 16;

	if (!xd->has_peer)
		return;
	bzero(r, total);
	xdp_hdr(r, xd->peer_route_hi, xd->peer_route_lo, 0, total,
	    NHI_XDP_PROPERTIES_CHANGED_REQUEST);
	memcpy(r + 32, xd->local_uuid, 16);
	tb_xdp_ctl_tx(xd, PDF_XDOMAIN_REQ, r, total);
}

void
tb_xdomain_kick(struct tb_xdomain *xd, uint32_t route_lo)
{
	uint8_t r[64];
	u_int total = XDP_HDR_LEN + 16;
	uint32_t rlo = route_lo != 0 ? route_lo : 1;

	bzero(r, total);
	xdp_hdr(r, 0, rlo, 0, total, NHI_XDP_PROPERTIES_CHANGED_REQUEST);
	memcpy(r + 32, xd->local_uuid, 16);
	tb_xdp_ctl_tx(xd, PDF_XDOMAIN_REQ, r, total);
	DELAY(20000);
	memset(r + 32, 0xff, 16);
	tb_xdp_ctl_tx(xd, PDF_XDOMAIN_REQ, r, total);
}

/* ---- public lifecycle ---------------------------------------------------- */

/* The CM sets the peer/local UUIDs + route on XDOMAIN_CONNECTED and builds our
 * directory.  (Discovery can also bootstrap from a PROPERTIES_REQUEST.) */
void
tb_xdomain_set_peer(struct tb_xdomain *xd, const uint8_t *peer_uuid,
    const uint8_t *local_uuid, uint32_t route_hi, uint32_t route_lo)
{
	memcpy(xd->peer_uuid, peer_uuid, 16);
	memcpy(xd->local_uuid, local_uuid, 16);
	xd->local_route_hi = xd->peer_route_hi = route_hi;
	xd->local_route_lo = xd->peer_route_lo = route_lo;
	xd->has_peer = true;
	tb_xdomain_build_dir(xd);
}

void
tb_xdomain_set_service_handler(struct tb_xdomain *xd,
    tb_xdomain_service_cb *cb, void *ctx)
{
	xd->service_cb = cb;
	xd->service_ctx = ctx;
}

void
tb_xdomain_set_peer_handler(struct tb_xdomain *xd,
    tb_xdomain_peer_cb *cb, void *ctx)
{
	xd->peer_cb = cb;
	xd->peer_ctx = ctx;
}

int
tb_xdomain_init(struct nhi_softc *nsc, struct nhi_ring_pair *ring0,
    struct tb_xdomain **xdp)
{
	struct nhi_dispatch txd[1], rxd[3];
	struct tb_xdomain *xd;
	int error;

	xd = malloc(sizeof(*xd), M_NHI, M_NOWAIT | M_ZERO);
	if (xd == NULL)
		return (ENOMEM);
	xd->nsc = nsc;
	xd->ring0 = ring0;

	/* Receive both XDomain request and response PDFs on ring 0. */
	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = PDF_XDOMAIN_REQ;  rxd[0].cb = tb_xdp_rx_cb; rxd[0].context = xd;
	rxd[1].pdf = PDF_XDOMAIN_RESP; rxd[1].cb = tb_xdp_rx_cb; rxd[1].context = xd;
	rxd[2].pdf = 0; rxd[2].cb = NULL; rxd[2].context = NULL;
	if ((error = nhi_register_pdf(ring0, txd, rxd)) != 0) {
		free(xd, M_NHI);
		return (error);
	}
	*xdp = xd;
	return (0);
}

void
tb_xdomain_fini(struct tb_xdomain *xd)
{
	struct nhi_dispatch txd[1], rxd[3];

	txd[0].pdf = 0; txd[0].cb = NULL; txd[0].context = NULL;
	rxd[0].pdf = PDF_XDOMAIN_REQ;  rxd[0].cb = tb_xdp_rx_cb; rxd[0].context = xd;
	rxd[1].pdf = PDF_XDOMAIN_RESP; rxd[1].cb = tb_xdp_rx_cb; rxd[1].context = xd;
	rxd[2].pdf = 0; rxd[2].cb = NULL; rxd[2].context = NULL;
	nhi_deregister_pdf(xd->ring0, txd, rxd);
	free(xd, M_NHI);
}

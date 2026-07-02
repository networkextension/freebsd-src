/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_xdomain.c - Thunderbolt XDomain discovery responder (Phase 2, V2.1).
 *
 * When a remote host (e.g. a Mac) is connected the firmware ICM raises an
 * XDOMAIN_CONNECTED event (handled in nhi_icm.c, which stashes the peer/local
 * UUIDs + route).  The peer then drives the XDomain discovery protocol on
 * ring 0 (PDF XDOMAIN_REQ): a UUID request, then offset-chunked PROPERTIES
 * reads of our property directory.  We answer here so the peer can identify
 * our host (and, once the network service is advertised, bring up Thunderbolt
 * Bridge).  Wire formats cross-checked against Linux drivers/thunderbolt/
 * xdomain.c, property.c and tb_msgs.h.
 *
 * Endianness: nhi_ctl_rx delivers payloads as native dwords and nhi_ctl_tx
 * byteswaps native->big-endian per dword (matching Linux cpu_to_be32_array).
 * So everything here is built/parsed in NATIVE order; keys and text are packed
 * big-endian-into-a-native-u32 so the transport's byteswap puts the bytes on
 * the wire in order.  This lives in nhi(4) for now; it lifts into tb(4) later.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/jail.h>
#include <sys/proc.h>
#include <machine/bus.h>

#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

/* tb_xdp_header = route_hi(4) + route_lo(4) + length_sn(4) + uuid(16) + type(4) */
#define	XDP_HDR_LEN	32

/* Pack up to 4 chars of key[off..] big-endian into a native u32 (so the TX
 * per-dword byteswap lands the bytes on the wire in order).  Also used for the
 * NUL-terminated TEXT leaf data. */
static uint32_t
xdp_word(const char *s, u_int slen, u_int off)
{
	uint32_t v = 0;
	u_int i;

	/*
	 * Pack chars BIG-endian (s[off] in the top byte).  key_hi/key_lo and text
	 * are read big-endian by the peer, and the ring-0 TX byteswap lands the
	 * bytes on the wire in string order.  (Confirmed on macOS: LE packing made
	 * the "network" key show as "wten" - reversed - so AppleThunderboltIP
	 * never matched the service.)
	 */
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
	/*
	 * Third dword is struct tb_property_entry's { u16 length; u8 reserved;
	 * u8 type } in little-endian byte layout: length in the low 16 bits,
	 * type in the top byte (Linux drivers/thunderbolt/property.c).
	 */
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

/*
 * Build our property directory (host identity only for now: vendor/device id +
 * names + maxhopid).  The network service directory is the next slice.  Layout:
 * root header(2 dwords) + entries(4 dwords each) + TEXT leaf data.
 */
void
nhi_xdomain_build_dir(struct nhi_softc *sc)
{
	static const uint8_t net_uuid[16] = NHI_NET_DIR_UUID;
	uint32_t *b = sc->prop_block;
	static const char vname[] = "FreeBSD";
	char host[24];
	u_int e, fb_off, hn_off, net_off, fb_dw, hn_dw, net_len, total;

	getcredhostname(curthread->td_ucred, host, sizeof(host));
	if (host[0] == '\0')
		strlcpy(host, "freebsd", sizeof(host));

	fb_dw = howmany(strlen(vname) + 1, 4);
	hn_dw = howmany(strlen(host) + 1, 4);
	net_len = 4 + 4 * 4;		/* subdir: uuid(4) + 4 VALUE entries(16) */

	/* 7 entries -> data region after dword 2 + 7*4 = 30 */
	fb_off = 30;
	hn_off = fb_off + fb_dw;
	net_off = hn_off + hn_dw;
	total = net_off + net_len;

	bzero(b, sizeof(sc->prop_block));
	b[0] = NHI_PROP_MAGIC;
	/*
	 * Root directory length = the ENTRIES region only (dwords), NOT the whole
	 * block.  The peer computes num_entries = length/4 (Linux property.c), so
	 * counting the text data + network subdir here makes it parse those as
	 * bogus entries and reject the directory ("Unknown Cross Domain Device").
	 * Entries occupy dwords 2..fb_off-1, i.e. (fb_off - 2) dwords.
	 */
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

	/* network service subdirectory: uuid header + prtc* immediates. */
	memcpy(&b[net_off], net_uuid, sizeof(net_uuid));
	e = net_off + 4;
	xdp_put_entry(b, e, "prtcid", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCID); e += 4;
	xdp_put_entry(b, e, "prtcvers", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCVERS); e += 4;
	xdp_put_entry(b, e, "prtcrevs", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCREVS); e += 4;
	xdp_put_entry(b, e, "prtcstns", 1, NHI_PROP_TYPE_VALUE, NHI_TBNET_PRTCSTNS);

	sc->prop_dwords = total;
	sc->prop_generation = 1;
	device_printf(sc->dev,
	    "xdomain: property dir built (%u dwords, host=\"%s\", +network svc)\n",
	    total, host);
}

static void xdp_hdr(uint8_t *, uint32_t, uint32_t, uint32_t, u_int, uint32_t);

/*
 * Tell the peer our properties changed so it re-reads our directory (e.g. after
 * a driver reload, where the firmware replays XDOMAIN_CONNECTED to us but the
 * peer sees no link change).  PROPERTIES_CHANGED_REQUEST = xdp header + our
 * src_uuid (48 bytes) — the same frame the Mac sends us.
 */
void
nhi_xdomain_notify_changed(struct nhi_softc *sc)
{
	uint8_t r[64];
	u_int total = XDP_HDR_LEN + 16;	/* hdr + src_uuid */

	if (!sc->has_peer)
		return;
	bzero(r, total);
	xdp_hdr(r, sc->peer_route_hi, sc->peer_route_lo, 0, total,
	    NHI_XDP_PROPERTIES_CHANGED_REQUEST);
	memcpy(r + 32, sc->local_uuid, 16);
	nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_REQ, r, total);
	device_printf(sc->dev, "xdomain: sent PROPERTIES_CHANGED (re-read nudge)\n");
}

/*
 * Active P2P read: request the PEER's property directory (offset 0) so we can
 * dump macOS's own known-good property block as a byte-exact reference for our
 * serializer.  The PROPERTIES_RESPONSE comes back on ring 0 as pdf=7 and is
 * hexdumped by the event loop.  tb_xdp_properties = hdr(32) + src_uuid(16) +
 * dst_uuid(16) + u16 offset + u16 reserved = 68 bytes.
 */
void
nhi_xdomain_request_props(struct nhi_softc *sc)
{
	uint8_t r[72];
	u_int total = XDP_HDR_LEN + 16 + 16 + 4;	/* = 68 */

	if (!sc->has_peer)
		return;
	bzero(r, total);
	/* forward-declared below; fill header then src/dst/offset */
	le32enc(r + 0, sc->peer_route_hi);
	le32enc(r + 4, sc->peer_route_lo);
	le32enc(r + 8, ((total - 12) / 4) & NHI_XDP_LENGTH_MASK);
	{
		static const uint8_t xu[16] = NHI_XDP_UUID;
		memcpy(r + 12, xu, 16);
	}
	le32enc(r + 28, NHI_XDP_PROPERTIES_REQUEST);
	memcpy(r + 32, sc->local_uuid, 16);	/* src = us */
	memcpy(r + 48, sc->peer_uuid, 16);	/* dst = peer */
	le32enc(r + 64, 0);			/* offset 0, reserved 0 */
	nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_REQ, r, total);
	device_printf(sc->dev, "xdomain: sent PROPERTIES_REQUEST to peer "
	    "(reference read)\n");
}

/* Common XDomain response header. */
static void
xdp_hdr(uint8_t *r, uint32_t rhi, uint32_t rlo, uint32_t length_sn,
    u_int total, uint32_t type)
{
	static const uint8_t xdp_uuid[16] = NHI_XDP_UUID;
	u_int dwords = (total - 12) / 4;	/* len excludes route_hi/lo/length_sn */

	le32enc(r + 0, rhi);
	le32enc(r + 4, rlo);
	le32enc(r + 8, (dwords & NHI_XDP_LENGTH_MASK) | (length_sn & NHI_XDP_SN_MASK));
	memcpy(r + 12, xdp_uuid, 16);
	le32enc(r + 28, type);
}

static u_int
xdp_uuid_response(struct nhi_softc *sc, uint8_t *r, uint32_t rhi, uint32_t rlo,
    uint32_t length_sn)
{
	u_int total = XDP_HDR_LEN + 16 + 8;	/* + src_uuid + src_route = 56 */

	xdp_hdr(r, rhi, rlo, length_sn, total, NHI_XDP_UUID_RESPONSE);
	memcpy(r + 32, sc->local_uuid, 16);
	le32enc(r + 48, sc->local_route_hi);
	le32enc(r + 52, sc->local_route_lo);
	return (total);
}

static u_int
xdp_props_response(struct nhi_softc *sc, uint8_t *r, uint32_t rhi, uint32_t rlo,
    uint32_t length_sn, const uint8_t *peer_uuid, u_int off)
{
	u_int chunk, total, i;

	chunk = (off < sc->prop_dwords) ?
	    MIN(sc->prop_dwords - off, NHI_XDP_PROP_CHUNK_DWORDS) : 0;
	total = XDP_HDR_LEN + 16 + 16 + 8 + chunk * 4;	/* +src+dst+off/len+gen */

	xdp_hdr(r, rhi, rlo, length_sn, total, NHI_XDP_PROPERTIES_RESPONSE);
	memcpy(r + 32, sc->local_uuid, 16);		/* src = us */
	memcpy(r + 48, peer_uuid, 16);			/* dst = requester */
	/* {u16 offset; u16 data_length}: offset (echo) in the low half of the
	 * dword, total-block length (dwords) in the high half. */
	le32enc(r + 64, (off & 0xffff) | ((sc->prop_dwords & 0xffff) << 16));
	le32enc(r + 68, sc->prop_generation);
	for (i = 0; i < chunk; i++)
		le32enc(r + 72 + i * 4, sc->prop_block[off + i]);
	return (total);
}

void
nhi_xdomain_handle(struct nhi_softc *sc, const uint8_t *f, u_int len)
{
	static const uint8_t tbip_uuid[16] = NHI_TBIP_SVC_UUID;
	uint8_t r[NHI_CTL_FRAME_SIZE];
	uint32_t req_rhi, req_rlo, rhi, rlo, length_sn, type;
	u_int rlen, off;

	if (len < XDP_HDR_LEN)
		return;

	/* ThunderboltIP login frames carry the service UUID, not the discovery
	 * UUID, and use a different (68-byte) header layout - hand them off. */
	if (memcmp(f + 12, tbip_uuid, 16) == 0) {
		nhi_tbip_handle(sc, f, len);
		return;
	}
	req_rhi = le32dec(f + 0);
	req_rlo = le32dec(f + 4);
	length_sn = le32dec(f + 8);
	/* protocol uuid at f+12 (expect tb_xdp_uuid) */
	type = le32dec(f + 28);

	/*
	 * Respond to the route the request CAME FROM, with the top bit cleared
	 * (Linux tb_xdp_handle_request: route = (route_hi<<32|route_lo) &
	 * ~BIT_ULL(63)).  Echoing the route verbatim (bit63 set) looped the
	 * reply back to our own host router; using the stored peer_route broke
	 * multi-hop (dock) topologies where the peer's request route differs.
	 */
	rhi = req_rhi & 0x7fffffff;
	rlo = req_rlo;
	device_printf(sc->dev,
	    "xdomain rx type=%u len=%u reqroute=%x:%x peerroute=%x:%x\n",
	    type, len, req_rhi, req_rlo, rhi, rlo);

	switch (type) {
	case NHI_XDP_UUID_REQUEST:
		rlen = xdp_uuid_response(sc, r, rhi, rlo, length_sn);
		nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, rlen);
		device_printf(sc->dev, "xdomain: UUID_REQUEST -> UUID_RESPONSE\n");
		break;
	case NHI_XDP_PROPERTIES_REQUEST:
		/* req: hdr(32) + src_uuid(16)@32 + dst_uuid(16)@48 + offset@64 */
		if (len < 68)
			break;
		off = le32dec(f + 64) & 0xffff;	/* u16 offset (low half of the dword) */
		rlen = xdp_props_response(sc, r, rhi, rlo, length_sn, f + 32, off);
		nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, rlen);
		device_printf(sc->dev,
		    "xdomain: PROPERTIES_REQUEST off=%u -> %u bytes\n", off, rlen);
		break;
	case NHI_XDP_PROPERTIES_CHANGED_REQUEST:
		/* peer says its properties changed; just ack (header only). */
		xdp_hdr(r, rhi, rlo, length_sn, XDP_HDR_LEN,
		    NHI_XDP_PROPERTIES_CHANGED_RESPONSE);
		nhi_ctl_tx(sc, NHI_PDF_XDOMAIN_RESP, r, XDP_HDR_LEN);
		device_printf(sc->dev, "xdomain: PROPERTIES_CHANGED -> ack\n");
		break;
	default:
		device_printf(sc->dev, "xdomain: unhandled type=%u len=%u\n",
		    type, len);
		break;
	}
}

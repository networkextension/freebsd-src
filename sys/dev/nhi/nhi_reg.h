/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_reg.h - Native Host Interface (NHI) register and ID definitions.
 *
 * Per DESIGN.md: hardware facts here are cross-checked against Linux
 * drivers/thunderbolt/nhi_regs.h (register offsets are hardware facts, not
 * creative expression) and the device IDs observed on the wire (SolidRun
 * Honeycomb + GIGABYTE GC-Titan Ridge JHL7540).  Items still tagged VERIFY
 * are not yet confirmed by Linux source AND silicon; confirm via probe before
 * trusting.  If a probe disagrees, fix this header (and DESIGN.md) first.
 *
 * Register offsets and bit positions in this revision were reconciled against
 * the upstream Linux nhi_regs.h; the from-memory guesses they replaced are
 * noted in the commit message.
 */
#ifndef _NHI_REG_H_
#define _NHI_REG_H_

/*
 * PCI match table.  The NHI is the only function the host OS DMA-talks to.
 *
 * Observed device IDs (DESIGN.md open question #1, resolved):
 *   Titan Ridge JHL7540 4C 2018:
 *     bridge 0x15ea, NHI 0x15eb, xHCI 0x15ec   (lspci on Honeycomb, Discord)
 *     bridge 0x15ea, NHI 0x15f7, xHCI 0x15f8   (Manouchehri gist)
 *   Both NHI IDs (0x15eb / 0x15f7) label the same JHL7540 4C; match BOTH.
 *   The GIGABYTE GC-Titan Ridge is JHL7540.
 */
#define	NHI_VENDOR_INTEL		0x8086

#define	NHI_DEV_TR_4C_NHI		0x15eb	/* JHL7540 4C NHI (Honeycomb) */
#define	NHI_DEV_TR_4C_NHI_ALT		0x15f7	/* JHL7540 4C NHI (alt label) */
#define	NHI_DEV_AR_2C_NHI		0x1575	/* VERIFY (Alpine Ridge 2C) */
#define	NHI_DEV_AR_4C_NHI		0x1577	/* VERIFY (Alpine Ridge 4C) */
#define	NHI_DEV_AR_LP_NHI		0x15bf	/* VERIFY (Alpine Ridge LP) */
#define	NHI_DEV_MR_NHI			0x1137	/* VERIFY (Maple Ridge TB4) */
/*
 * Meteor Lake-P integrated USB4/TB4 host router NHIs.  Two functions, one per
 * TB4 port: NHI0 at PCI 0:13:2, NHI1 at 0:13:3.  CONFIRMED on the ThinkPad
 * T14s Gen 5 (swift@192.168.11.101): subvendor 0x17aa/0x2329, BAR0 256 KiB.
 * NOTE: integrated USB4 NHI advertises PCI class 0x0c0340 (USB4 Host
 * Interface), NOT the 0x088000 of discrete TB3 NHIs (DESIGN.md §1.2).
 * Connection-manager mode (ICM vs SW-CM) is still unverified here - read
 * FW_STS/OUTMAIL once the driver attaches (TESTPLAN TP-1b); integrated Intel
 * TB is expected SW-CM only.
 */
#define	NHI_DEV_MTL_P_NHI0		0x7ec2	/* Meteor Lake-P NHI0 (CONFIRMED) */
#define	NHI_DEV_MTL_P_NHI1		0x7ec3	/* Meteor Lake-P NHI1 (CONFIRMED) */

/*
 * Ring descriptor.  16 bytes; an array of these forms a TX or RX ring in
 * coherent DMA memory.  Layout from Linux struct ring_desc.
 */
struct nhi_ring_desc {
	uint64_t	phys;		/* DMA address of the frame buffer */
	uint32_t	attrs;		/* length[11:0], eof[15:12], sof[19:16],
					 * flags[31:20] */
	uint32_t	time;		/* timestamp (RX) / reserved (TX) */
} __packed;

#define	NHI_DESC_LEN_MASK		0x00000fff
#define	NHI_DESC_EOF_SHIFT		12
#define	NHI_DESC_SOF_SHIFT		16
#define	NHI_DESC_FLAGS_SHIFT		20

/*
 * Descriptor flag values (Linux enum ring_desc_flags) - they live in the
 * 12-bit flags field, so OR them in shifted by NHI_DESC_FLAGS_SHIFT.
 */
#define	NHI_RING_DESC_COMPLETED		0x2	/* HW set: frame consumed */
#define	NHI_RING_DESC_POSTED		0x4	/* driver set: descriptor valid */
#define	NHI_RING_DESC_INTERRUPT		0x8	/* driver set: IRQ on completion */

/*
 * BAR0 register map.  Confirmed offsets (Linux nhi_regs.h).
 */

/* Descriptor ring engine: per-ring control block, 16-byte stride. */
#define	NHI_REG_TX_RING_BASE		0x00000
#define	NHI_REG_RX_RING_BASE		0x08000
#define	NHI_REG_RING_STRIDE		0x10	/* desc block: hop * 16 */
/*
 * Per-ring control-block field offsets (Linux nhi.c ring_iowrite*desc).  The
 * block is 16 bytes: phys(0x00), a packed producer/consumer index dword
 * (0x08), and a size/maxframe dword (0x0C).  The earlier head/tail/count
 * guesses at 0x10/0x12/0x14 were wrong - they ran past the 16-byte stride
 * into the next ring's block (corrected against Linux; confirmed on Meteor
 * Lake when ring 0 lands).
 */
#define	NHI_RING_PHYS			0x00	/* 64-bit descriptor ring base */
#define	NHI_RING_INDEX			0x08	/* producer/consumer index + doorbell */
#define	 NHI_RING_CONS_MASK		0x0000ffff	/* [15:0]  consumer */
#define	 NHI_RING_PROD_SHIFT		16		/* [31:16] producer */
#define	 NHI_RING_PROD_MASK		0xffff0000
#define	NHI_RING_SIZE			0x0c	/* descriptor count + RX maxframe */
#define	 NHI_RING_SIZE_MASK		0x00000fff	/* [11:0]  ring size */
#define	 NHI_RING_MAXFRAME_SHIFT	16		/* [31:16] RX max frame */

/*
 * Per-ring options block.  hop * 32 from the base; field 0x00 = ring flags,
 * 0x04 = SOF/EOF accept mask (TX) or E2E hop (RX).  (For ring 0, hop=0, so the
 * stride is moot - VERIFY the 32-byte stride against silicon when hop>0 rings
 * land.)  Flags from Linux enum ring_flags.
 */
#define	NHI_REG_TX_OPTIONS_BASE		0x19800
#define	NHI_REG_RX_OPTIONS_BASE		0x29800
#define	NHI_REG_OPTIONS_STRIDE		0x20	/* VERIFY (hop * 32) */
#define	NHI_RING_OPTS_FLAGS		0x00
#define	NHI_RING_OPTS_MASK		0x04	/* SOF/EOF mask (TX) / E2E hop (RX) */
#define	 NHI_RX_OPTIONS_E2E_HOP_SHIFT	12
#define	 NHI_RX_OPTIONS_E2E_HOP_MASK	0x007ff000	/* bits [22:12] */
#define	 NHI_RING_FLAG_ISOCH_ENABLE	(1u << 27)
#define	 NHI_RING_FLAG_E2E		(1u << 28)
#define	 NHI_RING_FLAG_NO_SNOOP		(1u << 29)
#define	 NHI_RING_FLAG_RAW		(1u << 30)	/* ignore SOF/EOF mask */
#define	 NHI_RING_FLAG_ENABLE		(1u << 31)

/* Interrupt status/mask vectors and notify. */
#define	NHI_REG_RING_NOTIFY_BASE	0x37800
#define	NHI_REG_RING_INT_CLEAR		0x37808
#define	NHI_REG_RING_INT_BASE		0x38200
#define	NHI_REG_RING_INT_MASK_CLEAR_BASE 0x38208
#define	NHI_REG_INT_THROTTLE		0x38c00
#define	 NHI_INT_THROTTLE_INTERVAL_MASK	0x0000ffff
#define	NHI_REG_INT_VEC_ALLOC_BASE	0x38c40
#define	 NHI_INT_VEC_ALLOC_MASK		0x0000000f
#define	 NHI_INT_VEC_ALLOC_BITS		4
#define	 NHI_INT_VEC_ALLOC_REGS		8

/*
 * Capabilities.  Low 10 bits = hop (path) count; ring count derives here.
 * Bits [23:16] = NHI register version.
 */
#define	NHI_REG_CAPS			0x39640
#define	 NHI_CAPS_HOP_COUNT_MASK	0x000003ff
#define	 NHI_CAPS_VERSION_MASK		0x00ff0000	/* bits [23:16] */
#define	 NHI_CAPS_VERSION_SHIFT		16
#define	 NHI_CAPS_VERSION_2		0x40

/* DMA misc - interrupt auto-clear control. */
#define	NHI_REG_DMA_MISC		0x39864
#define	 NHI_DMA_MISC_INT_AUTO_CLEAR	(1u << 2)
#define	 NHI_DMA_MISC_DISABLE_AUTO_CLEAR (1u << 17)

/* Host interface reset. */
#define	NHI_REG_RESET			0x39898
#define	 NHI_RESET_HRR			(1u << 0)	/* host ring reset */

/*
 * Firmware mailbox - the path to the on-chip ICM.  INMAIL = host->FW.
 * Command is the low 8 bits; set OP_REQUEST to issue, FW clears it / sets
 * ERROR on completion.  (mbox_ready, DESIGN.md V1.3/V5.1.)
 */
#define	NHI_REG_INMAIL_DATA		0x39900
#define	NHI_REG_INMAIL_CMD		0x39904
#define	 NHI_INMAIL_CMD_MASK		0x000000ff	/* command field */
#define	 NHI_INMAIL_CMD_ERROR		(1u << 30)
#define	 NHI_INMAIL_CMD_OP_REQUEST	(1u << 31)

/* OUTMAIL = FW->host.  Carries the operation mode (ICM vs SW, decision D1). */
#define	NHI_REG_OUTMAIL_CMD		0x3990c
#define	 NHI_OUTMAIL_CMD_OPMODE_SHIFT	8
#define	 NHI_OUTMAIL_CMD_OPMODE_MASK	0x00000f00	/* bits [11:8] */

/*
 * Firmware operation mode reported in the OUTMAIL opmode field
 * (Linux enum nhi_fw_mode).  CM_MODE means the internal connection manager
 * firmware is running and accepting commands - the ICM path of decision D1.
 */
#define	NHI_OPMODE_SAFE			0x0	/* firmware safe mode */
#define	NHI_OPMODE_AUTH			0x1	/* NVM authentication mode */
#define	NHI_OPMODE_EP			0x2	/* endpoint mode */
#define	NHI_OPMODE_CM			0x3	/* connection-manager (ICM) */

/*
 * NHI mailbox commands (Linux enum nhi_mailbox_cmd).  Issued via
 * nhi_mailbox_cmd(); the low 8 bits of INMAIL_CMD.  VERIFY values against
 * silicon when the live mailbox path runs.
 */
#define	NHI_MBOX_SAVE_DEVS		0x05	/* VERIFY */
#define	NHI_MBOX_DISCONNECT_PCIE_PATHS	0x06	/* VERIFY */
#define	NHI_MBOX_DRV_UNLOADS		0x07	/* VERIFY */
#define	NHI_MBOX_DISCONNECT_PA		0x10	/* VERIFY */
#define	NHI_MBOX_DISCONNECT_PB		0x11	/* VERIFY */
#define	NHI_MBOX_ALLOW_ALL_DEVS		0x23	/* VERIFY */

/*
 * Integrated-controller (Ice Lake onward, incl. Meteor Lake) vendor-specific
 * capability registers, in PCI *config* space (Linux nhi_regs.h VS_CAP_*).  The
 * on-die Thunderbolt controller powers down until the host asserts force-power;
 * until then the connection-manager firmware does not answer ring-0 ICM
 * commands.  nhi_force_power() drives this (Linux nhi.c icl_nhi_force_power).
 */
#define	NHI_VS_CAP_9			0xc8	/* firmware status */
#define	 NHI_VS_CAP_9_FW_READY		(1u << 31)
#define	NHI_VS_CAP_22			0xfc	/* power control */
#define	 NHI_VS_CAP_22_FORCE_POWER	(1u << 1)
#define	 NHI_VS_CAP_22_DMA_DELAY_SHIFT	24
#define	 NHI_VS_CAP_22_DMA_DELAY_MASK	0xff000000	/* bits [31:24] */
#define	 NHI_VS_CAP_22_DMA_DELAY_VAL	0x22		/* Linux icl_nhi_force_power */

/* Firmware status - whether the ICM firmware is up. */
#define	NHI_REG_FW_STS			0x39944
#define	 NHI_FW_STS_ICM_EN		(1u << 0)
#define	 NHI_FW_STS_ICM_EN_INVERT	(1u << 1)
#define	 NHI_FW_STS_ICM_EN_CPU		(1u << 2)
#define	 NHI_FW_STS_CIO_RESET_REQ	(1u << 30)
#define	 NHI_FW_STS_NVM_AUTH_DONE	(1u << 31)

/*
 * Ring interrupt registers (Linux nhi.c).  Each is a bitmap with one bit per
 * ring: TX rings occupy bits [0..hop_count-1], RX rings [hop_count..2*hc-1],
 * RX overflow [2*hc..3*hc-1].  So ring 0's RX bit is at index hop_count.
 *   NOTIFY     - interrupt status (read to see which rings fired)
 *   INT_CLEAR  - write to clear status (unless DMA_MISC auto-clear is on)
 *   INT_BASE   - per-ring interrupt enable
 *   MASK_CLEAR - write 1 to disable a ring's interrupt
 * Offsets already defined above (NHI_REG_RING_NOTIFY_BASE etc.).
 */

/*
 * Ring-0 control-channel framing (Linux drivers/thunderbolt/ctl.c).  A control
 * frame is a ring-0 frame whose descriptor SOF and EOF nibbles both carry the
 * PDF (packet descriptor field) type; the payload is followed by a 4-byte
 * CRC32C footer (inverted, big-endian) and frame size = payload + 4.
 * Values: Linux enum tb_cfg_pkg_type.
 */
#define	NHI_PDF_READ			1
#define	NHI_PDF_WRITE			2
#define	NHI_PDF_ERROR			3
#define	NHI_PDF_NOTIFY_ACK		4
#define	NHI_PDF_EVENT			5
#define	NHI_PDF_XDOMAIN_REQ		6
#define	NHI_PDF_XDOMAIN_RESP		7
#define	NHI_PDF_ICM_EVENT		10
#define	NHI_PDF_ICM_CMD			11	/* host -> ICM request */
#define	NHI_PDF_ICM_RESP		12	/* ICM -> host response */

/*
 * ICM messages (Linux drivers/thunderbolt/tb_msgs.h).  Every ICM packet opens
 * with icm_pkg_header { u8 code; u8 flags; u8 packet_id; u8 total_packets; }.
 * DRIVER_READY (the mbox_ready / TP-1b probe): request is just the header with
 * code=0x3; integrated controllers (ICL/TGL/MTL, our 0x7ec2/0x7ec3) reply with
 * the icm_tr driver-ready response (header + reserved16 + info16 + nvm32 +
 * device_id16 + reserved16 = 16 bytes).
 */
#define	NHI_ICM_DRIVER_READY		0x03	/* icm_pkg_header.code */
#define	 NHI_ICM_FLAGS_ERROR		(1u << 0)	/* header.flags: rejected */
/* icm_tr driver-ready response .info field decode */
#define	 NHI_ICM_TR_SLEVEL_MASK		0x0007	/* security level [2:0] */
#define	 NHI_ICM_TR_PROTO_VER_SHIFT	4
#define	 NHI_ICM_TR_PROTO_VER_MASK	0x0070	/* protocol version [6:4] */

/* ICM notification (PDF ICM_EVENT) codes we care about (Linux icm.c). */
#define	NHI_ICM_EVENT_DEVICE_CONNECTED	0x03
#define	NHI_ICM_EVENT_XDOMAIN_CONNECTED	0x06
#define	NHI_ICM_EVENT_XDOMAIN_DISCONNECTED 0x07

/*
 * ICM commands for XDomain data paths (Linux icm.c, TR variant for integrated
 * controllers).  APPROVE sets up the bidirectional DMA tunnel for the network
 * service; the OS picks the NHI ring indices (>0) and fabric HopIDs and passes
 * them in.  icm_tr_pkg_approve_xdomain:
 *   hdr(4) + route_hi(4) + route_lo(4) + remote_uuid(16) +
 *   transmit_path(u16) + transmit_ring(u16) + receive_path(u16) + receive_ring(u16)
 * = 36 bytes (struct icm_tr_pkg_approve_xdomain, Linux tb_msgs.h); response echoes it.
 */
#define	NHI_ICM_APPROVE_XDOMAIN		0x10
#define	NHI_ICM_DISCONNECT_XDOMAIN	0x11

/*
 * ThunderboltIP (Apple) networking protocol (Linux drivers/net/thunderbolt).
 * Login/logout/status ride ring 0 as XDomain frames (PDF XDOMAIN_REQ/RESP) but
 * carry the ThunderboltIP *service* UUID (NHI_TBIP_SVC_UUID, below) in the xdp
 * header instead of the discovery UUID.  thunderbolt_ip_header:
 *   route_hi(4) route_lo(4) length_sn(4) uuid(16) initiator_uuid(16)
 *   target_uuid(16) type(4) command_id(4)  = 68 bytes.
 */
#define	NHI_TBIP_HDR_LEN		68
#define	 NHI_TBIP_OFF_UUID		12	/* protocol (service) uuid */
#define	 NHI_TBIP_OFF_INITIATOR		28
#define	 NHI_TBIP_OFF_TARGET		44
#define	 NHI_TBIP_OFF_TYPE		60
#define	 NHI_TBIP_OFF_COMMAND_ID	64
#define	NHI_TBIP_LOGIN			0
#define	NHI_TBIP_LOGIN_RESPONSE		1
#define	NHI_TBIP_LOGOUT			2
#define	NHI_TBIP_STATUS			3
#define	NHI_TBIP_PROTO_VERSION		1

/* network service property values (Linux tbnet). */
#define	NHI_TBNET_PRTCID		1
#define	NHI_TBNET_PRTCVERS		1
#define	NHI_TBNET_PRTCREVS		1
#define	 NHI_TBNET_E2E			0x1	/* prtcstns bit0 */
#define	 NHI_TBNET_MATCH_FRAGS_ID	0x2	/* bit1 */
#define	 NHI_TBNET_64K_FRAMES		0x4	/* bit2 */
#define	NHI_TBNET_PRTCSTNS \
	(NHI_TBNET_E2E | NHI_TBNET_MATCH_FRAGS_ID | NHI_TBNET_64K_FRAMES)

/* Data rings for the network service (ring 0 is control). */
#define	NHI_DATA_TX_RING		1
#define	NHI_DATA_RX_RING		2
#define	NHI_DATA_RING_COUNT		256
#define	NHI_DATA_FRAME_SIZE		4096

/*
 * ThunderboltIP data-frame header (Linux thunderbolt_ip_frame_header).  Rides
 * the data ring as raw LITTLE-endian bytes - the data path does NOT byteswap or
 * append a CRC32C footer like the ring-0 control path.  Each DMA frame is this
 * 12-byte header followed by up to NHI_TBIP_MAX_PAYLOAD payload bytes; a network
 * packet larger than that is split into frame_count frames matched by frame_id.
 */
#define	NHI_TBIP_FRAME_HDR_LEN		12
#define	 NHI_TBIP_FRAME_OFF_SIZE	0	/* __le32 frame_size (this frame's payload) */
#define	 NHI_TBIP_FRAME_OFF_INDEX	4	/* __le16 frame_index (0-based) */
#define	 NHI_TBIP_FRAME_OFF_ID		6	/* __le16 frame_id (per network packet) */
#define	 NHI_TBIP_FRAME_OFF_COUNT	8	/* __le32 frame_count (frames in packet) */
#define	NHI_TBIP_MAX_PAYLOAD	(NHI_DATA_FRAME_SIZE - NHI_TBIP_FRAME_HDR_LEN)
/* Per-DMA-frame SOF/EOF markers on the data ring descriptor (Linux tbnet). */
#define	NHI_TBIP_PDF_FRAME_START	1
#define	NHI_TBIP_PDF_FRAME_END		2

/*
 * XDomain discovery protocol (Linux drivers/thunderbolt/xdomain.c + tb_msgs.h).
 * Frames ride ring 0 with PDF XDOMAIN_REQ=6 / RESP=7 (above).  UUIDs here are
 * in standard (display) byte order; the ring-0 transport's per-dword byteswap
 * maps them to/from the wire (matches Linux uuid_t + cpu_to_be32_array).
 *
 *   tb_xdomain_header { u32 route_hi; u32 route_lo; u32 length_sn; }
 *   tb_xdp_header     { tb_xdomain_header xd; uint8_t uuid[16]; u32 type; }
 */
#define	NHI_XDP_LENGTH_MASK	0x0000003f	/* length_sn bits [5:0], dwords */
#define	NHI_XDP_SN_SHIFT	27
#define	NHI_XDP_SN_MASK		0x18000000	/* length_sn bits [28:27] */

/* The fixed XDomain discovery protocol UUID (b638d70e-42ff-40bb-97c2-90e2c0b2ff07). */
#define	NHI_XDP_UUID	{ 0xb6,0x38,0xd7,0x0e, 0x42,0xff, 0x40,0xbb, \
			  0x97,0xc2, 0x90,0xe2,0xc0,0xb2,0xff,0x07 }

/* enum tb_xdp_type */
#define	NHI_XDP_UUID_REQUEST		12
#define	NHI_XDP_UUID_RESPONSE		2
#define	NHI_XDP_PROPERTIES_REQUEST	3
#define	NHI_XDP_PROPERTIES_RESPONSE	4
#define	NHI_XDP_PROPERTIES_CHANGED_REQUEST  5
#define	NHI_XDP_PROPERTIES_CHANGED_RESPONSE 6
#define	NHI_XDP_ERROR_RESPONSE		7

/* enum tb_xdp_error */
#define	NHI_XDP_ERROR_SUCCESS		0
#define	NHI_XDP_ERROR_UNKNOWN_PACKET	1
#define	NHI_XDP_ERROR_UNKNOWN_DOMAIN	2
#define	NHI_XDP_ERROR_NOT_SUPPORTED	3
#define	NHI_XDP_ERROR_NOT_READY		4

/* Max property-block dwords per PROPERTIES_RESPONSE chunk (fits a 256B frame). */
#define	NHI_XDP_PROP_CHUNK_DWORDS	40

/*
 * Property directory block (Linux drivers/thunderbolt/property.c).  Built in
 * native dwords (the ring-0 TX path byteswaps to the big-endian wire format).
 * Root header = magic + length(dwords); entries are 5 dwords each; a DIRECTORY
 * entry's body starts with a 16-byte UUID.
 */
#define	NHI_PROP_MAGIC		0x55584401u	/* 'U','X','D',0x01 */
#define	NHI_PROP_TYPE_VALUE	0x76		/* 'v' immediate u32 */
#define	NHI_PROP_TYPE_TEXT	0x74		/* 't' NUL-term string */
#define	NHI_PROP_TYPE_DATA	0x64		/* 'd' binary blob */
#define	NHI_PROP_TYPE_DIRECTORY	0x44		/* 'D' nested dir (uuid body) */

/* network service directory UUID (c66189ca-1cce-4195-bdb8-49592e5f5a4f). */
#define	NHI_NET_DIR_UUID { 0xc6,0x61,0x89,0xca, 0x1c,0xce, 0x41,0x95, \
			   0xbd,0xb8, 0x49,0x59,0x2e,0x5f,0x5a,0x4f }
/* Apple ThunderboltIP service UUID (798f589e-3616-8a47-97c6-5664a920c8dd). */
#define	NHI_TBIP_SVC_UUID { 0x79,0x8f,0x58,0x9e, 0x36,0x16, 0x8a,0x47, \
			    0x97,0xc6, 0x56,0x64,0xa9,0x20,0xc8,0xdd }

#endif /* _NHI_REG_H_ */

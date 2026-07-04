/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2022 Scott Long
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Thunderbolt 3 / Native Host Interface driver variables
 *
 * $FreeBSD$
 */

#ifndef _NHI_VAR
#define _NHI_VAR

MALLOC_DECLARE(M_NHI);

#define NHI_MSIX_MAX		32
#define NHI_RING0_TX_DEPTH	16
#define NHI_RING0_RX_DEPTH	16
#define NHI_DEFAULT_NUM_RINGS	1
#define NHI_MAX_NUM_RINGS	32	/* XXX 2? */
#define NHI_RING0_FRAME_SIZE	256
#define NHI_MAILBOX_TIMEOUT	15

#define NHI_CMD_TIMEOUT		3	/* 3 seconds */

struct nhi_softc;
struct nhi_ring_pair;
struct nhi_intr_tracker;
struct nhi_cmd_frame;
struct hcm_softc;
struct router_softc;

struct nhi_cmd_frame {
	TAILQ_ENTRY(nhi_cmd_frame)	cm_link;
	uint32_t		*data;
	bus_addr_t		data_busaddr;
	u_int			req_len;
	uint16_t		flags;
#define CMD_MAPPED		(1 << 0)
#define CMD_POLLED		(1 << 1)
#define CMD_REQ_COMPLETE	(1 << 2)
#define CMD_RESP_COMPLETE	(1 << 3)
#define CMD_RESP_OVERRUN	(1 << 4)
	uint16_t		retries;
	uint16_t		pdf;		/* descriptor EOF (and SOF if sof==0) */
	uint16_t		sof;		/* descriptor SOF; 0 => use pdf.
					 * FRAME-mode data frames need SOF != EOF
					 * (ThunderboltIP: SOF=FRAME_START,
					 * EOF=FRAME_END). */
	uint16_t		idx;

	void			*context;
	u_int			timeout;

	uint32_t		*resp_buffer;
	u_int			resp_len;
};

#define NHI_RING_NAMELEN	16
struct nhi_ring_pair {
	struct nhi_softc	*sc;

	union nhi_ring_desc	*tx_ring;
	union nhi_ring_desc	*rx_ring;

	uint16_t		tx_pi;
	uint16_t		tx_ci;
	uint16_t		rx_pi;
	uint16_t		rx_ci;

	uint16_t		rx_pici_reg;
	uint16_t		tx_pici_reg;

	struct nhi_cmd_frame	**rx_cmd_ring;
	struct nhi_cmd_frame	**tx_cmd_ring;

	struct mtx		mtx;
	char			name[NHI_RING_NAMELEN];
	struct nhi_intr_tracker	*tracker;
	SLIST_ENTRY(nhi_ring_pair)	ring_link;

	TAILQ_HEAD(, nhi_cmd_frame)	tx_head;
	TAILQ_HEAD(, nhi_cmd_frame)	rx_head;

	uint16_t		tx_ring_depth;
	uint16_t		tx_ring_mask;
	uint16_t		rx_ring_depth;
	uint16_t		rx_ring_mask;
	uint16_t		rx_buffer_size;
	u_char			ring_num;

	bus_dma_tag_t		ring_dmat;
	bus_dmamap_t		ring_map;
	void			*ring;
	bus_addr_t		tx_ring_busaddr;
	bus_addr_t		rx_ring_busaddr;

	bus_dma_tag_t		frames_dmat;
	bus_dmamap_t		frames_map;
	void			*frames;
	bus_addr_t		tx_frames_busaddr;
	bus_addr_t		rx_frames_busaddr;

	/*
	 * Data-ring (non-ring-0) FRAME-mode + E2E options.  Zero for ring 0,
	 * which runs RAW.  Ported from the nhi_icm data path (nhi_ring.c
	 * nhi_data_setup); a ThunderboltIP net driver needs a FRAME-mode ring
	 * pair with end-to-end flow control.
	 */
	struct nhi_cmd_frame	*cmds;		/* per-ring frame trackers */
	u_char			frame_mode;	/* 1 = FRAME, 0 = RAW */
	u_char			e2e;		/* end-to-end flow control */
	u_char			e2e_hopid;	/* paired TX hop for RX E2E */
	uint16_t		sof_mask;	/* RX start-of-frame PDF bitmap */
	uint16_t		eof_mask;	/* RX end-of-frame PDF bitmap */

	/*
	 * RX poll fallback.  MSI-X allocation is flaky on some hosts (Meteor
	 * Lake advertises 16 vectors but pci_alloc_msix fails, so the driver
	 * runs on 2 MSI vectors).  A ring whose number is >= msix_count gets no
	 * dedicated vector - and no shared tracker either, since the tracker
	 * array is sized msix_count.  Such a ring is given a private tracker
	 * (for rxpdf storage) and is drained by a callout instead of an
	 * interrupt, matching the standalone driver's and macOS's poll path.
	 */
	struct callout		rxpoll_co;
	bool			rxpoll_active;
	bool			priv_tracker;	/* r->tracker was malloc'd here */
	uint32_t		dbg_polls;	/* diag: poll counter (rate-limit) */
};

/*
 * Options for creating a data ring pair (nhi_ring_create).  A ThunderboltIP
 * data ring is FRAME mode (RAW off) with end-to-end flow control; the RX ring
 * filters by SOF/EOF PDF and carries the paired TX hop for E2E credit.
 */
struct nhi_ring_opts {
	uint16_t		frame_size;	/* per-frame buffer (e.g. 4096) */
	uint16_t		sof_mask;	/* RX start-of-frame PDF bitmap */
	uint16_t		eof_mask;	/* RX end-of-frame PDF bitmap */
	u_char			frame_mode;	/* 1 = FRAME mode (RAW off) */
	u_char			e2e;		/* enable end-to-end flow control */
	u_char			e2e_hopid;	/* paired hop id for RX E2E */
};

/* PDF-indexed array of dispatch routines for interrupts */
typedef void (nhi_ring_cb_t)(void *, union nhi_ring_desc *,
    struct nhi_cmd_frame *);
struct nhi_pdf_dispatch {
	nhi_ring_cb_t		*cb;
	void			*context;
};

struct nhi_intr_tracker {
	struct nhi_softc	*sc;
	struct nhi_ring_pair	*ring;
	struct nhi_pdf_dispatch	txpdf[16];
	struct nhi_pdf_dispatch	rxpdf[16];
	u_int			vector;
};

struct nhi_softc {
	device_t		dev;
	device_t		ufp;
	u_int			debug;
	u_int			hwflags;
#define NHI_TYPE_UNKNOWN	0x00
#define NHI_TYPE_USB4		0x0f
#define NHI_TYPE_MASK		0x0f
#define NHI_MBOX_BUSY		0x10
	struct hcm_softc	*hcm;
	struct router_softc	*root_rsc;

	struct nhi_ring_pair	*ring0;
	struct nhi_intr_tracker	*intr_trackers;

	uint16_t		path_count;
	uint16_t		max_ring_count;

	struct mtx		nhi_mtx;
	SLIST_HEAD(, nhi_ring_pair)	ring_list;

	int			msix_count;
	struct resource		*irqs[NHI_MSIX_MAX];
	void			*intrhand[NHI_MSIX_MAX];
	int			irq_rid[NHI_MSIX_MAX];
	struct resource		*irq_pba;
	int			irq_pba_rid;
	struct resource		*irq_table;
	int			irq_table_rid;

	struct resource		*regs_resource;
	bus_space_handle_t	regs_bhandle;
	bus_space_tag_t		regs_btag;
	int			regs_rid;

	bus_dma_tag_t		parent_dmat;

	bus_dma_tag_t		ring0_dmat;
	bus_dmamap_t		ring0_map;
	void			*ring0_frames;
	bus_addr_t		ring0_frames_busaddr;
	struct nhi_cmd_frame	*ring0_cmds;

	struct sysctl_ctx_list	*sysctl_ctx;
	struct sysctl_oid	*sysctl_tree;

	struct intr_config_hook	ich;

	struct tb_icm		*icm;	/* ICM connection manager (ICM mode) */

	uint8_t			uuid[16];
	uint8_t			lc_uuid[16];
};

struct nhi_dispatch {
	uint8_t			pdf;
	nhi_ring_cb_t		*cb;
	void			*context;
};

#define NHI_IS_USB4(sc)	(((sc)->hwflags & NHI_TYPE_MASK) == NHI_TYPE_USB4)

int nhi_pci_configure_interrupts(struct nhi_softc *sc);
void nhi_pci_enable_interrupt(struct nhi_ring_pair *r);
void nhi_pci_disable_interrupts(struct nhi_softc *sc);
void nhi_pci_free_interrupts(struct nhi_softc *sc);
int nhi_pci_get_uuid(struct nhi_softc *sc);
int nhi_read_lc_mailbox(struct nhi_softc *, u_int reg, uint32_t *val);
int nhi_write_lc_mailbox(struct nhi_softc *, u_int reg, uint32_t val);

void nhi_get_tunables(struct nhi_softc *);
int nhi_attach(struct nhi_softc *);
int nhi_detach(struct nhi_softc *);

struct nhi_cmd_frame * nhi_alloc_tx_frame(struct nhi_ring_pair *);
void nhi_free_tx_frame(struct nhi_ring_pair *, struct nhi_cmd_frame *);

/*
 * Data-ring (non-ring-0) lifecycle for a network/tunnel driver.  create =
 * allocate ring + FRAME/E2E frame pool + configure; start = fill RX + activate
 * (routes ring N to its own MSI-X vector via nhi_pci_enable_interrupt); stop =
 * deactivate; destroy = free.
 */
int nhi_ring_create(struct nhi_softc *, u_int ringnum, u_int tx_depth,
    u_int rx_depth, const struct nhi_ring_opts *, struct nhi_ring_pair **);
int nhi_ring_start(struct nhi_ring_pair *);
int nhi_ring_stop(struct nhi_ring_pair *);
void nhi_ring_destroy(struct nhi_ring_pair *);

int nhi_inmail_cmd(struct nhi_softc *, uint32_t, uint32_t);
int nhi_outmail_cmd(struct nhi_softc *, uint32_t *);

int nhi_tx_schedule(struct nhi_ring_pair *, struct nhi_cmd_frame *);
int nhi_tx_synchronous(struct nhi_ring_pair *, struct nhi_cmd_frame *);
void nhi_intr(void *);

int nhi_register_pdf(struct nhi_ring_pair *, struct nhi_dispatch *,
    struct nhi_dispatch *);
int nhi_deregister_pdf(struct nhi_ring_pair *, struct nhi_dispatch *,
    struct nhi_dispatch *);

/* Low level read/write MMIO registers */
static __inline uint32_t
nhi_read_reg(struct nhi_softc *sc, u_int offset)
{
	return (le32toh(bus_space_read_4(sc->regs_btag, sc->regs_bhandle,
	    offset)));
}

static __inline void
nhi_write_reg(struct nhi_softc *sc, u_int offset, uint32_t val)
{
	bus_space_write_4(sc->regs_btag, sc->regs_bhandle, offset,
	    htole32(val));
}

static __inline struct nhi_cmd_frame *
nhi_alloc_tx_frame_locked(struct nhi_ring_pair *r)
{
	struct nhi_cmd_frame *cmd;

	if ((cmd = TAILQ_FIRST(&r->tx_head)) != NULL)
		TAILQ_REMOVE(&r->tx_head, cmd, cm_link);
	return (cmd);
}

static __inline void
nhi_free_tx_frame_locked(struct nhi_ring_pair *r, struct nhi_cmd_frame *cmd)
{
	/* Clear all flags except for MAPPED */
	cmd->flags &= CMD_MAPPED;
	cmd->resp_buffer = NULL;
	TAILQ_INSERT_TAIL(&r->tx_head, cmd, cm_link);
}

#endif /* _NHI_VAR */

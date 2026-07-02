/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_ring.c - NHI descriptor-ring engine + ring-0 control transport.
 *
 * Allocates a TX or RX descriptor ring in DMA-coherent memory plus a block of
 * streaming frame buffers, programs the ring registers, and drives the
 * producer/consumer doorbell.  ring 0 is the control channel used to carry
 * Thunderbolt control frames (the ICM DRIVER_READY handshake lands on top of
 * nhi_ctl_tx/nhi_ctl_rx here); data rings for if_tbt(4) reuse the allocator.
 *
 * Register layout and the ring bring-up sequence are cross-checked against
 * Linux drivers/thunderbolt/nhi.c + nhi_regs.h (ring_iowrite*desc/options,
 * tb_ring_start) and ctl.c (control-frame framing + CRC32C footer).
 *
 * arm64 coherency (DESIGN.md #1 risk, dma_selftest V4.3): descriptors are
 * BUS_DMA_COHERENT so the producer/consumer indices and completion flags are
 * read without explicit sync; frame buffers are streaming and every post/reap
 * is bracketed with bus_dmamap_sync().  x86 (this first test box) is coherent,
 * so the sync calls are no-ops here - which makes it the clean baseline to
 * compare the non-coherent arm64 targets against.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/gsb_crc32.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

/* Ring base address alignment.  VERIFY exact requirement against silicon. */
#define	NHI_RING_ALIGN	16

static void
nhi_ring_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error == 0 && nseg == 1)
		*(bus_addr_t *)arg = segs[0].ds_addr;
}

/*
 * Allocate a ring: the coherent descriptor array plus a contiguous block of
 * count*frame_size streaming frame buffers.
 */
int
nhi_ring_alloc(struct nhi_softc *sc, struct nhi_ring *r, int hop, bool is_tx,
    u_int count, u_int frame_size)
{
	bus_addr_t busaddr;
	bus_size_t sz;
	int error;

	bzero(r, sizeof(*r));
	r->sc = sc;
	r->hop = hop;
	r->is_tx = is_tx;
	r->count = count;
	r->frame_size = frame_size;

	/* --- descriptor ring (coherent) --- */
	sz = (bus_size_t)count * sizeof(struct nhi_ring_desc);
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
	    NHI_RING_ALIGN, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL, sz, 1, sz, 0, NULL, NULL, &r->desc_tag);
	if (error != 0) {
		device_printf(sc->dev, "ring %d: desc tag create failed: %d\n",
		    hop, error);
		return (error);
	}
	error = bus_dmamem_alloc(r->desc_tag, (void **)&r->desc,
	    BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO, &r->desc_map);
	if (error != 0) {
		device_printf(sc->dev, "ring %d: desc alloc failed: %d\n",
		    hop, error);
		goto fail_desc_tag;
	}
	busaddr = 0;
	error = bus_dmamap_load(r->desc_tag, r->desc_map, r->desc, sz,
	    nhi_ring_dmamap_cb, &busaddr, BUS_DMA_NOWAIT);
	if (error != 0 || busaddr == 0) {
		device_printf(sc->dev, "ring %d: desc load failed: %d\n",
		    hop, error);
		if (error == 0)
			error = ENOMEM;
		goto fail_desc_mem;
	}
	r->desc_busaddr = busaddr;

	/* --- frame buffers (streaming) --- */
	sz = (bus_size_t)count * frame_size;
	error = bus_dma_tag_create(bus_get_dma_tag(sc->dev),
	    frame_size, 0, BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
	    NULL, NULL, sz, 1, sz, 0, NULL, NULL, &r->buf_tag);
	if (error != 0) {
		device_printf(sc->dev, "ring %d: buf tag create failed: %d\n",
		    hop, error);
		goto fail_desc_load;
	}
	error = bus_dmamem_alloc(r->buf_tag, &r->buf,
	    BUS_DMA_WAITOK | BUS_DMA_ZERO, &r->buf_map);
	if (error != 0) {
		device_printf(sc->dev, "ring %d: buf alloc failed: %d\n",
		    hop, error);
		goto fail_buf_tag;
	}
	busaddr = 0;
	error = bus_dmamap_load(r->buf_tag, r->buf_map, r->buf, sz,
	    nhi_ring_dmamap_cb, &busaddr, BUS_DMA_NOWAIT);
	if (error != 0 || busaddr == 0) {
		device_printf(sc->dev, "ring %d: buf load failed: %d\n",
		    hop, error);
		if (error == 0)
			error = ENOMEM;
		goto fail_buf_mem;
	}
	r->buf_busaddr = busaddr;

	if (bootverbose)
		device_printf(sc->dev,
		    "ring %d (%s): %u descs, desc=%#jx buf=%#jx\n",
		    hop, is_tx ? "tx" : "rx", count,
		    (uintmax_t)r->desc_busaddr, (uintmax_t)r->buf_busaddr);
	return (0);

fail_buf_mem:
	bus_dmamem_free(r->buf_tag, r->buf, r->buf_map);
	r->buf = NULL;
fail_buf_tag:
	bus_dma_tag_destroy(r->buf_tag);
	r->buf_tag = NULL;
fail_desc_load:
	bus_dmamap_unload(r->desc_tag, r->desc_map);
	r->desc_busaddr = 0;
fail_desc_mem:
	bus_dmamem_free(r->desc_tag, r->desc, r->desc_map);
	r->desc = NULL;
fail_desc_tag:
	bus_dma_tag_destroy(r->desc_tag);
	r->desc_tag = NULL;
	return (error);
}

void
nhi_ring_free(struct nhi_ring *r)
{
	if (r->buf_busaddr != 0)
		bus_dmamap_unload(r->buf_tag, r->buf_map);
	if (r->buf != NULL)
		bus_dmamem_free(r->buf_tag, r->buf, r->buf_map);
	if (r->buf_tag != NULL)
		bus_dma_tag_destroy(r->buf_tag);
	if (r->desc_busaddr != 0)
		bus_dmamap_unload(r->desc_tag, r->desc_map);
	if (r->desc != NULL)
		bus_dmamem_free(r->desc_tag, r->desc, r->desc_map);
	if (r->desc_tag != NULL)
		bus_dma_tag_destroy(r->desc_tag);
	bzero(r, sizeof(*r));
}

static bus_size_t
nhi_ring_desc_base(struct nhi_ring *r)
{
	return ((r->is_tx ? NHI_REG_TX_RING_BASE : NHI_REG_RX_RING_BASE) +
	    (bus_size_t)r->hop * NHI_REG_RING_STRIDE);
}

static bus_size_t
nhi_ring_opts_base(struct nhi_ring *r)
{
	return ((r->is_tx ? NHI_REG_TX_OPTIONS_BASE : NHI_REG_RX_OPTIONS_BASE) +
	    (bus_size_t)r->hop * NHI_REG_OPTIONS_STRIDE);
}

/*
 * Program and enable the ring (Linux tb_ring_start): write the descriptor base
 * and size, clear the SOF/EOF mask, then set ENABLE|RAW in the options flags.
 * RAW makes the RX ring accept every PDF (we have a single control RX ring, so
 * all control frames land here).
 */
void
nhi_ring_program(struct nhi_ring *r)
{
	struct nhi_softc *sc = r->sc;
	bus_size_t base = nhi_ring_desc_base(r);
	bus_size_t opt = nhi_ring_opts_base(r);
	uint32_t size;

	nhi_write(sc, base + NHI_RING_PHYS,
	    (uint32_t)(r->desc_busaddr & 0xffffffff));
	nhi_write(sc, base + NHI_RING_PHYS + 4,
	    (uint32_t)(r->desc_busaddr >> 32));

	/*
	 * Linux ring_start: frame-mode rings (tbnet data path) write max-frame
	 * 0 (= 4096) and run without RAW, RX filtering by the PDF sof/eof
	 * masks; non-frame rings (control ring 0) write the real frame size
	 * and set RAW (ignore the masks).
	 */
	size = r->count & NHI_RING_SIZE_MASK;
	if (!r->is_tx && !r->frame_mode)
		size |= (r->frame_size << NHI_RING_MAXFRAME_SHIFT);
	nhi_write(sc, base + NHI_RING_SIZE, size);

	if (r->frame_mode) {
		uint32_t flags = NHI_RING_FLAG_ENABLE;
		uint32_t mask = 0;

		if (!r->is_tx)
			mask = ((uint32_t)r->sof_mask << 16) | r->eof_mask;
		nhi_write(sc, opt + NHI_RING_OPTS_MASK, mask);
		/* Step 1: set the ring valid/enable bit alone. */
		nhi_write(sc, opt + NHI_RING_OPTS_FLAGS, flags);
		/*
		 * Step 2 (Linux tb_ring_start: "Now that the ring valid bit
		 * is set we can configure E2E"): only AFTER the valid bit is
		 * live may the E2E flow-control bits be set - writing them
		 * together leaves E2E unarmed, no credits ever flow, and the
		 * peer's (fully-E2E) transmitter never puts a frame on the
		 * wire.  tbnet enables E2E on BOTH rings.
		 */
		if (r->e2e) {
			if (!r->is_tx)
				flags |= ((uint32_t)r->e2e_hop <<
				    NHI_RX_OPTIONS_E2E_HOP_SHIFT) &
				    NHI_RX_OPTIONS_E2E_HOP_MASK;
			flags |= NHI_RING_FLAG_E2E;
			nhi_write(sc, opt + NHI_RING_OPTS_FLAGS, flags);
		}
	} else {
		nhi_write(sc, opt + NHI_RING_OPTS_MASK, 0);
		nhi_write(sc, opt + NHI_RING_OPTS_FLAGS,
		    NHI_RING_FLAG_ENABLE | NHI_RING_FLAG_RAW);
	}
}

static void
nhi_ring_disable(struct nhi_ring *r)
{
	nhi_write(r->sc, nhi_ring_opts_base(r) + NHI_RING_OPTS_FLAGS, 0);
}

/* Ring the doorbell: publish r->head as the producer (TX) or consumer (RX). */
void
nhi_ring_doorbell(struct nhi_ring *r)
{
	bus_size_t base = nhi_ring_desc_base(r);

	if (r->is_tx)
		nhi_write(r->sc, base + NHI_RING_INDEX,
		    (r->head << NHI_RING_PROD_SHIFT) & NHI_RING_PROD_MASK);
	else
		nhi_write(r->sc, base + NHI_RING_INDEX,
		    r->head & NHI_RING_CONS_MASK);
}

/* Initialise one RX descriptor to point at its frame-buffer slot. */
static void
nhi_rx_desc_post(struct nhi_ring *r, u_int slot)
{
	struct nhi_ring_desc *d = &r->desc[slot];

	d->phys = r->buf_busaddr + (uint64_t)slot * r->frame_size;
	d->attrs = (r->frame_size & NHI_DESC_LEN_MASK) |
	    ((uint32_t)(NHI_RING_DESC_POSTED | NHI_RING_DESC_INTERRUPT) <<
	     NHI_DESC_FLAGS_SHIFT);
	d->time = 0;
}

/* Index (bit) of a ring's interrupt within the per-ring bitmap. */
static int
nhi_ring_int_bit(struct nhi_ring *r)
{
	return (r->is_tx ? r->hop : r->hop + r->sc->hop_count);
}

static void
nhi_ring_int_enable(struct nhi_ring *r, bool enable)
{
	struct nhi_softc *sc = r->sc;
	int bit = nhi_ring_int_bit(r);
	bus_size_t reg = NHI_REG_RING_INT_BASE + (bit / 32) * 4;
	uint32_t val;

	val = nhi_read(sc, reg);
	if (enable)
		val |= (1u << (bit % 32));
	else
		val &= ~(1u << (bit % 32));
	nhi_write(sc, reg, val);
}

/*
 * Bring up the control channel: ring-0 TX + RX.  Posts all RX buffers, enables
 * the RX interrupt, and turns on the Intel interrupt auto-clear quirk so ring
 * status clears itself (matches Linux QUIRK_AUTO_CLEAR_INT).
 */
int
nhi_ring0_setup(struct nhi_softc *sc)
{
	struct nhi_ring *tx = &sc->ring0_tx;
	struct nhi_ring *rx = &sc->ring0_rx;
	uint32_t misc;
	u_int i;
	int error;

	error = nhi_ring_alloc(sc, tx, NHI_CTL_RING, true,
	    NHI_CTL_RING_COUNT, NHI_CTL_FRAME_SIZE);
	if (error != 0)
		return (error);
	error = nhi_ring_alloc(sc, rx, NHI_CTL_RING, false,
	    NHI_CTL_RING_COUNT, NHI_CTL_FRAME_SIZE);
	if (error != 0) {
		nhi_ring_free(tx);
		return (error);
	}

	/*
	 * Disable HW auto-clear; we clear NOTIFY status explicitly via
	 * INT_CLEAR in nhi_ring0_intr.  VERIFY (intrdump): on MTL the HW
	 * auto-clear bit did NOT reset the status/edge latch - status bits
	 * accumulated (0x01001001 after 270 frames) and the edge-triggered
	 * MSI stopped firing after the very first 0->1 transition (270 CFG
	 * round-trips produced 1 interrupt).  Linux only sets the HW
	 * auto-clear bit in the MSI-X path; the legacy/single-MSI path clears
	 * explicitly (nhi_clear_interrupt: iowrite32(~0, REG_RING_INT_CLEAR)).
	 */
	misc = nhi_read(sc, NHI_REG_DMA_MISC);
	misc &= ~NHI_DMA_MISC_INT_AUTO_CLEAR;
	misc |= NHI_DMA_MISC_DISABLE_AUTO_CLEAR;
	nhi_write(sc, NHI_REG_DMA_MISC, misc);

	nhi_ring_program(tx);
	nhi_ring_program(rx);

	/* Post count-1 RX buffers (leave one slot so head != tail == empty). */
	bus_dmamap_sync(rx->buf_tag, rx->buf_map, BUS_DMASYNC_PREREAD);
	for (i = 0; i < rx->count - 1; i++)
		nhi_rx_desc_post(rx, i);
	rx->head = rx->count - 1;
	rx->tail = 0;
	nhi_ring_doorbell(rx);

	nhi_ring_int_enable(rx, true);

	sc->ring0_up = true;
	if (bootverbose)
		device_printf(sc->dev, "ring 0 up: tx+rx enabled, %u rx bufs\n",
		    rx->count - 1);
	return (0);
}

void
nhi_ring0_teardown(struct nhi_softc *sc)
{
	if (!sc->ring0_up)
		return;
	nhi_ring_int_enable(&sc->ring0_rx, false);
	nhi_ring_disable(&sc->ring0_tx);
	nhi_ring_disable(&sc->ring0_rx);
	nhi_ring_free(&sc->ring0_tx);
	nhi_ring_free(&sc->ring0_rx);
	sc->ring0_up = false;
}

/*
 * Transmit a control frame on ring 0: copy the payload into the TX slot, append
 * the CRC32C footer (inverted, big-endian; Linux tb_crc), set SOF=EOF=pdf, and
 * ring the doorbell.  frame size = payload + 4 (CRC).
 */
int
nhi_ctl_tx(struct nhi_softc *sc, uint8_t pdf, const void *data, u_int len)
{
	struct nhi_ring *tx = &sc->ring0_tx;
	struct nhi_ring_desc *d;
	uint8_t *fbuf;
	u_int slot, total, i;
	uint32_t crc;

	if (!sc->ring0_up)
		return (ENXIO);
	if ((len & 3) != 0)		/* control payloads are dword-aligned */
		return (EINVAL);
	if (len + 4 > tx->frame_size)
		return (EINVAL);

	slot = tx->head;
	fbuf = (uint8_t *)tx->buf + (size_t)slot * tx->frame_size;
	/*
	 * Control frames are big-endian per dword on the wire (Linux ctl.c
	 * cpu_to_be32_array); the CRC32C is then computed over those bytes.
	 */
	for (i = 0; i < len; i += 4)
		be32enc(fbuf + i, le32dec((const uint8_t *)data + i));
	crc = ~calculate_crc32c(~0U, fbuf, len);
	be32enc(fbuf + len, crc);
	total = len + 4;
	bus_dmamap_sync(tx->buf_tag, tx->buf_map, BUS_DMASYNC_PREWRITE);

	d = &tx->desc[slot];
	d->phys = tx->buf_busaddr + (uint64_t)slot * tx->frame_size;
	d->attrs = (total & NHI_DESC_LEN_MASK) |
	    (((uint32_t)pdf & 0xf) << NHI_DESC_EOF_SHIFT) |
	    (((uint32_t)pdf & 0xf) << NHI_DESC_SOF_SHIFT) |
	    ((uint32_t)(NHI_RING_DESC_POSTED | NHI_RING_DESC_INTERRUPT) <<
	     NHI_DESC_FLAGS_SHIFT);
	d->time = 0;

	tx->head = (tx->head + 1) % tx->count;
	nhi_ring_doorbell(tx);

	/*
	 * Ring-engine sanity (bootverbose): after a short delay the TX index's
	 * consumer half should advance to the producer (HW transmitted the
	 * frame) and the descriptor should come back with COMPLETED set.  This
	 * is the V4.3/dma_selftest-style readback that proved, on Meteor Lake,
	 * that the transport works even when the ICM declines to answer.
	 */
	if (bootverbose) {
		DELAY(2000);
		device_printf(sc->dev, "ctl tx: desc.attrs=0x%08x txidx=0x%08x "
		    "txopt=0x%08x rxidx=0x%08x notify=0x%08x\n", d->attrs,
		    nhi_read(sc, NHI_REG_TX_RING_BASE + NHI_RING_INDEX),
		    nhi_read(sc, NHI_REG_TX_OPTIONS_BASE + NHI_RING_OPTS_FLAGS),
		    nhi_read(sc, NHI_REG_RX_RING_BASE + NHI_RING_INDEX),
		    nhi_read(sc, NHI_REG_RING_NOTIFY_BASE));
	}
	return (0);
}

/*
 * Poll ring 0 for a received control frame (the descriptor's COMPLETED flag is
 * set by the NHI).  Copies the payload out (caller's *len is the buffer size on
 * entry, the frame length on return), returns the PDF in *pdf, and re-posts the
 * buffer.  ETIMEDOUT if nothing arrives within timeout_ms.
 */
int
nhi_ctl_rx(struct nhi_softc *sc, void *buf, u_int *len, uint8_t *pdf,
    int timeout_ms)
{
	struct nhi_ring *rx = &sc->ring0_rx;
	struct nhi_ring_desc *d;
	uint8_t *src;
	u_int slot, framelen, ndw, j;
	int i;

	if (!sc->ring0_up)
		return (ENXIO);

	/* timeout_ms == 0 => a single non-blocking check (used by the event loop). */
	for (i = 0; ; i++) {
		slot = rx->tail;
		d = &rx->desc[slot];
		/* descriptor ring is coherent; read flags directly */
		if (((d->attrs >> NHI_DESC_FLAGS_SHIFT) &
		    NHI_RING_DESC_COMPLETED) != 0) {
			framelen = d->attrs & NHI_DESC_LEN_MASK;
			bus_dmamap_sync(rx->buf_tag, rx->buf_map,
			    BUS_DMASYNC_POSTREAD);
			src = (uint8_t *)rx->buf + (size_t)slot * rx->frame_size;
			/* strip the 4-byte CRC footer */
			framelen = (framelen >= 4) ? framelen - 4 : 0;
			/* big-endian -> native per dword (be32_to_cpu_array) */
			ndw = framelen / 4;
			if (ndw * 4 > *len)
				ndw = *len / 4;
			for (j = 0; j < ndw; j++)
				le32enc((uint8_t *)buf + j * 4,
				    be32dec(src + j * 4));
			*len = ndw * 4;
			*pdf = (d->attrs >> NHI_DESC_EOF_SHIFT) & 0xf;

			/*
			 * Re-post at HEAD (the rolling hole), NOT at the slot
			 * just consumed (Linux ring_write_descriptors).  The
			 * ring runs with count-1 posted and one hole; the hole
			 * must roll forward or the HW fill pointer reaches the
			 * never-posted slot and stalls forever - seen as "RX
			 * goes deaf after ~15 frames" (one ring's worth).
			 */
			nhi_rx_desc_post(rx, rx->head);
			bus_dmamap_sync(rx->buf_tag, rx->buf_map,
			    BUS_DMASYNC_PREREAD);
			rx->tail = (rx->tail + 1) % rx->count;
			rx->head = (rx->head + 1) % rx->count;
			nhi_ring_doorbell(rx);
			return (0);
		}
		if (i >= timeout_ms * 10)	/* checked at least once */
			break;
		DELAY(100);
	}
	return (ETIMEDOUT);
}

/* ISR bottom-end for ring 0 (debug accounting; RX is reaped by polling). */
void
nhi_ring0_intr(struct nhi_softc *sc)
{
	sc->intr_count++;
	/*
	 * NOTIFY status is level, MSI is edge.  Explicitly write INT_CLEAR to
	 * reset the status bits and re-arm the next 0->1 edge (Linux
	 * nhi_clear_interrupt non-auto-clear path: iowrite32(~0,
	 * REG_RING_INT_CLEAR + ring)).  Read-to-clear alone did NOT reset the
	 * edge latch on MTL (see nhi_ring0_setup).  hop_count=12 => all our
	 * ring status bits (TX/RX/overflow, max bit 26) fit dword0, but clear
	 * dword1 too for safety on higher hop counts.
	 */
	(void)nhi_read(sc, NHI_REG_RING_NOTIFY_BASE);
	nhi_write(sc, NHI_REG_RING_INT_CLEAR, ~0U);
	nhi_write(sc, NHI_REG_RING_INT_CLEAR + 4, ~0U);
}

/*
 * Data-ring (network service) transport - Phase 3b.  Unlike ring 0 these frames
 * are RAW: the ThunderboltIP data frame (12-byte LE header + payload) goes on
 * the wire byte-for-byte, with NO per-dword byteswap and NO CRC32C footer.  TX
 * = NHI_DATA_TX_RING, RX = NHI_DATA_RX_RING; both NHI_DATA_RING_COUNT x
 * NHI_DATA_FRAME_SIZE.
 */
int
nhi_data_setup(struct nhi_softc *sc)
{
	struct nhi_ring *tx = &sc->data_tx;
	struct nhi_ring *rx = &sc->data_rx;
	u_int i;
	int error;

	if (sc->data_up)
		return (0);

	error = nhi_ring_alloc(sc, tx, NHI_DATA_TX_RING, true,
	    NHI_DATA_RING_COUNT, NHI_DATA_FRAME_SIZE);
	if (error != 0)
		return (error);
	error = nhi_ring_alloc(sc, rx, NHI_DATA_RX_RING, false,
	    NHI_DATA_RING_COUNT, NHI_DATA_FRAME_SIZE);
	if (error != 0) {
		nhi_ring_free(tx);
		return (error);
	}

	/* Frame mode (Linux RING_FLAG_FRAME): RX filters ThunderboltIP data
	 * PDFs and runs E2E flow control fed by our TX ring (tbnet). */
	tx->frame_mode = true;
	tx->e2e = true;
	rx->frame_mode = true;
	rx->sof_mask = 1u << NHI_TBIP_PDF_FRAME_START;
	rx->eof_mask = 1u << NHI_TBIP_PDF_FRAME_END;
	rx->e2e = true;
	rx->e2e_hop = NHI_DATA_TX_RING;

	nhi_ring_program(tx);
	nhi_ring_program(rx);

	/* Prime the RX ring before the tunnel is enabled (Linux ordering). */
	bus_dmamap_sync(rx->buf_tag, rx->buf_map, BUS_DMASYNC_PREREAD);
	for (i = 0; i < rx->count - 1; i++)
		nhi_rx_desc_post(rx, i);
	rx->head = rx->count - 1;
	rx->tail = 0;
	nhi_ring_doorbell(rx);
	nhi_ring_int_enable(rx, true);

	sc->data_up = true;
	device_printf(sc->dev, "data rings up: tx hop %d, rx hop %d (%u x %u)\n",
	    NHI_DATA_TX_RING, NHI_DATA_RX_RING, NHI_DATA_RING_COUNT,
	    NHI_DATA_FRAME_SIZE);
	return (0);
}

void
nhi_data_teardown(struct nhi_softc *sc)
{
	if (!sc->data_up)
		return;
	nhi_ring_int_enable(&sc->data_rx, false);
	nhi_ring_disable(&sc->data_tx);
	nhi_ring_disable(&sc->data_rx);
	nhi_ring_free(&sc->data_tx);
	nhi_ring_free(&sc->data_rx);
	sc->data_up = false;
}

/*
 * Transmit one ThunderboltIP data frame (caller built the 12-byte header +
 * payload in `frame`).  Raw copy - no byteswap, no CRC.  SOF/EOF carry the
 * FRAME_START/FRAME_END markers.  Advances head like the control TX; a 256-deep
 * ring is ample for single-queue v1 (TX-completion reaping is a perf item).
 */
int
nhi_data_tx(struct nhi_softc *sc, const void *frame, u_int len)
{
	struct nhi_ring *tx = &sc->data_tx;
	struct nhi_ring_desc *d;
	uint8_t *fbuf;
	u_int slot;

	if (!sc->data_up)
		return (ENXIO);
	if (len == 0 || len > tx->frame_size)
		return (EINVAL);

	slot = tx->head;
	d = &tx->desc[slot];
	/*
	 * TX flow control: if the slot we are about to reuse is still owned
	 * by the hardware (POSTED but not COMPLETED), the ring is full -
	 * overwriting it silently lost frames at line rate (TCP saw them as
	 * ~15k retransmits per 10 s).
	 */
	{
		uint32_t dflags = (d->attrs >> NHI_DESC_FLAGS_SHIFT) & 0xf;
		if ((dflags & NHI_RING_DESC_POSTED) != 0 &&
		    (dflags & NHI_RING_DESC_COMPLETED) == 0)
			return (ENOBUFS);
	}
	fbuf = (uint8_t *)tx->buf + (size_t)slot * tx->frame_size;
	memcpy(fbuf, frame, len);
	bus_dmamap_sync(tx->buf_tag, tx->buf_map, BUS_DMASYNC_PREWRITE);

	d = &tx->desc[slot];
	d->phys = tx->buf_busaddr + (uint64_t)slot * tx->frame_size;
	d->attrs = (len & NHI_DESC_LEN_MASK) |
	    (((uint32_t)NHI_TBIP_PDF_FRAME_END & 0xf) << NHI_DESC_EOF_SHIFT) |
	    (((uint32_t)NHI_TBIP_PDF_FRAME_START & 0xf) << NHI_DESC_SOF_SHIFT) |
	    ((uint32_t)(NHI_RING_DESC_POSTED | NHI_RING_DESC_INTERRUPT) <<
	     NHI_DESC_FLAGS_SHIFT);
	d->time = 0;

	tx->head = (tx->head + 1) % tx->count;
	nhi_ring_doorbell(tx);
	return (0);
}

/*
 * Reap one received data frame (raw bytes: header + payload).  Non-blocking:
 * 0 with *len set to the frame length, or ETIMEDOUT if the ring is empty.
 * Re-posts the slot and advances the consumer.
 */
int
nhi_data_rx(struct nhi_softc *sc, void *buf, u_int *len)
{
	struct nhi_ring *rx = &sc->data_rx;
	struct nhi_ring_desc *d;
	uint8_t *src;
	u_int slot, framelen;

	if (!sc->data_up)
		return (ENXIO);

	slot = rx->tail;
	d = &rx->desc[slot];
	if (((d->attrs >> NHI_DESC_FLAGS_SHIFT) & NHI_RING_DESC_COMPLETED) == 0)
		return (ETIMEDOUT);

	framelen = d->attrs & NHI_DESC_LEN_MASK;
	bus_dmamap_sync(rx->buf_tag, rx->buf_map, BUS_DMASYNC_POSTREAD);
	src = (uint8_t *)rx->buf + (size_t)slot * rx->frame_size;
	if (framelen > *len)
		framelen = *len;
	memcpy(buf, src, framelen);
	*len = framelen;

	/* Re-post at HEAD (rolling hole), not the consumed slot - same wrap
	 * rule as ring 0 (see nhi_ctl_rx); otherwise the data RX ring goes
	 * deaf after one ring's worth of frames. */
	nhi_rx_desc_post(rx, rx->head);
	bus_dmamap_sync(rx->buf_tag, rx->buf_map, BUS_DMASYNC_PREREAD);
	rx->tail = (rx->tail + 1) % rx->count;
	rx->head = (rx->head + 1) % rx->count;
	nhi_ring_doorbell(rx);
	return (0);
}

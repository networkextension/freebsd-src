/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_var.h - softc and internal interfaces for nhi(4).
 */
#ifndef _NHI_VAR_H_
#define _NHI_VAR_H_

#include <sys/lock.h>
#include <sys/mutex.h>	/* struct mtx is a by-value softc member (tbt_lock) */

struct nhi_ring_desc;
struct mbuf;
struct ifnet;

/* Control ring 0 geometry (small: control frames are tiny). */
#define	NHI_CTL_RING		0	/* hop number of the control ring */
#define	NHI_CTL_RING_COUNT	16	/* descriptors per direction */
#define	NHI_CTL_FRAME_SIZE	256	/* per-frame buffer (Linux TB_FRAME_SIZE) */

/*
 * A descriptor ring (TX or RX) in coherent DMA memory plus a block of
 * streaming frame buffers (one frame_size slot per descriptor).  ring 0 is the
 * control channel; data rings come from the same allocator.
 */
struct nhi_ring {
	struct nhi_softc	*sc;
	int			hop;		/* ring number */
	bool			is_tx;
	u_int			count;		/* descriptors in the ring */
	u_int			frame_size;	/* bytes per frame buffer */

	/* Descriptor ring (coherent). */
	bus_dma_tag_t		desc_tag;
	bus_dmamap_t		desc_map;
	struct nhi_ring_desc	*desc;		/* coherent descriptor array */
	bus_addr_t		desc_busaddr;	/* its DMA address */

	/* Frame buffers (streaming; need bus_dmamap_sync). */
	bus_dma_tag_t		buf_tag;
	bus_dmamap_t		buf_map;
	void			*buf;		/* KVA of count*frame_size */
	bus_addr_t		buf_busaddr;	/* DMA base of the block */

	u_int			head;		/* next descriptor we post */
	u_int			tail;		/* next the NHI will complete */
};

struct nhi_softc {
	device_t		dev;

	/* BAR0: memory-mapped NHI registers. */
	struct resource		*regs;
	int			regs_rid;
	bus_space_tag_t		regs_bt;
	bus_space_handle_t	regs_bh;

	/* MSI/MSI-X. */
	struct resource		*irq;
	int			irq_rid;
	void			*intrhand;
	int			msix_count;	/* vectors allocated (MSI or MSI-X) */

	/* Capabilities read at attach (nhi_probe). */
	uint32_t		hop_count;	/* number of paths/rings */
	uint32_t		fw_sts;		/* firmware status snapshot */
	bool			icm_present;	/* ICM firmware enabled */
	bool			force_power;	/* integrated: VSEC force-power */

	/* Control ring 0 (nhi_ring0_setup). */
	struct nhi_ring		ring0_tx;
	struct nhi_ring		ring0_rx;
	bool			ring0_up;
	uint32_t		intr_count;	/* ISR invocations (debug) */

	/* Ring-0 RX event loop (nhi_event_loop). */
	struct thread		*event_td;
	volatile int		event_run;

	/* XDomain state (nhi_xdomain.c). */
	bool			has_peer;	/* an XDOMAIN_CONNECTED seen */
	uint8_t			local_uuid[16];	/* our domain UUID (from ICM) */
	uint8_t			peer_uuid[16];	/* the remote host's UUID */
	uint32_t		local_route_hi;
	uint32_t		local_route_lo;
	uint32_t		peer_route_hi;	/* route to reach the peer (TX) */
	uint32_t		peer_route_lo;
	uint32_t		prop_block[64];	/* our serialized property dir */
	u_int			prop_dwords;	/* valid dwords in prop_block */
	uint32_t		prop_generation;

	/* ThunderboltIP / networking (nhi_tbip.c, Phase 3). */
	uint8_t			tbt_mac[6];	/* our synthetic MAC */
	uint8_t			peer_mac[6];	/* peer's MAC (from login resp) */
	uint32_t		local_tx_hopid;	/* our TX HopID (in our login) */
	uint32_t		remote_tx_hopid;/* peer's TX HopID (from its login) */
	uint32_t		local_rx_hopid;	/* our RX (in) HopID */
	bool			tbip_login_sent;	/* our login ACKed */
	bool			tbip_login_received;	/* we replied to peer login */
	bool			paths_approved;
	int			login_last;	/* ticks of our last proactive LOGIN */

	/* if_tbt(4) data plane (Phase 3b): rings + ifnet + RX reassembly. */
	struct ifnet		*ifp;		/* tbt0 */
	struct mtx		tbt_lock;	/* guards TX + reassembly state */
	bool			data_up;	/* data rings allocated/programmed */
	struct nhi_ring		data_tx;	/* hop NHI_DATA_TX_RING */
	struct nhi_ring		data_rx;	/* hop NHI_DATA_RX_RING */
	uint16_t		tx_frame_id;	/* per-packet id counter (TX) */
	struct mbuf		*rx_m;		/* packet being reassembled */
	uint16_t		rx_id;		/* reassembly: current frame_id */
	uint32_t		rx_count;	/* reassembly: expected frame_count */
	uint32_t		rx_index;	/* reassembly: next frame_index */
};

/* nhi_mailbox.c - firmware mailbox primitive (mbox_ready, V1.3/V5.1). */
int		nhi_mailbox_cmd(struct nhi_softc *sc, uint8_t cmd, uint32_t data);
int		nhi_mailbox_mode(struct nhi_softc *sc);
const char	*nhi_fw_mode_str(int mode);

/* nhi_ring.c - bus_dma(9) descriptor-ring engine (dma_selftest, V4.3). */
int		nhi_ring_alloc(struct nhi_softc *sc, struct nhi_ring *r,
		    int hop, bool is_tx, u_int count, u_int frame_size);
void		nhi_ring_free(struct nhi_ring *r);
void		nhi_ring_program(struct nhi_ring *r);
void		nhi_ring_doorbell(struct nhi_ring *r);

/* nhi_ring.c - ring 0 control transport (ring0_rx, V1.4/V5.1). */
int		nhi_ring0_setup(struct nhi_softc *sc);
void		nhi_ring0_teardown(struct nhi_softc *sc);
int		nhi_ctl_tx(struct nhi_softc *sc, uint8_t pdf, const void *data,
		    u_int len);
int		nhi_ctl_rx(struct nhi_softc *sc, void *buf, u_int *len,
		    uint8_t *pdf, int timeout_ms);
void		nhi_ring0_intr(struct nhi_softc *sc);

/* nhi_icm.c - ICM DRIVER_READY handshake (TP-1b / the ICM gate). */
int		nhi_icm_driver_ready(struct nhi_softc *sc);
void		nhi_icm_drain(struct nhi_softc *sc, int n);
void		nhi_icm_handle_event(struct nhi_softc *sc, const uint8_t *frame,
		    u_int len);

/* nhi_xdomain.c - XDomain discovery responder (Phase 2, V2.1). */
void		nhi_xdomain_build_dir(struct nhi_softc *sc);
void		nhi_xdomain_handle(struct nhi_softc *sc, const uint8_t *frame,
		    u_int len);
void		nhi_xdomain_notify_changed(struct nhi_softc *sc);
void		nhi_xdomain_request_props(struct nhi_softc *sc);

/* nhi_tbip.c - Apple ThunderboltIP login over ring 0 (Phase 3). */
void		nhi_tbip_init(struct nhi_softc *sc);
void		nhi_tbip_handle(struct nhi_softc *sc, const uint8_t *frame,
		    u_int len);
void		nhi_tbip_start_login(struct nhi_softc *sc);
void		nhi_tbip_logout(struct nhi_softc *sc);

/* nhi_ring.c - data-ring (network service) transport, raw DMA (Phase 3b). */
int		nhi_data_setup(struct nhi_softc *sc);
void		nhi_data_teardown(struct nhi_softc *sc);
int		nhi_data_tx(struct nhi_softc *sc, const void *frame, u_int len);
int		nhi_data_rx(struct nhi_softc *sc, void *buf, u_int *len);

/* nhi_icm.c - APPROVE_XDOMAIN_PATHS: program the bidirectional data tunnel. */
int		nhi_icm_approve_xdomain_paths(struct nhi_softc *sc);

/* if_tbt.c - ThunderboltIP ifnet "tbt0" (Phase 3b). */
int		nhi_tbt_connect(struct nhi_softc *sc);
void		nhi_tbt_disconnect(struct nhi_softc *sc);
void		nhi_tbt_rx_poll(struct nhi_softc *sc);

/* Register accessors - all NHI registers are 32-bit, LE on arm64/x86. */
static __inline uint32_t
nhi_read(struct nhi_softc *sc, bus_size_t off)
{
	return (bus_space_read_4(sc->regs_bt, sc->regs_bh, off));
}

static __inline void
nhi_write(struct nhi_softc *sc, bus_size_t off, uint32_t val)
{
	bus_space_write_4(sc->regs_bt, sc->regs_bh, off, val);
}

#endif /* _NHI_VAR_H_ */

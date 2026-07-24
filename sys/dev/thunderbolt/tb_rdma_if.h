/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tb_rdma_if.h - native C API that thunderbolt.ko exports to an out-of-tree
 * RDMA provider module (tbrdma.ko).  Mirrors Linux mlx4's split between the
 * native transport driver (mlx4_core) and the LinuxKPI/OFED provider
 * (mlx4_ib): the transport owns the hardware and hands the provider an opaque
 * per-controller handle plus pull-style getters (mlx4_register_interface() +
 * struct mlx4_interface, the add/remove-per-device consumer model).
 *
 * Deliberately NO ib_* or LinuxKPI types cross this boundary - only native
 * device_t / bus_dma_tag_t / fixed-width integers - so thunderbolt.ko never
 * has to include the OFED headers, and the two DMA/type worlds only meet
 * inside tbrdma.ko.
 */
#ifndef _TB_RDMA_IF_H_
#define _TB_RDMA_IF_H_

#include <sys/types.h>
#include <machine/bus.h>

struct nhi_softc;
struct tb_icm;
struct tbnet_softc;
struct ifnet;

/*
 * Opaque per-controller handle.  Defined in tb_rdma_shim.c; tbrdma.ko only
 * ever holds the pointer and passes it back to the getters below.
 */
struct tb_rdma_dev;

/*
 * Consumer interface.  tbrdma.ko registers one of these; thunderbolt.ko calls
 * ->add() for every controller that is already up and for each one that comes
 * up later, and ->remove() when a controller detaches or the consumer
 * unregisters.  Both callbacks run in a sleepable context.
 */
struct tb_rdma_interface {
	void	(*add)(struct tb_rdma_dev *);
	void	(*remove)(struct tb_rdma_dev *);
};

int	tb_rdma_register_interface(struct tb_rdma_interface *intf);
void	tb_rdma_unregister_interface(struct tb_rdma_interface *intf);

/* Pull-style getters: opaque handle -> native facts about the controller. */
device_t	tb_rdma_get_dev(struct tb_rdma_dev *tbd);
bus_dma_tag_t	tb_rdma_get_parent_dmat(struct tb_rdma_dev *tbd);
int		tb_rdma_get_unit(struct tb_rdma_dev *tbd);
void		tb_rdma_get_uuid(struct tb_rdma_dev *tbd, uint8_t out[16]);

/* Called by nhi(4) attach (post-init) / detach. */
void	tb_rdma_controller_ready(struct nhi_softc *sc);
void	tb_rdma_controller_gone(struct nhi_softc *sc);

/*
 * ---- M1b: data-ring + hop programming for an RDMA QP ----
 *
 * These let tbrdma.ko build a QP out of a pair of NHI FRAME-mode E2E rings and
 * program the fabric hop table, without importing any nhi/router types.
 */

/* Peer/session facts, pulled from struct tb_icm (avoids the single-callback
 * slot conflict, R1).  Returns 0 and fills *out when a peer session is live. */
struct tb_rdma_peer {
	uint8_t		has_peer;
	uint8_t		paths_approved;
	uint8_t		lane;			/* peer_route_lo & 0x3f, min 1 */
	uint32_t	route_hi;
	uint32_t	route_lo;
	u_int		local_tx_hopid;		/* our announced TX HopID */
	u_int		remote_tx_hopid;	/* peer's TX HopID (our RX) */
	uint8_t		peer_uuid[16];
};
int	tb_rdma_get_peer(struct tb_rdma_dev *tbd, struct tb_rdma_peer *out);

/* Implemented in tb_icm.c (where struct tb_icm is visible). */
int	tb_icm_get_peer(struct tb_icm *icm, struct tb_rdma_peer *out);
struct tbnet_softc *tb_icm_get_net(struct tb_icm *icm);

/*
 * The resident tbt0 ifnet, ref-held (if_ref).  tbrdma.ko returns this from the
 * ib_device get_netdev op so the RoCE GID table populates from tbt0's IP
 * (GID = the Thunderbolt-IP address, Apple's model).  Caller releases with
 * if_rele (the OFED core does dev_put after use).  NULL if no net softc yet.
 * (if_t == struct ifnet *; struct form avoids a <net/if.h> dep in this header.)
 */
struct ifnet *tb_rdma_get_netdev(struct tb_rdma_dev *tbd);

/* Opaque data-ring handle (wraps struct nhi_ring_pair). */
struct tb_rdma_ring;

/*
 * Create a FRAME-mode + E2E data ring (frame_mode=1, e2e=1).  ringnum must be
 * >= 3 (rings 0 control, 1/2 if_tbt); depths are powers of two.  For an RX
 * ring pass sof_mask/eof_mask; a TX ring passes 0/0.  e2e_hopid is the paired
 * TX ring number.
 */
int	tb_rdma_ring_create(struct tb_rdma_dev *tbd, u_int ringnum,
	    u_int tx_depth, u_int rx_depth, uint16_t frame_size,
	    uint8_t e2e_hopid, uint16_t sof_mask, uint16_t eof_mask,
	    struct tb_rdma_ring **out);
int	tb_rdma_ring_start(struct tb_rdma_ring *r);
int	tb_rdma_ring_stop(struct tb_rdma_ring *r);
void	tb_rdma_ring_destroy(struct tb_rdma_ring *r);

/*
 * Hand a ring to userspace: stop the kernel RX poll callout so userspace is
 * the sole drainer of this ring's descriptors (kernel-bypass data path).  Call
 * after tb_rdma_ring_start().  Avoids the double-drain race (kernel rxpoll_co
 * vs userspace polling the same DONE bits).
 */
void	tb_rdma_ring_set_userspace(struct tb_rdma_ring *r);

/*
 * Diagnostic (#85): read back a QP's live NHI ring-table state - TX/RX BASE0
 * (VALID / E2E / paired E2E HopID) and the PI/CI doorbells.  The peer's
 * transmitter is fully E2E-gated, so an unarmed RX E2E bit or a zero paired
 * HopID silently stops credit replenishment and stalls multi-frame messages.
 */
void	tb_rdma_dump_ring_regs(struct tb_rdma_dev *tbd, u_int txring,
	    u_int rxring);

/*
 * Physical layout of a ring's DMA memory, for mmap to userspace.  The
 * descriptor ring and frame pool are physically contiguous bus_dmamem_alloc
 * buffers, so one phys base + size covers each (no IOMMU on this NHI, so bus
 * address == physical address).
 */
struct tb_rdma_ring_mem {
	uint64_t	desc_phys;	/* descriptor ring bus/phys base */
	uint32_t	desc_size;	/* 16 * (tx_depth + rx_depth) */
	uint64_t	frames_phys;	/* frame pool bus/phys base */
	uint32_t	frames_size;	/* (tx_depth + rx_depth) * frame_size */
	uint16_t	tx_depth;
	uint16_t	rx_depth;
	uint16_t	frame_size;
	uint8_t		ring_num;
};
void	tb_rdma_ring_mmap_info(struct tb_rdma_ring *r,
	    struct tb_rdma_ring_mem *out);

/*
 * BAR0 doorbell window: physical base + a size covering every ring's TX
 * (0x8 + N*16) and RX (0x8008 + N*16) producer-index doorbell.
 */
void	tb_rdma_doorbell_bar(struct tb_rdma_dev *tbd, uint64_t *bar_pa,
	    uint32_t *size);

/*
 * Program one fabric hop-table entry on the root router.  RX hop is installed
 * on the lane adapter (out port = NHI, index = the peer's TX HopID); TX hop is
 * installed on the NHI adapter (out port = lane, index = our TX ring), with
 * egress flow control set.  credits = TB_SWCM_HOP_CREDITS (14) unless noted.
 */
int	tb_rdma_program_rx_hop(struct tb_rdma_dev *tbd, uint8_t lane,
	    u_int remote_tx_hopid, u_int rx_ring, u_int credits);
int	tb_rdma_program_tx_hop(struct tb_rdma_dev *tbd, u_int local_tx_hopid,
	    uint8_t lane, u_int tx_ring, u_int credits);
void	tb_rdma_invalidate_hop(struct tb_rdma_dev *tbd, uint8_t adap,
	    u_int hopid);

/* Read back a hop entry (2 dwords) for verification/self-test. */
int	tb_rdma_read_hop(struct tb_rdma_dev *tbd, uint8_t adap, u_int hopid,
	    uint32_t out[2]);

#endif /* _TB_RDMA_IF_H_ */

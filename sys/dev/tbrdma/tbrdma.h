/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tbrdma.h - private definitions for the Thunderbolt RDMA ib_device provider.
 *
 * tbrdma.ko is the LinuxKPI/OFED half of the split described in
 * <dev/thunderbolt/tb_rdma_if.h>: it registers an ib_device per Thunderbolt
 * controller and maps the (narrow, UC-only) verbs surface Apple's
 * AppleThunderboltRDMA exposes onto our native nhi(4) transport.  M1a
 * registers the device and answers the query_* verbs only; QP/MR/CQ come
 * later (M1b/M2).
 *
 * Structure layout follows sys/dev/mlx4/mlx4_ib: the embedded ib_* object is
 * the FIRST member of every driver subclass so the core's plain
 * ib_alloc_device()/rdma_zalloc_drv_obj() container math (INIT_RDMA_OBJ_SIZE,
 * which BUILD_BUG_ON_ZEROs a non-zero offsetof) is satisfied.
 */
#ifndef _TBRDMA_H_
#define _TBRDMA_H_

#include <linux/module.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <rdma/ib_verbs.h>

#include <dev/thunderbolt/tb_rdma_if.h>

#define	TBRDMA_MAX_QP		10	/* Apple cap */
#define	TBRDMA_MAX_QP_WR	4095	/* Apple cap */
#define	TBRDMA_MAX_SGE		1
#define	TBRDMA_MAX_MSG_SZ	(4095u * 4096u)	/* 16,773,120 B */

/*
 * Ring/hop layout.  Rings 0 (control), 1/2 (if_tbt data) are reserved, so RDMA
 * QP rings start at 3.  Fabric TX HopIDs for RDMA start well clear of if_tbt's
 * (local_tx_hopid = 8) to avoid clobbering the live tbt0 path.
 */
#define	TBRDMA_FIRST_RING	3
#define	TBRDMA_NUM_RINGS	32
#define	TBRDMA_RING_DEPTH	256	/* power of two */
#define	TBRDMA_FRAME_SIZE	4096
#define	TBRDMA_TX_HOPID_BASE	16	/* our per-QP fabric TX HopID base */
#define	TBRDMA_HOP_CREDITS	14
#define	TBRDMA_NHI_PORT_ADAP	7	/* NHI adapter port (== TB_RDMA_NHI_PORT) */

/*
 * create_qp udata response: ring metadata userspace needs to drive the
 * kernel-bypass data path (frame-pool BUS addresses for descriptors + geometry).
 * Shared verbatim with tbrdma-userspace/tbrdma.h.
 */
struct tbrdma_uresp_create_qp {
	uint64_t	tx_frames_phys;
	uint64_t	rx_frames_phys;
	uint32_t	tx_ringnum;
	uint32_t	rx_ringnum;
	uint32_t	tx_depth;
	uint32_t	rx_depth;
	uint32_t	frame_size;
	uint32_t	pad;
};

/*
 * mmap() offset selectors (as page indices, i.e. vma->vm_pgoff): userspace
 * mmaps each region of its QP at these fixed offsets on the uverbs fd.
 */
#define	TBRDMA_MMAP_DOORBELL	0	/* BAR0 doorbell window (io) */
#define	TBRDMA_MMAP_TX_DESC	1	/* TX ring descriptor memory */
#define	TBRDMA_MMAP_TX_FRAMES	2	/* TX ring frame pool */
#define	TBRDMA_MMAP_RX_DESC	3	/* RX ring descriptor memory */
#define	TBRDMA_MMAP_RX_FRAMES	4	/* RX ring frame pool */

struct tbrdma_dev {
	struct ib_device	ib_dev;		/* MUST be first */
	struct tb_rdma_dev	*tbd;		/* opaque native handle */
	u8			gid[16];	/* GID[0] placeholder (M1a) */
	struct mutex		res_lock;	/* guards ring_map */
	u32			ring_map;	/* bit N set => ring N in use */
};

struct tbrdma_cq {
	struct ib_cq		ibcq;		/* MUST be first */
};

struct tbrdma_mr {
	struct ib_mr		ibmr;		/* MUST be first */
	struct vm_page		**pages;	/* held user pages */
	int			npages;
	u64			dma_base;	/* NHI bus address of MR start */
	u32			region_id;	/* compact handle (== lkey/rkey) */
	size_t			length;
};

struct tbrdma_qp {
	struct ib_qp		ibqp;		/* MUST be first */
	struct tbrdma_dev	*tdev;
	struct tb_rdma_ring	*tx_ring;
	struct tb_rdma_ring	*rx_ring;
	u_int			tx_ringnum;
	u_int			rx_ringnum;
	u_int			local_tx_hopid;	/* our fabric TX HopID */
	u_int			rx_hopid;	/* peer's TX HopID = our RX index */
	u8			lane;		/* fabric lane adapter */
	u32			dest_qpn;	/* remote QPN */
	u8			dgid[16];	/* remote GID */
	enum ib_qp_state	state;
	bool			userspace;	/* rings handed to userspace */
	bool			rx_hop_done;
	bool			tx_hop_done;
	bool			rx_started;
	bool			tx_started;
};

struct tbrdma_ucontext {
	struct ib_ucontext	ibucontext;	/* MUST be first */
	struct tbrdma_qp	*qp;		/* userspace QP for mmap lookup */
};

struct tbrdma_pd {
	struct ib_pd		ibpd;		/* MUST be first */
};

static inline struct tbrdma_dev *
to_tdev(struct ib_device *ibdev)
{
	return container_of(ibdev, struct tbrdma_dev, ib_dev);
}

static inline struct tbrdma_ucontext *
to_tucontext(struct ib_ucontext *ibucontext)
{
	return container_of(ibucontext, struct tbrdma_ucontext, ibucontext);
}

static inline struct tbrdma_pd *
to_tpd(struct ib_pd *ibpd)
{
	return container_of(ibpd, struct tbrdma_pd, ibpd);
}

static inline struct tbrdma_cq *
to_tcq(struct ib_cq *ibcq)
{
	return container_of(ibcq, struct tbrdma_cq, ibcq);
}

static inline struct tbrdma_qp *
to_tqp(struct ib_qp *ibqp)
{
	return container_of(ibqp, struct tbrdma_qp, ibqp);
}

static inline struct tbrdma_mr *
to_tmr(struct ib_mr *ibmr)
{
	return container_of(ibmr, struct tbrdma_mr, ibmr);
}

/*
 * Internal QP engine (tbrdma_qp.c) - the real create/modify/destroy logic,
 * called both by the ib_* verb methods and by the white-box self-test.
 */
struct tbrdma_qp *tbrdma_qp_alloc(struct tbrdma_dev *, u32 max_send_wr,
	    u32 max_recv_wr);
int	tbrdma_qp_modify(struct tbrdma_qp *, enum ib_qp_state new_state,
	    u32 dest_qpn, const u8 dgid[16]);
void	tbrdma_qp_free(struct tbrdma_qp *);
int	tbrdma_selftest(struct tbrdma_dev *);
int	tbrdma_mmap(struct ib_ucontext *, struct vm_area_struct *);

/* tbrdma_verbs.c */
int	tbrdma_query_device(struct ib_device *, struct ib_device_attr *,
	    struct ib_udata *);
/* Live-tunable RX-ring per-PDF frame-accept masks (dev.tbrdma.rx_{sof,eof}_mask). */
extern int tbrdma_rx_sof_mask;
extern int tbrdma_rx_eof_mask;
/* #85 diagnostic: 0 = keep the kernel RX drainer (and its per-frame E2E credit
 * return) attached even for a userspace QP.  See tbrdma_main.c. */
extern int tbrdma_bypass_drain;

int	tbrdma_query_port(struct ib_device *, u8, struct ib_port_attr *);
enum rdma_link_layer tbrdma_get_link_layer(struct ib_device *, u8);
if_t	tbrdma_get_netdev(struct ib_device *, u8);
int	tbrdma_query_gid(struct ib_device *, u8, int, union ib_gid *);
int	tbrdma_add_gid(const struct ib_gid_attr *, void **);
int	tbrdma_del_gid(const struct ib_gid_attr *, void **);
int	tbrdma_query_pkey(struct ib_device *, u8, u16, u16 *);
int	tbrdma_get_port_immutable(struct ib_device *, u8,
	    struct ib_port_immutable *);
int	tbrdma_alloc_ucontext(struct ib_ucontext *, struct ib_udata *);
void	tbrdma_dealloc_ucontext(struct ib_ucontext *);
int	tbrdma_alloc_pd(struct ib_pd *, struct ib_udata *);
void	tbrdma_dealloc_pd(struct ib_pd *, struct ib_udata *);

/*
 * Mandatory-verb stubs (ib_device.c mandatory_table).  ib_register_device
 * rejects a device that leaves any of these NULL, so M1a provides
 * -EOPNOTSUPP stubs; the real QP/CQ/MR data path lands in M1b/M2.
 */
struct ib_qp *tbrdma_create_qp(struct ib_pd *, struct ib_qp_init_attr *,
	    struct ib_udata *);
int	tbrdma_modify_qp(struct ib_qp *, struct ib_qp_attr *, int,
	    struct ib_udata *);
int	tbrdma_destroy_qp(struct ib_qp *, struct ib_udata *);
int	tbrdma_post_send(struct ib_qp *, const struct ib_send_wr *,
	    const struct ib_send_wr **);
int	tbrdma_post_recv(struct ib_qp *, const struct ib_recv_wr *,
	    const struct ib_recv_wr **);
int	tbrdma_create_cq(struct ib_cq *, const struct ib_cq_init_attr *,
	    struct ib_udata *);
void	tbrdma_destroy_cq(struct ib_cq *, struct ib_udata *);
int	tbrdma_poll_cq(struct ib_cq *, int, struct ib_wc *);
int	tbrdma_req_notify_cq(struct ib_cq *, enum ib_cq_notify_flags);
struct ib_mr *tbrdma_get_dma_mr(struct ib_pd *, int);
struct ib_mr *tbrdma_reg_user_mr(struct ib_pd *, u64 start, u64 length,
	    u64 virt_addr, int access, struct ib_udata *);
int	tbrdma_dereg_mr(struct ib_mr *, struct ib_udata *);

#endif /* _TBRDMA_H_ */

/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tb_xdomain.h - XDomain discovery responder public interface.  The connection
 * manager creates it on the transport's ring 0, feeds it the peer/local UUIDs +
 * route on XDOMAIN_CONNECTED, and registers a service handler for the
 * ThunderboltIP login frames.
 */
#ifndef _TB_XDOMAIN_H_
#define _TB_XDOMAIN_H_

struct nhi_softc;
struct nhi_ring_pair;
struct tb_xdomain;

/* Handler for non-discovery (service-UUID) frames, e.g. the ThunderboltIP login. */
typedef void (tb_xdomain_service_cb)(void *ctx, const uint8_t *frame, u_int len);

int tb_xdomain_init(struct nhi_softc *nsc, struct nhi_ring_pair *ring0,
    struct tb_xdomain **xdp);
void tb_xdomain_fini(struct tb_xdomain *xd);
void tb_xdomain_set_peer(struct tb_xdomain *xd, const uint8_t *peer_uuid,
    const uint8_t *local_uuid, uint32_t route_hi, uint32_t route_lo);
void tb_xdomain_set_service_handler(struct tb_xdomain *xd,
    tb_xdomain_service_cb *cb, void *ctx);
void tb_xdomain_notify_changed(struct tb_xdomain *xd);
void tb_xdomain_kick(struct tb_xdomain *xd, uint32_t route_lo);

#endif /* _TB_XDOMAIN_H_ */

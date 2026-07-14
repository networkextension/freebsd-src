/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * if_tbt.h - ThunderboltIP net driver (tbt) public interface.  The interface is
 * resident: the CM calls tbnet_create() at driver load (present, link down),
 * tbnet_connect() when a peer's ThunderboltIP login + APPROVE complete (carrier
 * up), tbnet_disconnect() when the peer goes away (carrier down, interface
 * kept), and tbnet_destroy() at driver unload.
 */
#ifndef _TB_IF_TBT_H_
#define _TB_IF_TBT_H_

struct nhi_softc;
struct tbnet_softc;

int tbnet_create(struct nhi_softc *nsc, const uint8_t *mac,
    struct tbnet_softc **tsp);
int tbnet_connect(struct tbnet_softc *ts);
void tbnet_disconnect(struct tbnet_softc *ts);
void tbnet_destroy(struct tbnet_softc *ts);

#endif /* _TB_IF_TBT_H_ */

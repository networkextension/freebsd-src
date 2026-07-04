/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * if_tbt.h - ThunderboltIP net driver (tbt) public interface.  The connection
 * manager calls tbnet_attach once the ThunderboltIP login completes and
 * tbnet_detach on disconnect.
 */
#ifndef _TB_IF_TBT_H_
#define _TB_IF_TBT_H_

struct nhi_softc;
struct tbnet_softc;

int tbnet_attach(struct nhi_softc *nsc, const uint8_t *mac,
    const uint8_t *peer_mac, u_int local_tx_hopid, u_int remote_tx_hopid,
    struct tbnet_softc **tsp);
void tbnet_detach(struct tbnet_softc *ts);

#endif /* _TB_IF_TBT_H_ */

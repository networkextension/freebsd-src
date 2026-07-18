/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * tb_icm.h - ICM-mode connection manager public interface.  The attach glue
 * calls tb_icm_init when the controller firmware reports CM mode (beside the
 * in-tree HCM); it stands up XDomain + the ThunderboltIP login on ring 0 and
 * runs DRIVER_READY.
 */
#ifndef _TB_ICM_H_
#define _TB_ICM_H_

struct nhi_softc;
struct nhi_ring_pair;
struct tb_icm;

int tb_icm_init(struct nhi_softc *nsc, struct nhi_ring_pair *ring0,
    struct tb_icm **icmp);
void tb_icm_fini(struct tb_icm *icm);

#endif /* _TB_ICM_H_ */

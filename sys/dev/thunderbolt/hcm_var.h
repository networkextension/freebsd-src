/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2022 Scott Long
 *
 * Local override of the stock header: adds the serialized link-event entry
 * point (hot plug/unplug funneled onto the HCM taskqueue - the analogue of
 * Apple's single configuration thread; "no ad-hoc concurrency against the
 * fabric").
 */

#ifndef _HCM_VAR_H
#define _HCM_VAR_H

struct hcm_softc {
	u_int			debug;
	device_t		dev;
	struct nhi_softc	*nsc;

	struct task		cfg_task;
	struct task		link_task;	/* serialized plug/unplug */
	volatile int		plug_pending;	/* coalesced event flags */
	volatile int		unplug_pending;
	struct taskqueue	*taskqueue;
};

int hcm_attach(struct nhi_softc *);
int hcm_detach(struct nhi_softc *);
int hcm_router_discover(struct hcm_softc *);
void hcm_link_event(struct hcm_softc *, u_int port, bool unplug);

#endif /* _HCM_VAR_H */

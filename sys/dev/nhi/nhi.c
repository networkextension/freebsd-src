/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi(4) - Thunderbolt/USB4 Native Host Interface transport driver.
 *
 * Phase 1 of the FreeBSD Thunderbolt stack (see DESIGN.md).  This file is the
 * pure transport layer: it attaches to the NHI PCI function, maps BAR0, reads
 * the ring/hop count and firmware-mailbox capability, and wires MSI-X.  It is
 * deliberately the "nhi_probe" verification probe (DESIGN.md V1.2 / V5.0):
 * the minimal code that confirms the controller's BAR/register story before
 * any ring or mailbox traffic is attempted.
 *
 * The ring engine, control-ring (ring 0) bring-up, and the DRIVER_READY
 * mailbox handshake (mbox_ready, V1.3/V5.1) build on the primitives here and
 * land in follow-up commits.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/bus.h>
#include <sys/mbuf.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/endian.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/ethernet.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

#define	NHI_FLAG_FORCE_POWER	0x1	/* integrated: needs VSEC force-power */

struct nhi_ident {
	uint16_t	vendor;
	uint16_t	device;
	const char	*desc;
	u_int		flags;
};

static const struct nhi_ident nhi_idents[] = {
	{ NHI_VENDOR_INTEL, NHI_DEV_TR_4C_NHI,
	    "Thunderbolt 3 NHI (Titan Ridge 4C / JHL7540)" },
	{ NHI_VENDOR_INTEL, NHI_DEV_TR_4C_NHI_ALT,
	    "Thunderbolt 3 NHI (Titan Ridge 4C / JHL7540, alt id)" },
	{ NHI_VENDOR_INTEL, NHI_DEV_AR_2C_NHI,
	    "Thunderbolt 3 NHI (Alpine Ridge 2C)" },
	{ NHI_VENDOR_INTEL, NHI_DEV_AR_4C_NHI,
	    "Thunderbolt 3 NHI (Alpine Ridge 4C)" },
	{ NHI_VENDOR_INTEL, NHI_DEV_AR_LP_NHI,
	    "Thunderbolt 3 NHI (Alpine Ridge LP)" },
	{ NHI_VENDOR_INTEL, NHI_DEV_MR_NHI,
	    "Thunderbolt 4 NHI (Maple Ridge)" },
	{ NHI_VENDOR_INTEL, NHI_DEV_MTL_P_NHI0,
	    "Thunderbolt 4 NHI (Meteor Lake-P, NHI0)", NHI_FLAG_FORCE_POWER },
	{ NHI_VENDOR_INTEL, NHI_DEV_MTL_P_NHI1,
	    "Thunderbolt 4 NHI (Meteor Lake-P, NHI1)", NHI_FLAG_FORCE_POWER },
	{ 0, 0, NULL }
};

static const struct nhi_ident *
nhi_lookup(device_t dev)
{
	uint16_t vendor = pci_get_vendor(dev);
	uint16_t device = pci_get_device(dev);
	const struct nhi_ident *id;

	for (id = nhi_idents; id->desc != NULL; id++) {
		if (id->vendor == vendor && id->device == device)
			return (id);
	}
	return (NULL);
}

static int
nhi_probe(device_t dev)
{
	const struct nhi_ident *id;

	id = nhi_lookup(dev);
	if (id == NULL)
		return (ENXIO);

	device_set_desc(dev, id->desc);
	return (BUS_PROBE_DEFAULT);
}

/*
 * Map BAR0.  The NHI exposes a single 64-bit memory BAR at offset 0x10
 * (PCIR_BAR(0)); everything the host driver touches lives there.
 */
static int
nhi_map_regs(struct nhi_softc *sc)
{
	sc->regs_rid = PCIR_BAR(0);
	sc->regs = bus_alloc_resource_any(sc->dev, SYS_RES_MEMORY,
	    &sc->regs_rid, RF_ACTIVE);
	if (sc->regs == NULL) {
		device_printf(sc->dev, "cannot allocate BAR0\n");
		return (ENXIO);
	}
	sc->regs_bt = rman_get_bustag(sc->regs);
	sc->regs_bh = rman_get_bushandle(sc->regs);
	return (0);
}

/*
 * Best-effort MSI-X: the NHI uses per-ring + mailbox vectors.  For the probe
 * we only need to prove that vectors can be allocated and routed (arm64
 * GICv3/ITS); ring<->vector binding comes with the ring engine.
 */
static void
nhi_setup_intr(struct nhi_softc *sc)
{
	int avail, count, error;

	sc->msix_count = 0;

	/*
	 * Prefer MSI-X (the NHI uses per-ring + mailbox vectors); take a single
	 * control vector for now and grow with the ring engine.  On the Meteor
	 * Lake silicon the device advertises 16 MSI-X messages yet the first
	 * pci_alloc_msix() attempt was failing - so capture the real errno and
	 * fall back to MSI rather than silently giving up (TESTPLAN TP-1).
	 */
	avail = pci_msix_count(sc->dev);
	if (avail > 0) {
		count = 1;
		error = pci_alloc_msix(sc->dev, &count);
		if (error == 0 && count == 1) {
			sc->msix_count = count;
			device_printf(sc->dev, "using 1 MSI-X vector (of %d)\n",
			    avail);
			return;
		}
		device_printf(sc->dev,
		    "MSI-X alloc failed (err %d, %d advertised); trying MSI\n",
		    error, avail);
	} else {
		device_printf(sc->dev, "no MSI-X (count=%d); trying MSI\n",
		    avail);
	}

	/* Fall back to MSI; pci_release_msi() in detach covers both. */
	avail = pci_msi_count(sc->dev);
	if (avail > 0) {
		count = 1;
		error = pci_alloc_msi(sc->dev, &count);
		if (error == 0 && count >= 1) {
			sc->msix_count = count;
			device_printf(sc->dev, "using %d MSI vector (of %d)\n",
			    count, avail);
			return;
		}
		device_printf(sc->dev, "MSI alloc failed (err %d)\n", error);
	}
	device_printf(sc->dev, "no MSI/MSI-X vector allocated\n");
}

/*
 * Read the capability registers that prove BAR0 is live and tell us what we
 * are talking to: how many paths/rings the controller has, and whether the
 * ICM firmware is up (which decides ICM vs SW-CM, DESIGN.md D1).
 */
static void
nhi_read_caps(struct nhi_softc *sc)
{
	uint32_t caps, opmode, version;

	caps = nhi_read(sc, NHI_REG_CAPS);
	sc->hop_count = caps & NHI_CAPS_HOP_COUNT_MASK;
	version = (caps & NHI_CAPS_VERSION_MASK) >> NHI_CAPS_VERSION_SHIFT;

	sc->fw_sts = nhi_read(sc, NHI_REG_FW_STS);
	sc->icm_present = (sc->fw_sts & NHI_FW_STS_ICM_EN) != 0;

	opmode = nhi_mailbox_mode(sc);

	device_printf(sc->dev,
	    "caps_ver=0x%x hop_count=%u fw_sts=0x%08x icm=%s fw_mode=%s(0x%x)\n",
	    version, sc->hop_count, sc->fw_sts,
	    sc->icm_present ? "present" : "absent",
	    nhi_fw_mode_str(opmode), opmode);
}

/*
 * Force-power the integrated controller and wait for firmware-ready, via the
 * vendor-specific PCI config registers (the Intel NHI force-power sequence).  On
 * Ice Lake onward the on-die Thunderbolt controller stays powered down until
 * the host asserts this, and the connection-manager firmware will not answer
 * ring-0 ICM commands until then.  Sets the documented 0x22 DMA delay too.
 */
static int
nhi_force_power(struct nhi_softc *sc, bool on)
{
	uint32_t vs22, vs9;
	int retries;

	vs22 = pci_read_config(sc->dev, NHI_VS_CAP_22, 4);
	if (!on) {
		vs22 &= ~NHI_VS_CAP_22_FORCE_POWER;
		pci_write_config(sc->dev, NHI_VS_CAP_22, vs22, 4);
		return (0);
	}

	vs9 = pci_read_config(sc->dev, NHI_VS_CAP_9, 4);
	device_printf(sc->dev, "force-power: VS_CAP_22=0x%08x VS_CAP_9=0x%08x\n",
	    vs22, vs9);

	vs22 &= ~NHI_VS_CAP_22_DMA_DELAY_MASK;
	vs22 |= (NHI_VS_CAP_22_DMA_DELAY_VAL << NHI_VS_CAP_22_DMA_DELAY_SHIFT);
	vs22 |= NHI_VS_CAP_22_FORCE_POWER;
	pci_write_config(sc->dev, NHI_VS_CAP_22, vs22, 4);

	for (retries = 350; retries > 0; retries--) {
		vs9 = pci_read_config(sc->dev, NHI_VS_CAP_9, 4);
		if ((vs9 & NHI_VS_CAP_9_FW_READY) != 0) {
			device_printf(sc->dev,
			    "force-power: FW ready after %d ms (VS_CAP_9=0x%08x)\n",
			    (350 - retries) * 3, vs9);
			return (0);
		}
		DELAY(3000);
	}
	device_printf(sc->dev, "force-power: FW not ready (VS_CAP_9=0x%08x)\n",
	    vs9);
	return (ETIMEDOUT);
}

/* MSI/MSI-X interrupt handler: ack ring status + wake the RX event loop. */
static void
nhi_intr(void *arg)
{
	struct nhi_softc *sc = arg;

	nhi_ring0_intr(sc);
	wakeup(__DEVOLATILE(void *, &sc->event_run));	/* poke the RX event loop */
}

/* Sysctl trigger for the software replug: stash the level, poke the loop. */
static int
nhi_sysctl_reset(SYSCTL_HANDLER_ARGS)
{
	struct nhi_softc *sc = arg1;
	int val = 0, error;

	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val <= 0)
		return (0);
	sc->reset_req = arg2;	/* 1 = rescan, 2 = full reset */
	wakeup(__DEVOLATILE(void *, &sc->event_run));
	return (0);
}

/*
 * Debug probe: read our own router's HOPS config space (USB4 Spec 2.0 §8
 * cfg_read_pkg: route(8B) + tb_cfg_address{offset:13,length:6,port:6,
 * space:2,seq:2}) so we can SEE whether the ICM actually programmed the
 * XDomain path entries (hop table entry N = 2 dwords at offset N*2).
 * Responses (pdf=1 READ replies / pdf=3 errors) land in the event loop's
 * default case and get hexdumped.
 */
static void
nhi_dbg_hopdump(struct nhi_softc *sc)
{
	uint8_t req[12];
	uint32_t addr, port;

	static const u_int hops[] = { 1, 2, 8 };
	u_int i;

	for (port = 1; port <= 9; port++) {
		for (i = 0; i < 3; i++) {
			bzero(req, sizeof(req));  /* route 0 = our own router */
			addr = ((hops[i] * 2) & 0x1fff) |  /* entry = hop*2 dw */
			    (2u << 13) |		/* length: 2 dwords */
			    (port << 19) |		/* adapter/port */
			    (0u << 25) |		/* space 0 = HOPS */
			    (1u << 27);			/* seq */
			le32enc(req + 8, addr);
			nhi_ctl_tx(sc, 1 /* TB_CFG_PKG_READ */, req,
			    sizeof(req));
			DELAY(5000);
		}
	}
	device_printf(sc->dev, "hopdump: HOPS reads hops 1/2/8, ports 1-9\n");
}

/*
 * Experiment (dev.nhi.N.credits=1): bump the lane RX path entry's initial
 * credits (fw programs 14; effective E2E window toward us measures ~3
 * frames per 1ms tick = the 32 Mbit/s Mac->us ceiling).  Recoverable:
 * reset=1 makes the fw reprogram the entry.
 */
static void
nhi_dbg_credits(struct nhi_softc *sc)
{
	uint8_t req[20];
	uint32_t addr, dw0;

	bzero(req, sizeof(req));
	addr = (16 & 0x1fff) | (2u << 13) | (1u << 19) |  /* port1 HOPS[8] */
	    (0u << 25) | (1u << 27);
	le32enc(req + 8, addr);
	/* fw entry 0x801c3802: credits field (bits 24:17) 14 -> 64 */
	dw0 = (0x801c3802u & ~(0xffu << 17)) | (64u << 17);
	le32enc(req + 12, dw0);
	le32enc(req + 16, 0x0180c501);
	nhi_ctl_tx(sc, 2, req, sizeof(req));
	device_printf(sc->dev, "credits: lane HOPS[8] dw0=0x%08x (credits 64)\n",
	    dw0);
}

/*
 * Debug probe (dev.nhi.N.intrdump=1): snapshot the whole MSI/interrupt
 * routing block so we can SEE why the MSI fires only once.  Static analysis
 * says our device programming matches the reference implementation; the open question is the
 * delivery/routing layer.  Prime suspect: on MTL (MSI-X native) we fell back
 * to single MSI and never program REG_INT_VEC_ALLOC, so ring interrupts may
 * not be routed to our vector.  This reads (does NOT change) the state.
 */
static void
nhi_dbg_intrdump(struct nhi_softc *sc)
{
	uint32_t v0, v1, i;

	v0 = nhi_read(sc, NHI_REG_RING_INT_BASE);
	v1 = nhi_read(sc, NHI_REG_RING_INT_BASE + 4);
	device_printf(sc->dev, "intrdump: RING_INT (enable) +0=0x%08x +4=0x%08x\n",
	    v0, v1);

	v0 = nhi_read(sc, NHI_REG_RING_NOTIFY_BASE);
	v1 = nhi_read(sc, NHI_REG_RING_NOTIFY_BASE + 4);
	device_printf(sc->dev, "intrdump: NOTIFY (status) +0=0x%08x +4=0x%08x "
	    "(read clears)\n", v0, v1);

	/* IVR: 8 rings x 4 bits per 32-bit reg; hop_count=12 spans regs 0..2. */
	for (i = 0; i < NHI_INT_VEC_ALLOC_REGS; i++) {
		v0 = nhi_read(sc, NHI_REG_INT_VEC_ALLOC_BASE + i * 4);
		device_printf(sc->dev, "intrdump: IVR[%u] (0x%x) = 0x%08x\n",
		    i, NHI_REG_INT_VEC_ALLOC_BASE + i * 4, v0);
	}

	v0 = nhi_read(sc, NHI_REG_INT_THROTTLE);
	device_printf(sc->dev, "intrdump: THROTTLE[0] = 0x%08x\n", v0);
	v0 = nhi_read(sc, NHI_REG_DMA_MISC);
	device_printf(sc->dev, "intrdump: DMA_MISC = 0x%08x (auto_clear bit2=%u)\n",
	    v0, (v0 >> 2) & 1);
	device_printf(sc->dev, "intrdump: sc->intr_count=%u (vmstat should agree)\n",
	    sc->intr_count);
}

/*
 * Software "cable replug": re-arm the ICM without touching hardware cables.
 * level 1 (rescan) re-issues DRIVER_READY - a healthy ICM replays its topology
 * (DEVICE/XDOMAIN_CONNECTED) to the driver.  level 2 (reset) force-power
 * cycles the whole controller first (integrated parts only), recovering the
 * wedged-firmware state where events stop being raised entirely.  Runs in the
 * event-loop thread so it never races the ring consumers.
 */
static void
nhi_handle_reset(struct nhi_softc *sc, int level)
{
	if (level == 3) {		/* hopdump: read-only probe, no teardown */
		nhi_dbg_hopdump(sc);
		return;
	}
	if (level == 4) {		/* portdump: adapter types (PORT space) */
		uint8_t req[12];
		uint32_t addr, port;

		for (port = 1; port <= 9; port++) {
			bzero(req, sizeof(req));
			addr = (2 & 0x1fff) |	/* dword 2 = adapter TYPE */
			    (1u << 13) | (port << 19) |
			    (1u << 25) |	/* space 1 = TB_CFG_PORT */
			    (1u << 27);
			le32enc(req + 8, addr);
			nhi_ctl_tx(sc, 1, req, sizeof(req));
			DELAY(5000);
		}
		device_printf(sc->dev, "portdump: PORT space dw2 (type), ports 1-9\n");
		return;
	}
	if (level == 6) {
		nhi_dbg_credits(sc);
		return;
	}
	if (level == 7) {		/* intrdump: MSI/IVR routing snapshot */
		nhi_dbg_intrdump(sc);
		return;
	}
	if (level == 5) {
		/*
		 * pathfix: manually install the TX hop entry the ICM appears
		 * to skip - NHI adapter (port 7) HOPS[ring 1] -> out_port 1
		 * (lane), next_hop 8, styled after the fw-programmed RX entry
		 * (port1 HOPS[8] = 0x801c3802 / 0x0180c501).  CFG_WRITE pkg =
		 * route(8) + addr(4) + data dwords.
		 */
		uint8_t req[20];
		uint32_t addr;

		bzero(req, sizeof(req));
		addr = (2 & 0x1fff) |		/* HOPS entry 1 (offset 1*2) */
		    (2u << 13) | (7u << 19) |	/* len 2, port 7 (NHI) */
		    (0u << 25) | (1u << 27);	/* space HOPS, seq */
		le32enc(req + 8, addr);
		/* dw0: next_hop=8 | out_port=1<<11 | credits=14<<17 | enable */
		le32enc(req + 12, 8 | (1u << 11) | (14u << 17) | (1u << 31));
		/* dw1: weight/priority/counters like the fw RX entry */
		le32enc(req + 16, 0x0180c501);
		nhi_ctl_tx(sc, 2 /* TB_CFG_PKG_WRITE */, req, sizeof(req));
		device_printf(sc->dev, "pathfix: wrote NHI HOPS[1] -> port1 hop8\n");
		return;
	}
	device_printf(sc->dev, "%s requested: tearing down session\n",
	    level >= 2 ? "controller reset" : "rescan");
	/* Fabric-level teardown FIRST so the peer sees the XDomain drop and
	 * restarts its own discovery + login (else only our side is fresh). */
	nhi_icm_disconnect_xdomain(sc);
	nhi_tbt_disconnect(sc);
	sc->has_peer = false;
	if (level >= 2) {
		if (!sc->force_power) {
			device_printf(sc->dev,
			    "reset: no force-power control on this part; "
			    "doing rescan instead\n");
		} else {
			nhi_ring0_teardown(sc);
			nhi_force_power(sc, false);
			pause("nhirst", hz / 2);
			if (nhi_force_power(sc, true) != 0 ||
			    nhi_ring0_setup(sc) != 0) {
				device_printf(sc->dev,
				    "reset: controller did not come back\n");
				return;
			}
		}
	}
	nhi_icm_driver_ready(sc);
}

/*
 * Ring-0 RX event loop: continuously reap control frames and dispatch by PDF -
 * ICM notifications (XDOMAIN_CONNECTED, ...) and XDomain discovery requests
 * from a connected peer.  Polls non-blocking and sleeps between idle passes.
 */
static void
nhi_event_loop(void *arg)
{
	struct nhi_softc *sc = arg;
	uint8_t frame[NHI_CTL_FRAME_SIZE];
	u_int len;
	uint8_t pdf;

	while (sc->event_run) {
		if (sc->reset_req != 0) {
			nhi_handle_reset(sc, sc->reset_req);
			sc->reset_req = 0;
		}
		len = sizeof(frame);
		if (nhi_ctl_rx(sc, frame, &len, &pdf, 0) == 0) {
			switch (pdf) {
			case NHI_PDF_ICM_EVENT:
				nhi_icm_handle_event(sc, frame, len);
				break;
			case NHI_PDF_XDOMAIN_REQ:
			case NHI_PDF_XDOMAIN_RESP:
				/*
				 * macOS AppleThunderboltIP sends its LOGIN (and the
				 * LOGIN_RESPONSE) as XDOMAIN_RESP (pdf=7), not REQ, so
				 * route both PDFs through the XDomain dispatcher - it
				 * hands service-UUID frames to the tbip login handler.
				 */
				nhi_xdomain_handle(sc, frame, len);
				break;
			default:
				device_printf(sc->dev,
				    "ring0 rx pdf=%u len=%u:\n", pdf, len);
				hexdump(frame, len, "  rx ", 0);
				break;
			}
			continue;	/* drain quickly while frames remain */
		}
		if (sc->data_up)
			nhi_tbt_rx_poll(sc);	/* drain the network RX ring */
		nhi_tbip_start_login(sc);	/* active P2P: (re)send our LOGIN */
		/*
		 * Everything drained: unmask ring interrupts (the ISR masked
		 * them all).  If new work raced in, the still-asserted level
		 * yields a fresh MSI edge on unmask - nothing lost.
		 */
		nhi_intr_unmask(sc);
		/*
		 * MSI doesn't fire on 16.0-CURRENT (intr_count stays 0), so RX
		 * is polling-driven and the poll rate IS the throughput
		 * ceiling (the peer's E2E window refills once per drain).
		 * While the data path is up, busy-poll at ~20 kHz between
		 * ticks; otherwise idle at 20 Hz.  TODO: fix interrupts.
		 */
		if (sc->data_up) {
			int spin;

			for (spin = 0; spin < 20 && sc->event_run; spin++) {
				DELAY(50);
				nhi_tbt_rx_poll(sc);
			}
		} else
			tsleep(__DEVOLATILE(void *, &sc->event_run), 0,
			    "nhiev", hz / 20);
	}
	sc->event_td = NULL;
	wakeup(sc);
	kthread_exit();
}

/* Allocate the IRQ resource for the first MSI/MSI-X vector and hook it. */
static int
nhi_setup_handler(struct nhi_softc *sc)
{
	int error;

	if (sc->msix_count <= 0)
		return (ENXIO);

	sc->irq_rid = 1;	/* first MSI/MSI-X vector is rid 1 */
	sc->irq = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE);
	if (sc->irq == NULL) {
		device_printf(sc->dev, "cannot allocate IRQ resource\n");
		return (ENXIO);
	}
	error = bus_setup_intr(sc->dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, nhi_intr, sc, &sc->intrhand);
	if (error != 0)
		device_printf(sc->dev, "bus_setup_intr failed: %d\n", error);
	return (error);
}

static int
nhi_attach(device_t dev)
{
	struct nhi_softc *sc = device_get_softc(dev);
	const struct nhi_ident *id;
	int error;

	sc->dev = dev;
	id = nhi_lookup(dev);
	sc->force_power = (id != NULL && (id->flags & NHI_FLAG_FORCE_POWER) != 0);
	pci_enable_busmaster(dev);

	error = nhi_map_regs(sc);
	if (error != 0)
		return (error);

	device_printf(dev, "BAR0 mapped at %#jx (%ju bytes)\n",
	    (uintmax_t)rman_get_start(sc->regs),
	    (uintmax_t)rman_get_size(sc->regs));

	nhi_read_caps(sc);

	/*
	 * Integrated controllers: assert force-power so the on-die CM is awake
	 * before we drive ring 0, then re-read the mode (TP-1b / the ICM gate).
	 */
	if (sc->force_power) {
		nhi_force_power(sc, true);
		sc->fw_sts = nhi_read(sc, NHI_REG_FW_STS);
		device_printf(dev,
		    "post-force-power: fw_sts=0x%08x fw_mode=%s(0x%x)\n",
		    sc->fw_sts, nhi_fw_mode_str(nhi_mailbox_mode(sc)),
		    nhi_mailbox_mode(sc));
	}

	nhi_setup_intr(sc);

	if (nhi_setup_handler(sc) != 0)
		device_printf(dev, "continuing without an interrupt handler\n");

	error = nhi_ring0_setup(sc);
	if (error != 0) {
		device_printf(dev, "ring 0 setup failed: %d\n", error);
		return (0);	/* stay attached for inspection */
	}

	/*
	 * Issue DRIVER_READY on ring 0 and parse the ICM response: this is the
	 * mbox_ready / TP-1b probe that resolves whether the (dormant, safe-mode)
	 * ICM on this controller wakes into CM mode (DESIGN.md V1.3 / V5.1).
	 */
	nhi_icm_driver_ready(sc);

	/* Build our XDomain property directory and start the RX event loop. */
	nhi_xdomain_build_dir(sc);
	sc->event_run = 1;
	if (kthread_add(nhi_event_loop, sc, NULL, &sc->event_td, 0, 0,
	    "nhi%d-event", device_get_unit(dev)) != 0) {
		sc->event_run = 0;
		sc->event_td = NULL;
		device_printf(dev, "failed to start ring-0 event loop\n");
	}

	/* Software "replug": dev.nhi.N.rescan / dev.nhi.N.reset (see
	 * nhi_handle_reset).  Write 1 to trigger; runs in the event loop. */
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "rescan",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 1,
	    nhi_sysctl_reset, "I", "re-issue DRIVER_READY (ICM replays topology)");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "reset",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 2,
	    nhi_sysctl_reset, "I", "force-power cycle the controller + re-init");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "hopdump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 3,
	    nhi_sysctl_reset, "I", "debug: dump own router hop-8 path entries");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "portdump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 4,
	    nhi_sysctl_reset, "I", "debug: dump adapter types (PORT space)");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "pathfix",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 5,
	    nhi_sysctl_reset, "I", "debug: manually install NHI TX hop entry");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "credits",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 6,
	    nhi_sysctl_reset, "I", "debug: bump lane RX path credits 14 -> 64");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "intrdump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 7,
	    nhi_sysctl_reset, "I", "debug: snapshot MSI/IVR interrupt routing");

	return (0);
}

static int
nhi_detach(device_t dev)
{
	struct nhi_softc *sc = device_get_softc(dev);

	/* Stop the event loop before tearing down the rings it polls. */
	if (sc->event_td != NULL) {
		sc->event_run = 0;
		wakeup(__DEVOLATILE(void *, &sc->event_run));
		while (sc->event_td != NULL)
			tsleep(sc, 0, "nhiquit", hz / 10);
	}

	/* Detach tbt0 + free the data rings (Phase 3b); loop is stopped so no
	 * more if_input()/RX poll races with this. */
	if (sc->ifp != NULL) {
		ether_ifdetach(sc->ifp);
		if_free(sc->ifp);
		sc->ifp = NULL;
		if (sc->rx_m != NULL) {
			m_freem(sc->rx_m);
			sc->rx_m = NULL;
		}
		mtx_destroy(&sc->tbt_lock);
	}
	nhi_data_teardown(sc);

	nhi_ring0_teardown(sc);		/* disables ring IRQs + frees rings */

	if (sc->force_power)
		nhi_force_power(sc, false);

	if (sc->intrhand != NULL)
		bus_teardown_intr(dev, sc->irq, sc->intrhand);
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
	if (sc->msix_count > 0)
		pci_release_msi(dev);
	if (sc->regs != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->regs_rid,
		    sc->regs);
	return (0);
}

static device_method_t nhi_methods[] = {
	DEVMETHOD(device_probe,		nhi_probe),
	DEVMETHOD(device_attach,	nhi_attach),
	DEVMETHOD(device_detach,	nhi_detach),
	DEVMETHOD_END
};

static driver_t nhi_driver = {
	"nhi_icm",
	nhi_methods,
	sizeof(struct nhi_softc),
};

DRIVER_MODULE(nhi_icm, pci, nhi_driver, NULL, NULL);
MODULE_DEPEND(nhi_icm, pci, 1, 1, 1);
MODULE_VERSION(nhi_icm, 1);

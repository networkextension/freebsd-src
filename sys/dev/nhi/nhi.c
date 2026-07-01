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
 * vendor-specific PCI config registers (Linux nhi.c icl_nhi_force_power).  On
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
		tsleep(__DEVOLATILE(void *, &sc->event_run), 0, "nhiev", hz / 20);
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

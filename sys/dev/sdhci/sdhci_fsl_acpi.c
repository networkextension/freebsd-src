/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * ACPI front-end for the NXP QorIQ Layerscape eSDHC controller (_HID NXP0003).
 *
 * Under UEFI/ACPI the eSDHC is described in the DSDT (MMIO + IRQ in _CRS, base
 * clock and bus properties in _DSD) but there is no FDT clock framework. This
 * binding reads the firmware-provided properties and hands off to the shared
 * sdhci_fsl_attach_common(), letting the OS access the SD/TF card (e.g. to
 * rewrite the boot firmware) without booting under u-boot/FDT.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/mmc/mmc_helpers.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>
#include <dev/acpica/acpivar.h>

#include <dev/sdhci/sdhci.h>
#include <dev/sdhci/sdhci_fsl.h>

#include "mmcbr_if.h"
#include "sdhci_if.h"

#include "opt_mmccam.h"

static char *sdhci_fsl_acpi_hids[] = {
	"NXP0003",
	NULL
};

static int
sdhci_fsl_acpi_probe(device_t dev)
{
	int err;

	err = ACPI_ID_PROBE(device_get_parent(dev), dev, sdhci_fsl_acpi_hids,
	    NULL);
	if (err <= 0) {
		device_set_desc(dev,
		    "NXP QorIQ Layerscape eSDHC controller");
		return (err);
	}

	return (ENXIO);
}

static int
sdhci_fsl_acpi_attach(device_t dev)
{
	struct sdhci_fsl_fdt_softc *sc;
	struct mmc_helper mmc_helper;
	uint32_t clk_hz;

	sc = device_get_softc(dev);

	/*
	 * NXP0003 identifies the LX2160A-class eSDHC; use its SoC data for the
	 * quirks/errata and the base-clock divider.
	 */
	sc->soc_data = &sdhci_fsl_fdt_lx2160a_soc_data;
	sc->acpi = true;
	sc->little_endian = true;	/* _DSD "little-endian"; LX2160A eSDHC */

	/* Base (peripheral) clock from the _DSD "clock-frequency" property. */
	clk_hz = 0;
	if (device_get_property(dev, "clock-frequency", &clk_hz, sizeof(clk_hz),
	    DEVICE_PROP_UINT32) <= 0 || clk_hz == 0) {
		device_printf(dev, "missing 'clock-frequency' property\n");
		return (ENXIO);
	}
	sc->baseclk_hz = clk_hz / sc->soc_data->baseclk_div;

	/* mmc host properties (bus-width, max-frequency, ...) from _DSD. */
	memset(&mmc_helper, 0, sizeof(mmc_helper));
	if (mmc_parse(dev, &mmc_helper, &sc->slot.host) != 0)
		return (ENXIO);

	/*
	 * Cap to high speed (25 MHz). UHS modes (SDR50/SDR104) require a tuning
	 * sequence with voltage switching that is not yet wired up on the ACPI
	 * front-end, so restrict to a non-tuned speed for reliable enumeration.
	 */
	if (sc->slot.host.f_max > 25000000)
		sc->slot.host.f_max = 25000000;

	return (sdhci_fsl_attach_common(dev));
}

static const device_method_t sdhci_fsl_acpi_methods[] = {
	DEVMETHOD(device_probe,			sdhci_fsl_acpi_probe),
	DEVMETHOD(device_attach,		sdhci_fsl_acpi_attach),
	DEVMETHOD(device_detach,		sdhci_fsl_detach),

	/*
	 * No FDT GPIO line under ACPI; use the controller's own card-detect
	 * (present-state register) instead.
	 */
	DEVMETHOD(sdhci_get_card_present,	sdhci_generic_get_card_present),

	DEVMETHOD_END
};

DEFINE_CLASS_1(sdhci_fsl, sdhci_fsl_acpi_driver, sdhci_fsl_acpi_methods,
    sizeof(struct sdhci_fsl_fdt_softc), sdhci_fsl_driver);

DRIVER_MODULE(sdhci_fsl_acpi, acpi, sdhci_fsl_acpi_driver, NULL, NULL);
SDHCI_DEPEND(sdhci_fsl_acpi);

#ifndef MMCCAM
MMC_DECLARE_BRIDGE(sdhci_fsl_acpi);
#endif

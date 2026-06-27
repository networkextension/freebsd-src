/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Shared definitions for the NXP QorIQ Layerscape eSDHC controller, used by
 * both the FDT (sdhci_fsl_fdt.c) and ACPI (sdhci_fsl_acpi.c) bus front-ends.
 */

#ifndef _SDHCI_FSL_H_
#define _SDHCI_FSL_H_

#include <dev/mmc/mmc_helpers.h>
#include <dev/sdhci/sdhci.h>

struct sdhci_fdt_gpio;

struct sdhci_fsl_fdt_soc_data {
	int quirks;
	int baseclk_div;
	uint8_t errata;
	char *syscon_compat;
};

extern const struct sdhci_fsl_fdt_soc_data sdhci_fsl_fdt_lx2160a_soc_data;

struct sdhci_fsl_fdt_softc {
	device_t				dev;
	const struct sdhci_fsl_fdt_soc_data	*soc_data;
	struct resource				*mem_res;
	struct resource				*irq_res;
	void					*irq_cookie;
	uint32_t				baseclk_hz;
	uint32_t				maxclk_hz;
	struct sdhci_fdt_gpio			*gpio;
	struct sdhci_slot			slot;
	bool					slot_init_done;
	uint32_t				cmd_and_mode;
	uint16_t				sdclk_bits;
	struct mmc_helper			fdt_helper;
	uint32_t				div_ratio;
	uint8_t					vendor_ver;
	uint32_t				flags;

	/* Set by the bus front-end before calling sdhci_fsl_attach_common(). */
	bool					little_endian;
	bool					acpi;	/* ACPI front-end (no FDT node) */

	uint32_t (* read)(struct sdhci_fsl_fdt_softc *, bus_size_t);
	void (* write)(struct sdhci_fsl_fdt_softc *, bus_size_t, uint32_t);
};

DECLARE_CLASS(sdhci_fsl_driver);

/*
 * Bus-agnostic attach: allocate resources, configure the eSDHC block and bring
 * up the slot. The caller must first fill soc_data, baseclk_hz, little_endian,
 * acpi and the parsed mmc host properties (slot.host) in the softc.
 */
int	sdhci_fsl_attach_common(device_t dev);
int	sdhci_fsl_detach(device_t dev);

#endif /* _SDHCI_FSL_H_ */

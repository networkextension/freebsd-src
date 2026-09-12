/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Xiangbo Kong <yarshure@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Small Form Factor (SFF) Committee Pluggable (SFP) Transceiver (ACPI-based).
 *
 * The counterpart of sfp_fdt(4).  Firmware describes the cage the same way a
 * device tree does -- a node with compatible "sff,sfp" -- but places it where
 * ACPI places an i2c device: in the scope of the bus its EEPROM answers on,
 * with an I2cSerialBus resource giving the base page's 7-bit address, 0x50.
 * So where the FDT front-end has to follow an "i2c-bus" phandle to find its
 * bus, this one simply has it as a parent, and it can own the bus request
 * itself rather than borrowing another device on the bus.
 *
 * Everything above this -- DPAA2_MC_GET_SFF_DEV(), SFF_READ_EEPROM() and the
 * SIOCGI2C handler in the NIC driver -- is shared with the FDT path and knows
 * nothing about which of the two described the transceiver.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>
#include <sys/systm.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include "sff.h"
#include "sff_if.h"

#define	SFP_ACPI_COMPAT		"sff,sfp"

static int
sfp_acpi_probe(device_t dev)
{
	char compat[64];
	ssize_t s;

	if (acpi_disabled("sff"))
		return (ENXIO);
	if (acpi_get_handle(dev) == NULL)
		return (ENXIO);

	/*
	 * Match the same compatible string the device-tree binding uses; ACPI
	 * carries it in _DSD, with _HID "PRP0001".
	 */
	memset(compat, 0, sizeof(compat));
	s = device_get_property(dev, "compatible", compat, sizeof(compat) - 1,
	    DEVICE_PROP_ANY);
	if (s <= 0 || strcmp(compat, SFP_ACPI_COMPAT) != 0)
		return (ENXIO);

	device_set_desc(dev, "Small Form-factor Pluggable Transceiver");
	return (BUS_PROBE_DEFAULT);
}

static int
sfp_acpi_attach(device_t dev)
{

	/*
	 * Nothing to latch.  acpi_iicbus(4) already attached us to our
	 * namespace node, so acpi_get_device() on the handle a DPMAC's "sfp"
	 * property points at yields this device -- the ACPI equivalent of the
	 * xref registration sfp_fdt(4) has to do here.
	 */
	return (0);
}

static int
sfp_acpi_get_i2c_bus(device_t dev, device_t *i2c_bus)
{

	KASSERT(i2c_bus != NULL, ("%s: i2c_bus is NULL", __func__));

	*i2c_bus = device_get_parent(dev);
	return (0);
}

static int
sfp_acpi_read_eeprom(device_t dev, uint8_t dev_addr, uint8_t offset,
    uint8_t *buf, int len)
{

	/* We sit on the bus the EEPROM answers on, so we own the request. */
	return (sff_read_eeprom(dev, dev_addr, offset, buf, len));
}

static device_method_t sfp_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		sfp_acpi_probe),
	DEVMETHOD(device_attach,	sfp_acpi_attach),

	/* SFF */
	DEVMETHOD(sff_get_i2c_bus,	sfp_acpi_get_i2c_bus),
	DEVMETHOD(sff_read_eeprom,	sfp_acpi_read_eeprom),

	DEVMETHOD_END
};

DEFINE_CLASS_0(sfp_acpi, sfp_acpi_driver, sfp_acpi_methods, 0);

DRIVER_MODULE(sfp_acpi, iicbus, sfp_acpi_driver, 0, 0);
MODULE_VERSION(sfp_acpi, 1);
MODULE_DEPEND(sfp_acpi, sff, 1, 1, 1);
MODULE_DEPEND(sfp_acpi, acpi, 1, 1, 1);
MODULE_DEPEND(sfp_acpi, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);

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
 * Small Form Factor (SFF) Committee Pluggable (SFP) Transceiver: shared i2c
 * helpers used by the bus-specific front-ends (sfp_fdt, ...) and by NIC drivers
 * that must drive an i2c mux themselves.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include <dev/iicbus/iic.h>
#include <dev/iicbus/iiconf.h>

#include "sff.h"

int
sff_read_eeprom(device_t requester, uint8_t dev_addr, uint8_t offset,
    uint8_t *buf, int len)
{
	struct iic_msg msgs[2];

	if (requester == NULL)
		return (ENXIO);
	if (buf == NULL || len <= 0 || len > UINT16_MAX)
		return (EINVAL);

	/*
	 * Write the byte offset, then read the data back without releasing
	 * the bus in between (a repeat-start), so nothing else can move the
	 * EEPROM's internal address pointer between the two halves.  Holding
	 * the bus for the whole exchange also keeps an i2c mux upstream
	 * pointed at this device throughout -- iicbus(4) switches the mux as
	 * part of granting the bus, and would switch it away again for
	 * another consumer if we let go.
	 */
	msgs[0].slave = dev_addr;
	msgs[0].flags = IIC_M_WR | IIC_M_NOSTOP;
	msgs[0].len = 1;
	msgs[0].buf = &offset;
	msgs[1].slave = dev_addr;
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	return (iic2errno(iicbus_transfer_excl(requester, msgs, nitems(msgs),
	    IIC_INTRWAIT)));
}

device_t
sff_i2c_requester(device_t i2c_bus)
{

	return (device_find_child(i2c_bus, "iic", DEVICE_UNIT_ANY));
}

static int
sff_modevent(module_t mod __unused, int type, void *data __unused)
{
	switch (type) {
	case MOD_LOAD:
	case MOD_UNLOAD:
	case MOD_QUIESCE:
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t sff_moduledata = { "sff", sff_modevent, NULL };
DECLARE_MODULE(sff, sff_moduledata, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(sff, 1);
MODULE_DEPEND(sff, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);

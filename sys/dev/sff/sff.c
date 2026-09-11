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
sff_read_eeprom(device_t requester, int muxaddr, uint8_t chsel,
    uint8_t chrestore, uint8_t dev_addr, uint8_t offset, uint8_t *buf, int len)
{
	device_t bus = device_get_parent(requester);
	struct iic_msg sel, rd[2];
	uint8_t off = offset;
	int error;

	/*
	 * Hold the bus across the (optional) mux channel select, the offset
	 * write and the data read, so no other consumer can switch the mux or
	 * see it pointed at this device mid-transaction.
	 */
	error = iicbus_request_bus(bus, requester, IIC_INTRWAIT);
	if (error != 0)
		return (iic2errno(error));

	if (muxaddr != 0) {
		sel.slave = (uint16_t)muxaddr << 1;
		sel.flags = IIC_M_WR;
		sel.len = 1;
		sel.buf = &chsel;
		error = iicbus_transfer(requester, &sel, 1);
	}

	if (error == 0) {
		/* Write the byte offset (repeat-start), then read the data. */
		rd[0].slave = dev_addr;		/* already 8-bit (0xA0/0xA2) */
		rd[0].flags = IIC_M_WR | IIC_M_NOSTOP;
		rd[0].len = 1;
		rd[0].buf = &off;
		rd[1].slave = dev_addr;
		rd[1].flags = IIC_M_RD;
		rd[1].len = len;
		rd[1].buf = buf;
		error = iicbus_transfer(requester, rd, 2);
	}

	if (muxaddr != 0) {
		sel.slave = (uint16_t)muxaddr << 1;
		sel.flags = IIC_M_WR;
		sel.len = 1;
		sel.buf = &chrestore;
		(void)iicbus_transfer(requester, &sel, 1);
	}

	iicbus_release_bus(bus, requester);
	return (error != 0 ? iic2errno(error) : 0);
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

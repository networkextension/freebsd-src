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

#ifndef _DEV_SFF_SFF_H_
#define _DEV_SFF_SFF_H_

/*
 * Read from an SFP module's EEPROM over i2c.
 *
 * dev_addr is the slave address in the form iic_msg(9) uses -- the 7-bit
 * address shifted left by one -- which for an SFF-8472 module is 0xa0 for the
 * base page and 0xa2 for the diagnostics page.  That is also the form
 * SIOCGI2C's struct ifi2creq carries, so a NIC driver passes what it was
 * given.
 *
 * requester is a device on the iicbus the EEPROM answers on; it owns the bus
 * for the duration of the exchange.  A front-end that is itself an i2c slave
 * (the ACPI one) passes itself; one that only holds a reference to the bus
 * (the FDT one) has to borrow a device on it -- see sff_i2c_requester().
 */
int	sff_read_eeprom(device_t requester, uint8_t dev_addr, uint8_t offset,
	    uint8_t *buf, int len);

/*
 * A bus request needs an owner that sits on the bus being requested.  A
 * front-end that has only the iicbus itself borrows the iic(4) child every
 * iicbus carries.  Returns NULL if there is none, which sff_read_eeprom()
 * reports as ENXIO.
 */
device_t sff_i2c_requester(device_t i2c_bus);

#endif /* _DEV_SFF_SFF_H_ */

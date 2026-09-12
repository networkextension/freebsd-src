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
 * Read from an SFP module's EEPROM page over i2c.
 *
 * dev_addr is the 8-bit (left-aligned) page address, i.e. 0xA0 for the base
 * SFF-8472 page and 0xA2 for the diagnostics page.  The bus is held for the
 * whole exchange (offset write with a repeat-start, then read).
 *
 * When muxaddr != 0 an i2c-mux channel is selected before the read (by writing
 * chsel to the 7-bit mux address muxaddr) and restored afterwards (chrestore).
 * This is for platforms whose i2c mux has no driver (e.g. some ACPI systems);
 * pass muxaddr == 0 when the mux is switched transparently by the i2c
 * framework, which is the normal FDT case.
 *
 * requester must be a device sitting on the target iicbus (e.g. its iic(4)
 * child); it is used as the bus-request owner.
 */
int	sff_read_eeprom(device_t requester, int muxaddr, uint8_t chsel,
	    uint8_t chrestore, uint8_t dev_addr, uint8_t offset, uint8_t *buf,
	    int len);

#endif /* _DEV_SFF_SFF_H_ */

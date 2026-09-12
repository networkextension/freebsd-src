/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Xiangbo Kong <yarshure@gmail.com>
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
/*
 * Find the iic(4) child of an iicbus, to be used as the bus-request owner for
 * sff_read_eeprom().  Returns NULL if the bus has no such child.
 */
device_t sff_i2c_requester(device_t i2c_bus);

int	sff_read_eeprom(device_t requester, int muxaddr, uint8_t chsel,
	    uint8_t chrestore, uint8_t dev_addr, uint8_t offset, uint8_t *buf,
	    int len);

#endif /* _DEV_SFF_SFF_H_ */

/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * nhi_mailbox.c - NHI firmware mailbox primitive.
 *
 * The mailbox is the synchronous register path to the on-chip ICM (Internal
 * Connection Manager): host writes a command into INMAIL and the firmware
 * acknowledges by clearing the request bit (or setting ERROR).  OUTMAIL
 * reports the firmware operation mode, which is how we confirm the ICM is
 * actually up before driving XDomain traffic (DESIGN.md decision D1, the
 * mbox_ready verification V1.3/V5.1).
 *
 * Higher-level ICM requests (DRIVER_READY, APPROVE_XDOMAIN_PATHS, ...) are
 * control-channel packets on ring 0 and land with the ring engine; this file
 * is just the raw mailbox transport plus the mode query.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/pci/pcivar.h>

#include "nhi_reg.h"
#include "nhi_var.h"

/* Firmware should ack a mailbox command well within this window. */
#define	NHI_MAILBOX_TIMEOUT_US	500000		/* 500 ms */
#define	NHI_MAILBOX_POLL_US	100

/*
 * Issue a mailbox command and wait for the firmware to consume it.
 * Returns 0 on success, ETIMEDOUT if the request bit never clears, or EIO if
 * the firmware flags an error.
 */
int
nhi_mailbox_cmd(struct nhi_softc *sc, uint8_t cmd, uint32_t data)
{
	uint32_t val;
	int i;

	nhi_write(sc, NHI_REG_INMAIL_DATA, data);
	nhi_write(sc, NHI_REG_INMAIL_CMD,
	    ((uint32_t)cmd & NHI_INMAIL_CMD_MASK) | NHI_INMAIL_CMD_OP_REQUEST);

	for (i = 0; i < NHI_MAILBOX_TIMEOUT_US / NHI_MAILBOX_POLL_US; i++) {
		val = nhi_read(sc, NHI_REG_INMAIL_CMD);
		if ((val & NHI_INMAIL_CMD_OP_REQUEST) == 0)
			goto done;
		DELAY(NHI_MAILBOX_POLL_US);
	}
	device_printf(sc->dev, "mailbox cmd 0x%02x timed out\n", cmd);
	return (ETIMEDOUT);

done:
	if (val & NHI_INMAIL_CMD_ERROR) {
		device_printf(sc->dev, "mailbox cmd 0x%02x rejected\n", cmd);
		return (EIO);
	}
	return (0);
}

/* Read the firmware operation mode from OUTMAIL (NHI_OPMODE_*). */
int
nhi_mailbox_mode(struct nhi_softc *sc)
{
	uint32_t val;

	val = nhi_read(sc, NHI_REG_OUTMAIL_CMD);
	return ((val & NHI_OUTMAIL_CMD_OPMODE_MASK) >>
	    NHI_OUTMAIL_CMD_OPMODE_SHIFT);
}

const char *
nhi_fw_mode_str(int mode)
{
	switch (mode) {
	case NHI_OPMODE_SAFE:	return ("safe");
	case NHI_OPMODE_AUTH:	return ("nvm-auth");
	case NHI_OPMODE_EP:	return ("endpoint");
	case NHI_OPMODE_CM:	return ("cm/icm");
	default:		return ("unknown");
	}
}

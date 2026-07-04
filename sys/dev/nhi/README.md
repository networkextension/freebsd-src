# nhi(4) / nhi_icm — ICM-mode Thunderbolt/USB4 host stack

A native Thunderbolt 3/4 (USB4) host driver that talks to the controller's
firmware **Internal Connection Manager (ICM)** and brings up **Apple
ThunderboltIP** host-to-host networking, so a Mac's *Thunderbolt Bridge* (and
Windows and other USB4NET hosts) can reach the FreeBSD box over a Thunderbolt cable.

This is distinct from the in-tree HCM driver in `sys/dev/thunderbolt/` (module
`tb`), which does connection management in the OS. This one leans on the
firmware ICM and focuses on the XDomain + ThunderboltIP networking path. The
module is named **`nhi_icm`** (`sys/modules/nhi_icm`); both drivers register an
NHI PCI driver for the same controllers, so **load only one at a time**.

## Layers (all in this directory, one module)

- `nhi.c`, `nhi_ring.c`, `nhi_mailbox.c` — NHI transport: BAR0 regs, TX/RX ring
  engine (bus_dma coherent descriptors + doorbells), MSI(-X), firmware mailbox.
- `nhi_icm.c` — ICM client: `DRIVER_READY`, event handling, `APPROVE_XDOMAIN`.
- `nhi_xdomain.c` — XDomain discovery: UUID exchange, property directory
  (advertises the `network` service).
- `nhi_tbip.c` — Apple ThunderboltIP login handshake over ring 0.
- `if_tbt.c` — `tbt0` Ethernet ifnet: data rings + fragment/reassemble.

## Phase-1 status (verified live vs macOS)

On a ThinkPad T14s (Meteor Lake, NHI `0x8086:0x7ec2`/`0x7ec3`):
- **XDomain discovery works** — macOS System Profiler shows the FreeBSD box as a
  full device with the *Internet Protocol* (ThunderboltIP) service.
- **Bidirectional ThunderboltIP LOGIN completes** — data rings allocated.
- Remaining: `APPROVE_XDOMAIN` → `tbt0` end-to-end (see the design repo).

## Design history

The full design (the deliverable — hardware/protocol facts each with a `VERIFY:`
probe) and the ThinkPad test plan are alongside the sources here:
- `DESIGN.md` — architecture, ICM/XDomain/ThunderboltIP protocol details, probes.
- `TESTPLAN.md` — the T14s bring-up + verification steps.

License: BSD-2-Clause (see each source file's header).

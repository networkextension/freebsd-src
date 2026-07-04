# Building `thunderbolt.ko`

This branch is the in-tree FreeBSD USB4/Thunderbolt driver (`sys/dev/thunderbolt/`,
HCM transport) plus our additions — the ICM connection manager (`tb_icm`),
XDomain shared layer (`tb_xdomain`), and the ThunderboltIP net driver
(`if_tbt`) — with a module Makefile that builds them together.

## Standalone (against a FreeBSD 16-CURRENT source tree)

    make -C sys/modules/thunderbolt SYSDIR=/path/to/freebsd/sys

Produces `sys/modules/thunderbolt/thunderbolt.ko`. `SYSDIR` supplies the build
infrastructure (`bsd.kmod.mk`) and system headers; this branch's own
`sys/dev/thunderbolt/` headers take precedence (via `-I` in the Makefile), so the
modified `nhi_reg.h` / `nhi_var.h` / etc. are the ones compiled.

## In a full FreeBSD checkout

Overlay this branch's `sys/dev/thunderbolt/` and `sys/modules/thunderbolt/` onto a
16-CURRENT tree, then `cd sys/modules/thunderbolt && make`.

## Load

    kldload ./thunderbolt.ko

Integrated controllers (Meteor Lake) are force-powered and driven in ICM mode
automatically; see `sys/dev/thunderbolt/tb_icm.c`. The `if_tbt` interface
(`tbtN`) is created at load. `scripts/tb-bridge.sh` brings it up as a DHCP
network for a Thunderbolt-connected peer.

Verified: builds clean on 16.0-CURRENT (`main-cf851111`).

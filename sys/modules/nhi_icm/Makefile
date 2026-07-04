# nhi_icm(4) - Thunderbolt/USB4 ICM-mode NHI driver.
#
# Research import of an out-of-tree ICM-mode Thunderbolt stack (XDomain
# discovery + Apple ThunderboltIP networking), distinct from the in-tree
# HCM driver in sys/dev/thunderbolt (module "tb").  Sources live in
# sys/dev/nhi.  See sys/dev/nhi/README.md.
#
# NOTE: both this and the base "tb" driver register an NHI PCI driver for the
# same controllers - load only one at a time.

.PATH: ${SRCTOP}/sys/dev/nhi

KMOD=	nhi_icm
SRCS=	nhi.c nhi_mailbox.c nhi_ring.c nhi_icm.c nhi_xdomain.c nhi_tbip.c
SRCS+=	if_tbt.c
SRCS+=	device_if.h bus_if.h pci_if.h

.include <bsd.kmod.mk>

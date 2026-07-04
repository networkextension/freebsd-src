# Builds thunderbolt.ko: the in-tree USB4/Thunderbolt transport (HCM) plus the
# ICM connection manager, XDomain, and the ThunderboltIP (tbt) net driver.
#
# Standalone build against a FreeBSD 16-CURRENT source tree:
#     make -C sys/modules/thunderbolt SYSDIR=/path/to/freebsd/sys
# (-I on the tree root below makes this branch's modified dev/thunderbolt/
#  headers win over the ones in SYSDIR.)

.PATH:		${.CURDIR}/../../dev/thunderbolt
CFLAGS+=	-I${.CURDIR}/../..

KMOD=	thunderbolt
# in-tree transport + HCM (FreeBSD base)
SRCS=	nhi_pci.c nhi.c tb_pcib.c tb_acpi_pcib.c tb_debug.c nhi_wmi.c
SRCS+=	router.c hcm.c tb_dev.c
# our additions: ICM connection manager, XDomain, ThunderboltIP net driver
SRCS+=	tb_icm.c tb_xdomain.c if_tbt.c
SRCS+=	opt_thunderbolt.h
SRCS+=	device_if.h bus_if.h pci_if.h pcib_if.h tb_if.c tb_if.h
SRCS+=	opt_acpi.h acpi_if.h acpi_wmi_if.h

opt_thunderbolt.h:
	echo "#define THUNDERBOLT_DEBUG 1" > ${.TARGET}

.include <bsd.kmod.mk>

.PATH: ${SRCTOP}/sys/dev/thunderbolt

KMOD=	tb
SRCS=	nhi_pci.c nhi.c tb_pcib.c tb_acpi_pcib.c tb_debug.c nhi_wmi.c
SRCS+=	router.c hcm.c tb_dev.c if_tbt.c tb_xdomain.c tb_icm.c
SRCS+=	tb_rdma_shim.c
SRCS+=	opt_thunderbolt.h
SRCS+=	device_if.h bus_if.h pci_if.h pcib_if.h tb_if.c tb_if.h
SRCS+=	opt_acpi.h acpi_if.h acpi_wmi_if.h

# Export tb_rdma_* symbols so the out-of-tree RDMA provider (tbrdma.ko) can
# link against this module at kldload time.
EXPORT_SYMS=	YES

opt_thunderbolt.h:
	echo "#define THUNDERBOLT_DEBUG 1" > ${.TARGET}

.include <bsd.kmod.mk>

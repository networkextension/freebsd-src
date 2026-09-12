.PATH:	${SRCTOP}/sys/dev/sff

KMOD=	sff

SRCS+=	sff.c
SRCS+=	sff_if.c sff_if.h
SRCS+=	bus_if.h device_if.h iicbus_if.h

.if !empty(OPT_FDT)
SRCS+=	sfp_fdt.c \
	ofw_bus_if.h
.endif

#
# sff(4) exists to be consumed by NIC drivers: they need sff_read_eeprom(),
# sff_i2c_requester() and the SFF_* kobj method descriptors.  Without this the
# kmod build localises every symbol and a consumer module fails at kldload.
#
EXPORT_SYMS=	YES

.include <bsd.kmod.mk>

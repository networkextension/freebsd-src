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
# sff(4) 存在的意义就是被 NIC 驱动消费:它们需要 sff_read_eeprom() 和
# SFF_* kobj 方法描述符。没有这行,kmod 构建会把符号全部局部化,
# 消费者在 kldload 时失败。
#
EXPORT_SYMS=	YES

.include <bsd.kmod.mk>

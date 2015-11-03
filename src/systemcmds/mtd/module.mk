#
# RAMTRON file system driver
#

MODULE_COMMAND	 = mtd
SRCS		 = mtd.c q25.c

MAXOPTIMIZATION	 = -Os

EXTRACFLAGS	= -Wno-error -Wno-unused-but-set-variable


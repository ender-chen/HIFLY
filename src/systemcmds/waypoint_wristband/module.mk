#
# Assorted tests and the like
#

MODULE_COMMAND		 = waypoint_wb
MODULE_STACKSIZE	 = 12000
MAXOPTIMIZATION		 = -Os

SRCS			 = waypoint_wb_main.cpp

EXTRACXXFLAGS = -Wframe-larger-than=2500 -Wno-float-equal -Wno-double-promotion -Wno-error=logical-op


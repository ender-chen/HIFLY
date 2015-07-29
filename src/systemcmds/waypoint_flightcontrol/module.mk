#
# Assorted tests and the like
#

MODULE_COMMAND		 = waypoint_fc
MODULE_STACKSIZE	 = 12000
MAXOPTIMIZATION		 = -Os

SRCS			 = waypoint_fc_main.cpp

EXTRACXXFLAGS = -Wframe-larger-than=2500 -Wno-float-equal -Wno-double-promotion -Wno-error=logical-op


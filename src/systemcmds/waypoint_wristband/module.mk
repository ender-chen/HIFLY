#
# Assorted tests and the like
#

MODULE_COMMAND		 = waypoint_wb
MODULE_STACKSIZE	 = 1200
MAXOPTIMIZATION		 = -Os

SRCS			 = waypoint_wb_main.cpp \
    waypoint_wb_params.c

EXTRACXXFLAGS = -Wno-double-promotion


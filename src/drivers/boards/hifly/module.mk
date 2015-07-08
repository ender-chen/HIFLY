#
# Board-specific startup code for the HIFLY
#

SRCS		 = hifly_can.c \
		   hifly_init.c \
		   hifly_pwm_servo.c \
		   hifly_spi.c \
		   hifly_usb.c \
		   hifly_led.c

MAXOPTIMIZATION	 = -Os

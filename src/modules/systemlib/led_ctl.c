/****************************************************************************
 * px4/sensors/test_gpio.c
 *
 *  Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <px4_posix.h>
#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include <arch/board/board.h>

#include <drivers/drv_led.h>

#include "led_ctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int led_ctl_open(void)
{
	int		fd = -1;
	fd = px4_open(LED0_DEVICE_PATH, 0);
	if (fd < 0) {
		printf("\tLED: open fail\n");
		return ERROR;
	}

	return fd;
}

void led_ctl_on(int fd, int led)
{
	if (fd > 0) {
		ioctl(fd, LED_ON, led);
	}
}

void led_ctl_off(int fd, int led)
{
	if (fd > 0) {
		ioctl(fd, LED_OFF, led);
	}
}

void led_ctl_toggle(int fd, int led)
{
	if (fd > 0) {
		ioctl(fd, LED_TOGGLE, led);
	}
}

void led_ctl_blink(int fd, int led, int seconds)
{
	if (fd > 0) {
		int i = 0;
		int interval = 50000;
		int count = seconds*1000000/interval;
		for (i = 0; i < count; ++i) {
			led_ctl_toggle(fd, led);
			usleep(interval);
		}
	}
}

void led_ctl_close(int fd)
{
	if (fd > 0) {
		close(fd);
	}
}

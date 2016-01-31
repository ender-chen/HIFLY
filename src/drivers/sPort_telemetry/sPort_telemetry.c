/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file sPort_telemetry.c
 * @author Mark Whitehorn <kd0aij@github.com>
 * @author Stefan Rado <px4@sradonia.net>
 *
 * FrSky SmartPort telemetry implementation.
 *
 * This daemon emulates FrSky SmartPort sensors by responding to polling
 * packets received from an attached FrSky X series receiver.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>

#include "sPort_data.h"


/* thread state */
static volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int sPort_task;

/* functions */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config_original);
static void usage(void);
static int sPort_telemetry_thread_main(int argc, char *argv[]);
__EXPORT int sPort_telemetry_main(int argc, char *argv[]);


/**
 * Opens the UART device and sets all required serial parameters.
 */
static int sPort_open_uart(const char *uart_name, struct termios *uart_config_original)
{
	/* Open UART */
	const int uart = open(uart_name, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		err(1, "Error opening port: %s", uart_name);
	}

	/* make sure uart FD is blocking */
	int flags = fcntl(uart, F_GETFL);
	if (flags < 0) {
		err(1, "Error getting FD flags: %s", uart_name);
	}
	flags &= ~O_NONBLOCK;
	flags = fcntl(uart, F_SETFL, flags);
	if (flags < 0) {
		err(1, "Error setting FD flags: %s", uart_name);
	}
	
	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		warnx("ERR: tcgetattr%s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Disable output post-processing */
	uart_config.c_oflag &= ~OPOST;

	/* Set baud rate */
	static const speed_t speed = B57600;

	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR: %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;
}

/**
 * Print command usage information
 */
static void usage()
{
	fprintf(stderr,
		"usage: sPort_telemetry start [-d <devicename>]\n"
		"       sPort_telemetry stop\n"
		"       sPort_telemetry status\n");
	exit(1);
}

/**
 * The daemon thread.
 */
static int sPort_telemetry_thread_main(int argc, char *argv[])
{
	/* Default values for arguments */
	char *device_name = "/dev/ttyS1"; /* USART2 */

	/* Work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	int ch;

	while ((ch = getopt(argc, argv, "d:")) != EOF) {
		switch (ch) {
		case 'd':
			device_name = optarg;
			break;

		default:
			usage();
			break;
		}
	}

	/* Open UART */
	warnx("opening uart");
	struct termios uart_config_original;
	const int uart = sPort_open_uart(device_name, &uart_config_original);

	if (uart < 0) {
		warnx("could not open %s", device_name);
		err(1, "could not open %s", device_name);
	}

	/* Subscribe to topics */
	sPort_init();

	thread_running = true;

	/* Main thread loop */
	char sbuf[2];
	uint8_t fiftyfive = 0x55;

	while (!thread_should_exit) {

		/* wait for poll frame starting with value 0x7E */
		int newBytes = read(uart, &sbuf[0], 1);
//		warnx("%x, %x \n", sbuf[0], sbuf[1]);
		
		if (newBytes < 1 || sbuf[0] != 0x7E) continue;
		
		/* read the ID byte */
		sbuf[1] = read(uart, &sbuf[0], 1);
		
//		sPort_send_data(uart, id, 7);
		
		/*** test ***/
		/* write single byte */
		write(uart, &fiftyfive, 1);
		/* read it back */
		read(uart, &sbuf[0], 1);
	}

	/* Reset the UART flags to original state */
	tcsetattr(uart, TCSANOW, &uart_config_original);
	close(uart);

	thread_running = false;
	return 0;
}

/**
 * The main command function.
 * Processes command line arguments and starts the daemon.
 */
int sPort_telemetry_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("missing command");
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* this is not an error */
		if (thread_running) {
			errx(0, "sPort_telemetry already running");
		}

		thread_should_exit = false;
		sPort_task = px4_task_spawn_cmd("sPort_telemetry",
						SCHED_DEFAULT,
						SCHED_PRIORITY_DEFAULT,
						2000,
						sPort_telemetry_thread_main,
						(char *const *)argv);

		while (!thread_running) {
			usleep(200);
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {

		/* this is not an error */
		if (!thread_running) {
			errx(0, "sPort_telemetry already stopped");
		}

		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			warnx(".");
		}

		warnx("terminated.");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	usage();
	/* not getting here */
	return 0;
}

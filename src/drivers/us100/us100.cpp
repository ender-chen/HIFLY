/**
 * @file us100.cpp
 * @author pengyin.huang <pengyin.huang@ckt-telecom.com>
 *
 *
 */

#include <fcntl.h>
#include <nuttx/config.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <termios.h>

#include <drivers/drv_px4flow.h>
#include <uORB/topics/optical_flow.h>
#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#define DEFAULT_UART "/dev/ttyS6";		/**< USART2*/

#define FILTER_COUNTER 30
/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif

orb_advert_t            _px4flow_topic;

static const int ERROR = -1;
pthread_mutex_t send_poll_mutex;
static int thread_should_exit = false;		/**< Deamon exit flag */
static int thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static const char daemon_name[] = "us100";
static const char commandline_usage[] = "usage: us100_sonar start|status|stop [-d <device>]";

/**
 * Deamon management function.
 */
extern "C" __EXPORT int us100_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int us100_sonar_thread_main(int argc, char *argv[]);

static int send_poll(int uart, uint8_t *buffer, size_t size);
static int open_uart(const char *device);
int
open_uart(const char *device)
{
        /* baud rate */
        static const speed_t speed = B9600;

        /* open uart */
        const int uart = open(device, O_RDWR | O_NOCTTY);

        if (uart < 0) {
                err(1, "ERR: opening %s", device);
        }

        /* Back up the original uart configuration to restore it after exit */
        int termios_state;
        struct termios uart_config_original;
        if ((termios_state = tcgetattr(uart, &uart_config_original)) < 0) {
                close(uart);
                err(1, "ERR: %s: %d", device, termios_state);
        }

        /* Fill the struct for the new configuration */
        struct termios uart_config;
        tcgetattr(uart, &uart_config);

        /* Clear ONLCR flag (which appends a CR for every LF) */
        uart_config.c_oflag &= ~ONLCR;

        /* Set baud rate */
        if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
                close(uart);
                err(1, "ERR: %s: %d (cfsetispeed, cfsetospeed)",
                         device, termios_state);
        }

        if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
                close(uart);
                err(1, "ERR: %s (tcsetattr)", device);
        }

        return uart;
}



int
send_poll(int uart, uint8_t *buffer, size_t size)
{
	pthread_mutex_lock(&send_poll_mutex);
	for(size_t i = 0; i < size; i++)
	{
		write(uart, &buffer[i], sizeof(buffer[i]));
	}
	uint8_t buf[2];
/*	for(int i = 0; i < 2; i++)
		read(uart, &dummy[i], sizeof(dummy[i]));*/
//	read(uart, &dummy, size);
	usleep(800);
	struct pollfd fds;
	fds.fd = uart;
	fds.events = POLLIN;
	if(poll(&fds, 1, 10000) > 0)
	{
		read(uart, buf, 2);
	}
	pthread_mutex_unlock(&send_poll_mutex);
	int32_t gdist_aver = 0;
	static int16_t gdist[FILTER_COUNTER] = {0};
	static int j = 0;
	//Sliding window filter on sonar
	if(((uint16_t)buf[0] << 8 | buf[1]) > 7000)
	{
		if(j == 0)
		{
			gdist[j] = gdist[FILTER_COUNTER - 1];
		}
		else{
            j++;
			gdist[j] = gdist[j - 1];
		}
	}
	else
	{
		gdist[j++] = (uint16_t)buf[0] << 8 | buf[1];
	}
	if(j == FILTER_COUNTER)
	{
		j = 0;
	}
	for(int i = 0; i < FILTER_COUNTER; i++)
	{
		gdist_aver += gdist[i];
	}
	gdist_aver /= FILTER_COUNTER;
	struct optical_flow_s report;
        //report.flow_comp_x_m = 0;
        //report.flow_comp_y_m = 0;
        //report.flow_raw_x = 0;
        //report.flow_raw_y = 0;
        report.ground_distance_m = float(gdist_aver) / 1000.0f;
        report.quality =  0;
        report.sensor_id = 0;
        report.timestamp = hrt_absolute_time();

	warnx("%d", gdist_aver);
        /* publish it */
        orb_publish(ORB_ID(optical_flow), _px4flow_topic, &report);
	return OK;
}


int
us100_sonar_thread_main(int argc, char *argv[])
{
	warnx("starting");

	thread_running = true;

	const char *device = DEFAULT_UART;

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) { //device set
			if (argc > i + 1) {
				device = argv[i + 1];

			} else {
				thread_running = false;
				errx(1, "missing parameter to -d\n%s", commandline_usage);
			}
		}
	}

	/* enable UART, writes potentially an empty buffer, but multiplexing is disabled */
	const int uart = open_uart(device);
	if (uart < 0) {
		errx(1, "Failed opening us100 UART, exiting.");
		thread_running = false;
	}
	pthread_mutex_init(&send_poll_mutex, NULL);
	uint8_t buffer[1];
//	size_t size = 0;
	/* get a publish handle on the px4flow topic */
        struct optical_flow_s zero_report;
        memset(&zero_report, 0, sizeof(zero_report));
	_px4flow_topic = orb_advertise(ORB_ID(optical_flow), &zero_report);
	while (!thread_should_exit) {
		// Currently we only support a General Air Module sensor.
		//build_gam_request(&buffer[0], &size);
		buffer[0] = 0x55;
		send_poll(uart, buffer, 1);

		// The sensor will need a little time before it starts sending.
		usleep(100000);
	}

	warnx("exiting");
	close(uart);
	thread_running = false;

	return 0;
}

/**
 * Process command line arguments and start the daemon.
 */
int
us100_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "missing command\n%s", commandline_usage);
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running");
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd(daemon_name,
					     SCHED_DEFAULT,
					     SCHED_PRIORITY_DEFAULT,
					     1024,
					     us100_sonar_thread_main,
					     (argv) ? (char **)&argv[2] : (char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		exit(0);
	}

	errx(1, "unrecognized command\n%s", commandline_usage);
}

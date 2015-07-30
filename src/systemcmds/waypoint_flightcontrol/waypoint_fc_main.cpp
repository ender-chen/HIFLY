/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file waypoint_fc_main.c
 *
 *
 *
 * @author mingliangwu
 */

#include <sys/stat.h>
#include <poll.h>
#include <dirent.h>
#include <stdio.h>
#include <stddef.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <string.h>
#include <uORB/topics/waypoint_excuted_report.h>
#include <uORB/topics/waypoint_received_report.h>


#include <drivers/drv_hrt.h>

#define CL "\033[K" // clear line

/**
 * app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int waypoint_fc_main(int argc, char *argv[]);


static int _task = -1;
static bool _task_should_exit = true;    /**< daemon exit flag */

static void usage()
{
    errx(1, "usage: waypoint_fc {start|stop}");
}

static int start();
static int stop();
static int waypoint_fc_task_main();

int waypoint_fc_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage();
    }

    if (!strcmp(argv[1], "start")) {

        start();
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        stop();
    }

    return 0;
}

static int start()
{
    ASSERT(_task == -1);

    struct stat buffer;
    if (stat("/fs/microsd/", &buffer))
    {
        warnx("no microSD card mounted, aborting waypoint report test\n");
        return -1;
    }

    _task_should_exit = false;

    /* start the task */
    _task = task_spawn_cmd("waypoint_fc",
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT + 20,
            1500,
            (main_t)waypoint_fc_task_main,
            nullptr);

    if (_task < 0) {
        warn("task start failed errno=%d", errno);
        return -1;
    }

    return 0;
}

static int stop()
{
    if (_task != -1) {

        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                task_delete(_task);
                break;
            }
        } while (_task != -1);
    }

    return 0;
}

static int
waypoint_fc_task_main()
{
    warnx("task enter\n");

    int fd_waypoint_received = -1;
    char received_report_filename[64] = {'\0'};
    sprintf(received_report_filename, "%s.txt", "/fs/microsd/waypoint_received");

    int fd_waypoint_excuted = -1;
    char excuted_report_filename[64] = {'\0'};
    sprintf(excuted_report_filename, "%s.txt", "/fs/microsd/waypoint_excuted");

    fd_waypoint_received = open(received_report_filename, O_APPEND | O_WRONLY | O_CREAT);
    if (fd_waypoint_received < 0)
    {
        warnx("failed to create %s fd=%d\n errno=%d",
                received_report_filename, fd_waypoint_received, errno);
        perror(received_report_filename);
        return 1;
    }
	else
	{
	    const char* start = "start:recv_time lat lon alt\n";
		write(fd_waypoint_received, start, strlen(start));
        fsync(fd_waypoint_received);
	}

    fd_waypoint_excuted = open(excuted_report_filename, O_APPEND | O_WRONLY | O_CREAT);
    if (fd_waypoint_excuted < 0)
    {
        warnx("failed to create %s fd=%d errno=%d\n",
                excuted_report_filename, fd_waypoint_excuted, errno);
        perror(excuted_report_filename);
        return 1;
    }
	else
	{
	    const char* start = "start:start_time end_time lat lon alt\n";
		write(fd_waypoint_excuted, start, strlen(start));
        fsync(fd_waypoint_excuted);
	}

    int _waypoint_received_report_sub = orb_subscribe(ORB_ID(waypoint_received_report));
    int _waypoint_excuted_report_sub = orb_subscribe(ORB_ID(waypoint_excuted_report));
    while(!_task_should_exit)
    {
        bool updated = false;

        orb_check(_waypoint_received_report_sub, &updated);
        if (updated)
        {
            struct waypoint_received_report_s waypoint_received;
            orb_copy(ORB_ID(waypoint_received_report), _waypoint_received_report_sub, &waypoint_received);

            char buf[64] ;
            memset(buf, 0, 64);
            sprintf(buf, "%llu\t%.7f\t%.7f\t%.7f\n",
                    waypoint_received.receive_time, (double)waypoint_received.lat, 
                    (double)waypoint_received.lon, (double)waypoint_received.altitude);
            write(fd_waypoint_received, buf, strlen(buf));
            fsync(fd_waypoint_received);

        }

        orb_check(_waypoint_excuted_report_sub, &updated);
        if (updated)
        {
            struct waypoint_excuted_report_s waypoint_excuted;
            orb_copy(ORB_ID(waypoint_excuted_report), _waypoint_excuted_report_sub, &waypoint_excuted);
            char buf[64];
            memset(buf, 0, 64);
            sprintf(buf, "%llu\t%llu\t%.7f\t%.7f\t%.7f\n",
                    waypoint_excuted.start_time, waypoint_excuted.end_time,
                    (double)waypoint_excuted.lat, (double)waypoint_excuted.lon, 
                    (double)waypoint_excuted.altitude);
            write(fd_waypoint_excuted, buf, strlen(buf) + 1);
            fsync(fd_waypoint_excuted);
        }
        usleep(100000);

    }

    if (fd_waypoint_received > 0)
    {
        close(fd_waypoint_received);
    }

    if (fd_waypoint_excuted > 0)
    {
        close(fd_waypoint_excuted);
    }

    warnx("task exit\n");
    return 0;
}

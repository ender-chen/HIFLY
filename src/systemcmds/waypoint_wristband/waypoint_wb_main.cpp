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
 * @file waypoint_wb_main.c
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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_command.h>

#include <drivers/drv_hrt.h>
#include <math.h>
#include <systemlib/param/param.h>


#define CL "\033[K" // clear line

/**
 * app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int waypoint_wb_main(int argc, char *argv[]);


static int _task = -1;
static bool _task_should_exit = true;    /**< daemon exit flag */

static void usage()
{
    errx(1, "usage: waypoint_wb {start|stop}");
}

static int start();
static int stop();
static int waypoint_wb_task_main();

int waypoint_wb_main(int argc, char *argv[])
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
    warnx("task start");

    if (_task > 0)
    {
        warn("task is running");
        return -1;
    }

    _task_should_exit = false;

    /* start the task */
    _task = task_spawn_cmd("waypoint_wb",
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT + 20,
            1500,
            (main_t)waypoint_wb_task_main,
            nullptr);

    if (_task < 0) {
        warn("task start failed errno=%d", errno);
        return -1;
    }

    return 0;
}

static int stop()
{
    if (_task > 0) {

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
                warnx("task delete");
                break;
            }
        } while (_task > 0);
    }

    warnx("task stop");
    return 0;
}

    static int
waypoint_wb_task_main()
{
    warnx("task enter");

    const char* start_str = "start: send_time, lat, lon, alt";

    int fd_waypoint_sent = -1;
    char sent_report_filename[64] = {'\0'};
    sprintf(sent_report_filename, "%s.txt", "/fs/microsd/waypoint_sent");

    fd_waypoint_sent = open(sent_report_filename, O_APPEND | O_WRONLY | O_CREAT);
    if (fd_waypoint_sent <= 0)
    {
        warnx("failed to create %s fd=%d errno=%d",
                sent_report_filename, fd_waypoint_sent, errno);
    }
    else
    {
        write(fd_waypoint_sent, start_str, strlen(start_str));
    }

    int _global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    int _gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    struct vehicle_gps_position_s gps_position;
    memset(&gps_position, 0 ,sizeof(gps_position));

    param_t _param_fc_system_id = param_find("WB_2FC_SYS_ID");
    param_t _param_fc_component_id = param_find("WB_2FC_COMP_ID");
    param_t _param_eph_threshold = parm_find("COM_HOME_H_T");
    int32_t system_id;
    param_get(_param_fc_system_id, &system_id);
    int32_t component_id;
    param_get(_param_fc_component_id, &component_id);
    int32_t eph_threshold;
    param_get(_param_eph_threshold, &eph_threshold);


    orb_advert_t _cmd_long_pub = -1;
    while(!_task_should_exit)
    {

        bool updated = false;
        orb_check(_global_position_sub, &updated);
        if (updated)
        {
            struct vehicle_global_position_s global_position;
            orb_copy(ORB_ID(vehicle_global_position), _global_position_sub, &global_position);

            orb_check(_gps_position_sub, &updated);
            if (updated)
            {
                orb_copy(ORB_ID(vehicle_gps_position), _gps_position_sub, &gps_position);
            }
            float vel_xy = sqrtf(gps_position.vel_n_m_s * gps_position.vel_n_m_s
                + gps_position.vel_e_m_s * gps_position.vel_e_m_s);

            struct vehicle_command_s cmd;
            cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_WAYPOINT;
            cmd.target_system = system_id;
            cmd.target_component = component_id;
            cmd.param2 =(float)gps_position.vel_n_m_s;
            cmd.param3 =(float)gps_position.vel_e_m_s;
            cmd.param4 =(float)gps_position.vel_d_m_s;
            cmd.param5 =(float)global_position.lat;
            cmd.param6 =(float)global_position.lon;
            cmd.param7 =(float)global_position.alt;
            warnx("global position update %d, %d %d, %.7f %.7f %.7f %.7f %.7f, %.7f %.7f, eph%.7f epv%.7f sa%d",
                cmd.command, cmd.target_system, cmd.target_component,
                cmd.param2, cmd.param3, cmd.param4,
                cmd.param5,cmd.param6,
                vel_xy, gps_position.vel_d_m_s,
                gps_position.eph, gps_position.epv, gps_position.satellites_used);
            if (gps_position.fix_type == 3 && gps_position.eph < eph_threshold)
            {
                if (_cmd_long_pub <= 0)
                {
                    _cmd_long_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
                }
                else
                {
                    orb_publish(ORB_ID(vehicle_command), _cmd_long_pub, &cmd);
                }
            }

            char buf[256] ;
            memset(buf, 0, 256);
            sprintf(buf, "%llu\t%.7f\t%.7f\t%.7f\t,%.7f\t%.7f\t,eph%.7f\t epv%.7f\t sa%d\t",
                    global_position.time_utc_usec, global_position.lat,
                    global_position.lon, (double)global_position.alt, vel_xy, gps_position.vel_d_m_s, 
                    gps_position.eph, gps_position.epv, gps_position.satellites_used);
            if (fd_waypoint_sent > 0)
            {
                write(fd_waypoint_sent, buf, strlen(buf));
            }
        }

        usleep(100000);

    }

    if (fd_waypoint_sent > 0)
    {
        fsync(fd_waypoint_sent);
        close(fd_waypoint_sent);
        fd_waypoint_sent = -1;
    }
    warnx("task exit");

    _task = -1;
    return 0;
}

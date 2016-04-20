/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file navigator_main.cpp
 *
 * Handles mission items, geo fencing and failsafe navigation behavior.
 * Published the position setpoint triplet for the position controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/fence.h>
#include <uORB/topics/navigation_capabilities.h>
#include <uORB/topics/vehicle_command.h>
#include <drivers/drv_baro.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <geo/geo.h>
#include <dataman/dataman.h>
#include <mathlib/mathlib.h>
#include <mavlink/mavlink_log.h>

#include "navigator.h"

/**
 * navigator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int navigator_main(int argc, char *argv[]);

#define GEOFENCE_CHECK_INTERVAL 200000

#define TARGET_NONE_ERROR 0
#define TARGET_POS_ERROR 1
#define TARGET_ALT_ERROR 2
#define TARGET_VEL_ERROR 3
#define TARGET_WAIT_DATA 4

namespace navigator
{

Navigator	*g_navigator;
}

Navigator::Navigator() :
	SuperBlock(NULL, "NAV"),
	_task_should_exit(false),
	_navigator_task(-1),
	_mavlink_fd(-1),
	_global_pos_sub(-1),
	_gps_pos_sub(-1),
	_home_pos_sub(-1),
	_vstatus_sub(-1),
	_capabilities_sub(-1),
	_control_mode_sub(-1),
	_onboard_mission_sub(-1),
	_offboard_mission_sub(-1),
	_param_update_sub(-1),
	_vehicle_command_sub(-1),
	_follow_target_sub(-1),
	_follow_ref_pos_sub(-1),
	_pos_sp_triplet_pub(nullptr),
	_mission_result_pub(nullptr),
	_geofence_result_pub(nullptr),
	_att_sp_pub(nullptr),
	_vstatus{},
	_control_mode{},
	_global_pos{},
	_gps_pos{},
	_sensor_combined{},
	_home_pos{},
	_mission_item{},
	_nav_caps{},
	_pos_sp_triplet{},
	_mission_result{},
	_att_sp{},
	_previous_target{},
	_current_target{},
	_follow_ref_pos{},
	_mission_item_valid(false),
	_mission_instance_count(0),
	_loop_perf(perf_alloc(PC_ELAPSED, "navigator")),
	_geofence{},
	_geofence_violation_warning_sent(false),
	_inside_fence(true),
	_can_loiter_at_sp(false),
	_pos_sp_triplet_updated(false),
	_pos_sp_triplet_published_invalid_once(false),
	_mission_result_updated(false),
	_target_valid(false),
	_navigation_mode(nullptr),
	_mission(this, "MIS"),
	_loiter(this, "LOI"),
	_takeoff(this, "TKF"),
	_land(this, "LND"),
	_rtl(this, "RTL"),
	_rcLoss(this, "RCL"),
	_dataLinkLoss(this, "DLL"),
	_engineFailure(this, "EF"),
	_gpsFailure(this, "GPSF"),
	_idle(this, "IDLE"),
    _follow_camera(this, "FOLCAM"),
	_follow_circle(this, "FOLCLE"),
	_follow_far_close(this, "FOLFC"),
	_follow_loiter(this, "FOLLOI"),
	_follow_target(this, "FOLTAR"),
	_param_loiter_radius(this, "LOITER_RAD"),
	_param_acceptance_radius(this, "ACC_RAD"),
	_param_datalinkloss_obc(this, "DLL_OBC"),
	_param_rcloss_obc(this, "RCL_OBC"),
	_param_target_debug(this, "TAR_DEBUG_EN", false),
	_param_target_timeout(this, "TAR_TIMEOUT", false),
	_param_target_vel_acc_max(this, "TAR_VEL_ACC_MAX", false),
	_param_target_pos_acc_max(this, "TAR_POS_ACC_MAX", false),
	_param_target_vel_max(this, "TAR_VEL_MAX", false)
{
	/* Create a list of our possible navigation types */
	_navigation_mode_array[0] = &_mission;
	_navigation_mode_array[1] = &_loiter;
	_navigation_mode_array[2] = &_rtl;
	_navigation_mode_array[3] = &_dataLinkLoss;
	_navigation_mode_array[4] = &_engineFailure;
	_navigation_mode_array[5] = &_gpsFailure;
	_navigation_mode_array[6] = &_rcLoss;
	_navigation_mode_array[7] = &_takeoff;
	_navigation_mode_array[8] = &_land;
	_navigation_mode_array[9] = &_idle;
	_navigation_mode_array[10] = &_follow_camera;
	_navigation_mode_array[11] = &_follow_circle;
	_navigation_mode_array[12] = &_follow_far_close;
	_navigation_mode_array[13] = &_follow_target;

	updateParams();
}

Navigator::~Navigator()
{
	if (_navigator_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_navigator_task);
				break;
			}
		} while (_navigator_task != -1);
	}

	navigator::g_navigator = nullptr;
}

void
Navigator::global_position_update()
{
	orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
}

void
Navigator::gps_position_update()
{
	orb_copy(ORB_ID(vehicle_gps_position), _gps_pos_sub, &_gps_pos);
}

void
Navigator::sensor_combined_update()
{
	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
}

void
Navigator::home_position_update(bool force)
{
	bool updated = false;
	orb_check(_home_pos_sub, &updated);

	if (updated || force) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}
}

void
Navigator::navigation_capabilities_update()
{
	orb_copy(ORB_ID(navigation_capabilities), _capabilities_sub, &_nav_caps);
}

void
Navigator::vehicle_status_update()
{
	if (orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus) != OK) {
		/* in case the commander is not be running */
		_vstatus.arming_state = vehicle_status_s::ARMING_STATE_STANDBY;
	}
}

void
Navigator::vehicle_control_mode_update()
{
	if (orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode) != OK) {
		/* in case the commander is not be running */
		_control_mode.flag_control_auto_enabled = false;
		_control_mode.flag_armed = false;
	}
}

void
Navigator::params_update()
{
	parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), _param_update_sub, &param_update);
}

void
Navigator::follow_target_update()
{
	orb_copy(ORB_ID(follow_target), _follow_target_sub, &_current_target);

	if (_param_target_debug.get() == 1 && _previous_target.timestamp != 0) {
		uint64_t time_interval = (_current_target.timestamp - _previous_target.timestamp) / 1000.0f;
		uint32_t seq_interval = _current_target.seq - _previous_target.seq;

		mavlink_log_info(_mavlink_fd,"interval: time %llu ms, seq %d", time_interval, seq_interval);
	}

	int target_status = target_validation_check();

	if (target_status == TARGET_NONE_ERROR) {
		set_target_valid();
	}
}

void
Navigator::follow_ref_pos_update()
{
	orb_copy(ORB_ID(follow_reference_position), _follow_ref_pos_sub, &_follow_ref_pos);
}

void
Navigator::task_main_trampoline(int argc, char *argv[])
{
	navigator::g_navigator->task_main();
}

void
Navigator::task_main()
{
	_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
	_geofence.setMavlinkFd(_mavlink_fd);

	bool have_geofence_position_data = false;

	/* Try to load the geofence:
	 * if /fs/microsd/etc/geofence.txt load from this file
	 * else clear geofence data in datamanager */
	struct stat buffer;

	if (stat(GEOFENCE_FILENAME, &buffer) == 0) {
		warnx("Try to load geofence.txt");
		_geofence.loadFromFile(GEOFENCE_FILENAME);

	} else {
		if (_geofence.clearDm() != OK) {
			mavlink_log_critical(_mavlink_fd, "failed clearing geofence");
		}
	}

	if (stat(RESTRICTED_AREA_FILENAME, &buffer) == 0) {
		mavlink_and_console_log_info(_mavlink_fd, "Try to load restricted area database");
		_geofence.load_restricted_area(RESTRICTED_AREA_FILENAME);

	} else {
		mavlink_and_console_log_critical(_mavlink_fd, "No restricted area database found");
	}

	/* do subscriptions */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_capabilities_sub = orb_subscribe(ORB_ID(navigation_capabilities));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_onboard_mission_sub = orb_subscribe(ORB_ID(onboard_mission));
	_offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));
	_param_update_sub = orb_subscribe(ORB_ID(parameter_update));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	_follow_ref_pos_sub = orb_subscribe(ORB_ID(follow_reference_position));

	/* copy all topics first time */
	vehicle_status_update();
	vehicle_control_mode_update();
	global_position_update();
	gps_position_update();
	sensor_combined_update();
	home_position_update(true);
	navigation_capabilities_update();
	params_update();

	hrt_abstime mavlink_open_time = 0;
	const hrt_abstime mavlink_open_interval = 500000;

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[1] = {};

	/* Setup of loop */
	fds[0].fd = _global_pos_sub;
	fds[0].events = POLLIN;

	bool global_pos_available_once = false;

	while (!_task_should_exit) {

		/* wait for up to 200ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			/* timed out - periodic check for _task_should_exit, etc. */
			if (global_pos_available_once) {
				PX4_WARN("navigator timed out");
			}
			continue;

		} else if (pret < 0) {
			/* this is undesirable but not much we can do - might want to flag unhappy status */
			PX4_WARN("nav: poll error %d, %d", pret, errno);
			continue;
		}

		global_pos_available_once = true;

		perf_begin(_loop_perf);

		if (_mavlink_fd < 0 && hrt_absolute_time() > mavlink_open_time) {
			/* try to reopen the mavlink log device with specified interval */
			mavlink_open_time = hrt_abstime() + mavlink_open_interval;
			_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
		}

		bool updated;

		/* gps updated */
		orb_check(_gps_pos_sub, &updated);
		if (updated) {
			gps_position_update();
			if (_geofence.getSource() == Geofence::GF_SOURCE_GPS) {
				have_geofence_position_data = true;
			}
		}

		/* sensors combined updated */
		orb_check(_sensor_combined_sub, &updated);
		if (updated) {
			sensor_combined_update();
		}

		/* parameters updated */
		orb_check(_param_update_sub, &updated);
		if (updated) {
			params_update();
			updateParams();
		}

		/* vehicle control mode updated */
		orb_check(_control_mode_sub, &updated);
		if (updated) {
			vehicle_control_mode_update();
		}

		/* vehicle status updated */
		orb_check(_vstatus_sub, &updated);
		if (updated) {
			vehicle_status_update();
		}

		/* navigation capabilities updated */
		orb_check(_capabilities_sub, &updated);
		if (updated) {
			navigation_capabilities_update();
		}

		/* home position updated */
		orb_check(_home_pos_sub, &updated);
		if (updated) {
			home_position_update();
		}

		/* follow target updated */
		orb_check(_follow_target_sub, &updated);
		if (updated) {
			follow_target_update();
		}

		/* follow reference position updated */
		orb_check(_follow_ref_pos_sub, &updated);
		if (updated) {
			follow_ref_pos_update();
		}

		orb_check(_vehicle_command_sub, &updated);
		if (updated) {
			vehicle_command_s cmd;
			orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &cmd);

			if (cmd.command == vehicle_command_s::VEHICLE_CMD_NAV_TAKEOFF) {
				warnx("navigator: got takeoff coordinates");
			}
		}

		/* global position updated */
		if (fds[0].revents & POLLIN) {
			global_position_update();
			if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
				have_geofence_position_data = true;
			}
		}

		/* Check geofence violation */
		static hrt_abstime last_geofence_check = 0;
		if (have_geofence_position_data &&
			(_geofence.getGeofenceAction() != geofence_result_s::GF_ACTION_NONE) &&
			(hrt_elapsed_time(&last_geofence_check) > GEOFENCE_CHECK_INTERVAL)) {
			bool inside = _geofence.inside(_global_pos, _gps_pos, _sensor_combined.baro_alt_meter[0], _home_pos, home_position_valid(), _geofence_result.hor_violated, _geofence_result.ver_violated);

			if (_geofence.getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
				_geofence_result.restricted_area_warning = _geofence.inside_restricted_area(_global_pos.lat, _global_pos.lon);
			} else {
				_geofence_result.restricted_area_warning = _geofence.inside_restricted_area((double)_gps_pos.lat * 1.0e-7, (double)_gps_pos.lon * 1.0e-7);
			}

			last_geofence_check = hrt_absolute_time();
			have_geofence_position_data = false;

			_geofence_result.geofence_action = _geofence.getGeofenceAction();
			if (!inside) {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = true;
				publish_geofence_result();

				/* Issue a warning about the geofence violation once */
				if (!_geofence_violation_warning_sent) {
					mavlink_log_critical(_mavlink_fd, "Geofence violation");
					_geofence_violation_warning_sent = true;
				}
			} else {
				/* inform other apps via the mission result */
				_geofence_result.geofence_violated = false;
				publish_geofence_result();
				/* Reset the _geofence_violation_warning_sent field */
				_geofence_violation_warning_sent = false;
			}
		}

		/* Do stuff according to navigation state set by commander */
		switch (_vstatus.nav_state) {
			case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			case vehicle_status_s::NAVIGATION_STATE_ACRO:
			case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
			case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
			case vehicle_status_s::NAVIGATION_STATE_DESCEND:
				_navigation_mode = nullptr;
				_can_loiter_at_sp = false;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
				if (_nav_caps.abort_landing) {
					// pos controller aborted landing, requests loiter
					// above landing waypoint
					_navigation_mode = &_loiter;
					_pos_sp_triplet_published_invalid_once = false;
				} else {
					_pos_sp_triplet_published_invalid_once = false;
					_navigation_mode = &_mission;
				}
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_loiter;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
				_pos_sp_triplet_published_invalid_once = false;
				if (_param_rcloss_obc.get() != 0) {
					_navigation_mode = &_rcLoss;
				} else {
					_navigation_mode = &_rtl;
				}
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_rtl;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_takeoff;
				break;
			case vehicle_status_s::NAVIGATION_STATE_LAND:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_land;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
				/* Use complex data link loss mode only when enabled via param
				* otherwise use rtl */
				_pos_sp_triplet_published_invalid_once = false;
				if (_param_datalinkloss_obc.get() != 0) {
					_navigation_mode = &_dataLinkLoss;
				} else {
					_navigation_mode = &_rtl;
				}
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_engineFailure;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_gpsFailure;
				break;
			case vehicle_status_s::NAVIGATION_STATE_AUTO_IDLE:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_idle;
				break;
			case vehicle_status_s::NAVIGATION_STATE_FOLLOW_CAMERA:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_follow_camera;
				break;
			case vehicle_status_s::NAVIGATION_STATE_FOLLOW_CIRCLE:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_follow_circle;
				break;
			case vehicle_status_s::NAVIGATION_STATE_FOLLOW_FC_ARC:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_follow_far_close;
				break;
			case vehicle_status_s::NAVIGATION_STATE_FOLLOW_LOITER:
				_pos_sp_triplet_published_invalid_once = false;
				_navigation_mode = &_follow_target;
				break;
			default:
				_navigation_mode = nullptr;
				_can_loiter_at_sp = false;
				break;
		}

		/* iterate through navigation modes and set active/inactive for each */
		for (unsigned int i = 0; i < NAVIGATOR_MODE_ARRAY_SIZE; i++) {
			_navigation_mode_array[i]->run(_navigation_mode == _navigation_mode_array[i]);
		}

		/* if nothing is running, set position setpoint triplet invalid once */
		if (_navigation_mode == nullptr && !_pos_sp_triplet_published_invalid_once) {
			_pos_sp_triplet_published_invalid_once = true;
			_pos_sp_triplet.previous.valid = false;
			_pos_sp_triplet.current.valid = false;
			_pos_sp_triplet.next.valid = false;
			_pos_sp_triplet_updated = true;
		}

		if (_pos_sp_triplet_updated) {
			publish_position_setpoint_triplet();
			_pos_sp_triplet_updated = false;
		}

		if (_mission_result_updated) {
			publish_mission_result();
			_mission_result_updated = false;
		}

		perf_end(_loop_perf);
	}
	warnx("exiting.");

	_navigator_task = -1;
	return;
}

int
Navigator::start()
{
	ASSERT(_navigator_task == -1);

	/* start the task */
	_navigator_task = px4_task_spawn_cmd("navigator",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT + 5,
					 1500,
					 (px4_main_t)&Navigator::task_main_trampoline,
					 nullptr);

	if (_navigator_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
Navigator::status()
{
	/* TODO: add this again */
	// warnx("Global position is %svalid", _global_pos_valid ? "" : "in");

	// if (_global_pos.global_valid) {
	// 	warnx("Longitude %5.5f degrees, latitude %5.5f degrees", _global_pos.lon, _global_pos.lat);
	// 	warnx("Altitude %5.5f meters, altitude above home %5.5f meters",
	// 	      (double)_global_pos.alt, (double)(_global_pos.alt - _home_pos.alt));
	// 	warnx("Ground velocity in m/s, N %5.5f, E %5.5f, D %5.5f",
	// 	      (double)_global_pos.vel_n, (double)_global_pos.vel_e, (double)_global_pos.vel_d);
	// 	warnx("Compass heading in degrees %5.5f", (double)(_global_pos.yaw * M_RAD_TO_DEG_F));
	// }

	if (_geofence.valid()) {
		warnx("Geofence is valid");
		/* TODO: needed? */
//		warnx("Vertex longitude latitude");
//		for (unsigned i = 0; i < _fence.count; i++)
//		warnx("%6u %9.5f %8.5f", i, (double)_fence.vertices[i].lon, (double)_fence.vertices[i].lat);

	} else {
		warnx("Geofence not set (no /etc/geofence.txt on microsd) or not valid");
	}
}

void
Navigator::publish_position_setpoint_triplet()
{
	/* update navigation state */
	_pos_sp_triplet.nav_state = _vstatus.nav_state;

	/* lazily publish the position setpoint triplet only once available */
	if (_pos_sp_triplet_pub != nullptr) {
		orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub, &_pos_sp_triplet);

	} else {
		_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet), &_pos_sp_triplet);
	}
}

float
Navigator::get_acceptance_radius()
{
	return get_acceptance_radius(_param_acceptance_radius.get());
}

float
Navigator::get_acceptance_radius(float mission_item_radius)
{
	float radius = mission_item_radius;

	// XXX only use navigation capabilities for now
	// when in fixed wing mode
	// this might need locking against a commanded transition
	// so that a stale _vstatus doesn't trigger an accepted mission item.
	if (!_vstatus.is_rotary_wing && !_vstatus.in_transition_mode && hrt_elapsed_time(&_nav_caps.timestamp) < 5000000) {
		if (_nav_caps.turn_distance > radius) {
			radius = _nav_caps.turn_distance;
		}
	}

	return radius;
}

void Navigator::add_fence_point(int argc, char *argv[])
{
	_geofence.addPoint(argc, argv);
}

void Navigator::load_fence_from_file(const char *filename)
{
	_geofence.loadFromFile(filename);
}


static void usage()
{
	warnx("usage: navigator {start|stop|status|fence|fencefile}");
}

int navigator_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (navigator::g_navigator != nullptr) {
			warnx("already running");
			return 1;
		}

		navigator::g_navigator = new Navigator;

		if (navigator::g_navigator == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != navigator::g_navigator->start()) {
			delete navigator::g_navigator;
			navigator::g_navigator = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (navigator::g_navigator == nullptr) {
		warnx("not running");
		return 1;
	}

	if (!strcmp(argv[1], "stop")) {
		delete navigator::g_navigator;
		navigator::g_navigator = nullptr;
	} else if (!strcmp(argv[1], "status")) {
		navigator::g_navigator->status();
	} else if (!strcmp(argv[1], "fence")) {
		navigator::g_navigator->add_fence_point(argc - 2, argv + 2);
	} else if (!strcmp(argv[1], "fencefile")) {
		navigator::g_navigator->load_fence_from_file(GEOFENCE_FILENAME);
	} else {
		usage();
		return 1;
	}

	return 0;
}

void
Navigator::publish_mission_result()
{
	_mission_result.instance_count = _mission_instance_count;

	/* lazily publish the mission result only once available */
	if (_mission_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(mission_result), _mission_result_pub, &_mission_result);

	} else {
		/* advertise and publish */
		_mission_result_pub = orb_advertise(ORB_ID(mission_result), &_mission_result);
	}

	/* reset some of the flags */
	_mission_result.seq_reached = false;
	_mission_result.seq_current = 0;
	_mission_result.item_do_jump_changed = false;
	_mission_result.item_changed_index = 0;
	_mission_result.item_do_jump_remaining = 0;
	_mission_result.valid = true;
}

void
Navigator::publish_geofence_result()
{

	/* lazily publish the geofence result only once available */
	if (_geofence_result_pub != nullptr) {
		/* publish mission result */
		orb_publish(ORB_ID(geofence_result), _geofence_result_pub, &_geofence_result);

	} else {
		/* advertise and publish */
		_geofence_result_pub = orb_advertise(ORB_ID(geofence_result), &_geofence_result);
	}
}

void
Navigator::publish_att_sp()
{
	/* lazily publish the attitude sp only once available */
	if (_att_sp_pub != nullptr) {
		/* publish att sp*/
		orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

	} else {
		/* advertise and publish */
		_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
	}
}

void
Navigator::set_mission_failure(const char* reason)
{
	if (!_mission_result.mission_failure) {
		_mission_result.mission_failure = true;
		set_mission_result_updated();
		mavlink_log_critical(_mavlink_fd, "%s", reason);
	}
}

#define VEL_N 0
#define VEL_E 1
#define VEL_D 2
#define ALT 3
#define LAT 4
#define LON 5
#define SUB_NUM 6

#define WP_NUM 5
static float wp_buf[SUB_NUM][WP_NUM + 1] = {0.0f,0.0f};

static float waypoint_filter(int sel, float val)
{
        float sum = 0.0f;
        wp_buf[sel][WP_NUM] = val;

        for(int i = 0; i < WP_NUM; i++) {
                wp_buf[sel][i] = wp_buf[sel][i+1];
                sum += wp_buf[sel][i];
        }

        return sum / WP_NUM;
}

int
Navigator::target_validation_check()
{
	static float last_hor_velocity = 0.0f;
	static float last_ver_velocity = 0.0f;
	static uint32_t valid_target_conut = 0;

	if (valid_target_conut != 0) {

		float dt_s = (_current_target.timestamp - _previous_target.timestamp) / 1000.0f / 1000.0f;
		math::Vector<3> pre_vel(_previous_target.vel_n_m_s, _previous_target.vel_e_m_s, 0.0f);
		math::Vector<3> cur_vel(_current_target.vel_n_m_s, _current_target.vel_e_m_s, 0.0f);
		math::Vector<3> dif_vel = cur_vel - pre_vel;

		// calculate horizontal acceleration via velocity
		float vel_acc_hor = dif_vel.length() / dt_s;

		if (vel_acc_hor > _param_target_vel_acc_max.get()) {
			mavlink_log_critical(_mavlink_fd, "target vel hor acc: %.1f", (double)vel_acc_hor);
			return TARGET_VEL_ERROR;
		}

		// calculate vertical acceleration via velocity
		float dif_vel_z = _current_target.vel_d_m_s - _previous_target.vel_d_m_s;
		float vel_acc_ver = fabsf(dif_vel_z) / dt_s;

		if (vel_acc_ver > _param_target_vel_acc_max.get()) {
			mavlink_log_critical(_mavlink_fd, "target vel ver acc: %.1f", (double)vel_acc_ver);
			return TARGET_VEL_ERROR;
		}

		float dist_xy = -1.0f;
		float dist_z = -1.0f;
		// calculate distance the target had moved
		get_distance_to_point_global_wgs84(_current_target.lat, _current_target.lon, _current_target.alt,
				_previous_target.lat, _previous_target.lon, _previous_target.alt,
				&dist_xy, &dist_z);

		float pos_vel = dist_xy / dt_s;
		if(pos_vel > _param_target_vel_max.get()) {
			mavlink_log_critical(_mavlink_fd, "target pos vel: %.1f", (double)pos_vel);
			return TARGET_POS_ERROR;
		}

		// need at least 2 data point for position velocity estimate
		if (valid_target_conut > 2 ) {
			// calculate target acceleration via position
			float pos_acc_hor = fabsf(dist_xy / dt_s - last_hor_velocity) / dt_s;

			// calculate horizontal acceleration via position
			if (pos_acc_hor > _param_target_pos_acc_max.get()) {
				mavlink_log_critical(_mavlink_fd, "target pos hor acc: %.1f", (double)pos_acc_hor);
				return TARGET_POS_ERROR;
			}

			// calculate vertical acceleration via position
			float pos_acc_ver = fabsf(dist_z / dt_s - last_ver_velocity) / dt_s;

			if (pos_acc_ver > _param_target_vel_acc_max.get()) {
				mavlink_log_critical(_mavlink_fd, "target pos ver acc: %.1f", (double)pos_acc_ver);
				return TARGET_POS_ERROR;
			}
		}

		last_hor_velocity = fabsf(dist_xy) / dt_s;
		last_ver_velocity = fabsf(dist_z) / dt_s;

		// reset target valid if target timeout for a long time
		if (dt_s > _param_target_timeout.get()) {
			valid_target_conut = 0;
		}
	}

	_previous_target = _current_target;

	// target filter
	_current_target.vel_n_m_s = waypoint_filter(VEL_N, _current_target.vel_n_m_s);
	_current_target.vel_e_m_s = waypoint_filter(VEL_E, _current_target.vel_e_m_s);
	_current_target.vel_d_m_s = waypoint_filter(VEL_D, _current_target.vel_d_m_s);
	_current_target.alt = waypoint_filter(ALT, _current_target.alt);

	valid_target_conut ++;

	// wait for target refresh;
	if (valid_target_conut <= WP_NUM) {
		return TARGET_WAIT_DATA;
	}

	return TARGET_NONE_ERROR;
}

struct follow_target_s* Navigator::get_valid_target()
{
	if(_target_valid == true) {
		_target_valid = false;
		return &_current_target;
	} else {
		return nullptr;
	}
}

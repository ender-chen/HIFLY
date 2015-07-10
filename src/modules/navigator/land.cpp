/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file navigator_land.cpp
 * Helper class to access LAND
 * @author Pengyin.huang@ck-telecom.com
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "land.h"

#define DELAY_SIGMA	0.01f

LAND::LAND(Navigator *navigator, const char *name) :
MissionBlock(navigator, name),
_land_state(LAND_STATE_NONE)
{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

LAND::~LAND()
{
}

void
LAND::on_inactive()
{
	/* reset LAND state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_land_state = LAND_STATE_NONE;
	}
}

void
LAND::on_activation()
{
	/* decide where to enter the LAND procedure when we switch into it */
	if (_land_state == LAND_STATE_NONE) {
		/* for safety reasons don't go into LAND if landed */
		if (_navigator->get_vstatus()->condition_landed) {
			_land_state = LAND_STATE_LANDED;
			mavlink_log_info(_navigator->get_mavlink_fd(), "#audio: no LAND when landed");
		} else {
			/* set altitude setpoint to current altitude */
			_land_state = LAND_STATE_LAND;
		}
	}

	set_land_item();
}

void
LAND::on_active()
{
	if (_land_state != LAND_STATE_LANDED && is_mission_item_reached()) {
		advance_land();
		set_land_item();
	}
}

void
LAND::set_land_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure we have the latest params */
	updateParams();

	set_previous_pos_setpoint();
	_navigator->set_can_loiter_at_sp(false);

	switch(_land_state){
		case LAND_STATE_LAND:
		_mission_item.lat = _navigator->get_global_position()->lat;
		_mission_item.lon = _navigator->get_global_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_home_position()->alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_LAND;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_info(_navigator->get_mavlink_fd(), "#audio: LAND: land at home alt");
		break;

		case LAND_STATE_LANDED:
		_mission_item.lat = _navigator->get_home_position()->lat;
		_mission_item.lon = _navigator->get_home_position()->lon;
		_mission_item.altitude_is_relative = false;
		_mission_item.altitude = _navigator->get_home_position()->alt;
		_mission_item.yaw = NAN;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = NAV_CMD_IDLE;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = 0.0f;
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = true;
		_mission_item.origin = ORIGIN_ONBOARD;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "LAND: completed, landed");
		break;

		default:
		break;
	}


	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
LAND::advance_land()
{
	switch (_land_state)
	{
		case LAND_STATE_LAND:
		_land_state = LAND_STATE_LANDED;
		break;

		default:
		break;
	}
}

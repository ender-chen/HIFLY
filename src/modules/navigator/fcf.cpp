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
 * @file navigator_fcf.cpp
 * Helper class to access fcf
 * @author dong chen <dong.chen@ck-telecom.com>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
//#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "fcf.h"

#define DELAY_SIGMA	0.01f

FCF::FCF(Navigator *navigator, const char *name) :
	NavigatorMode(navigator, name),
	_fcf_state(FCF_STATE_NONE),
	_fcf_item({0}),
	_waypoint({0}),
	_local_pos({0}),
	_ref_pos({0}),
	_ref_alt(0),
	_fcf_item_reached(false),
	_param_bottom_alt(this, "FCF_BOTTOM_ALT", false),
	_param_top_alt(this, "FCF_TOP_ALT", false),
	_param_hori_dist(this, "FCF_HORI_DIST", false),
	_param_acceptance_radius(this, "FCF_ACC_RAD", false),
	_param_rate_x(this, "FCF_RATE_X", false),
	_param_rate_y(this, "FCF_RATE_Y", false),
	_param_rate_z(this, "FCF_RATE_Z", false),
	_param_set_direction(this, "FCF_DIR", false)
{
	/* load initial params */
	updateParams();
	/* initial reset */
	on_inactive();
}

FCF::~FCF()
{
}

void
FCF::on_inactive()
{
	/* reset RTL state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_fcf_state = FCF_STATE_NONE;
	}
}

void
FCF::on_activation()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	if (_fcf_state == FCF_STATE_NONE) {
			_fcf_state = FCF_STATE_CLIMB;
	}
	waypoint_update();
	update_ref();
	target_local_position_update();
	set_fcf_item();
	fcf_item_to_position_setpoint(&_fcf_item, &pos_sp_triplet->current);
	_navigator->set_position_setpoint_triplet_updated();
}

void
FCF::on_active()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	if (is_fcf_item_reached()) {
		waypoint_update();
		update_ref();
		target_local_position_update();
		advance_fcf();
		set_fcf_item();
		
		_navigator->set_position_setpoint_triplet_updated();
	}
	fcf_item_to_position_setpoint(&_fcf_item, &pos_sp_triplet->current);
}

void
FCF::fcf_item_to_position_setpoint(const struct fcf_item_s *item, struct position_setpoint_s *sp)
{
	follow_strategy();
	sp->x = item->x;
	sp->y = item->y;
	sp->z = item->z;
	sp->yaw = get_bearing_to_point_local(_navigator->get_local_position()->x, _navigator->get_local_position()->y, get_target_local_position()->x, get_target_local_position()->y);
	sp->yaw_valid = true;
	sp->valid = true;
	sp->position_valid = true;
}

void
FCF::follow_strategy()
{
}

void
FCF::waypoint_update()
{
	bool waypoint_updated = false;
	orb_check(_navigator->get_waypoint_sub(), &waypoint_updated);
	if (waypoint_updated) {
		orb_copy(ORB_ID(waypoint), _navigator->get_waypoint_sub(), &_waypoint);
	}
}

void
FCF::target_local_position_update()
{
	double lat_sp = _waypoint.lat;
	double lon_sp = _waypoint.lon;
	float alt_sp = _waypoint.alt;
	map_projection_project(&_ref_pos, lat_sp, lon_sp, &_local_pos.x, &_local_pos.y);
	_local_pos.z = -(alt_sp - _ref_alt);
}

void
FCF::update_ref()
{
	map_projection_init(&_ref_pos, _navigator->get_local_position()->ref_lat, _navigator->get_local_position()->ref_lon); 
	_ref_alt = _navigator->get_local_position()->ref_alt;
}

void
FCF::set_fcf_item()
{
	/* make sure we have the latest params */
	updateParams();

	_navigator->set_can_loiter_at_sp(false);

	switch (_fcf_state) {
	case FCF_STATE_CLIMB: {
		float climb_alt = _param_bottom_alt.get();
		_fcf_item.x = get_target_local_position() -> x;
		_fcf_item.y = get_target_local_position() -> y;
		_fcf_item.z = -climb_alt;
		_fcf_item.yaw = NAN;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "FCF: Move to the target position");
		break;
	}

	case FCF_STATE_RIGHT_GO: {
		float top_alt = _param_top_alt.get();
		_fcf_item.x = get_target_local_position() -> x + _param_hori_dist.get() * fabsf(cosf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
		_fcf_item.y = get_target_local_position() -> y + _param_hori_dist.get() * fabsf(sinf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
		_fcf_item.z = -top_alt;
		_fcf_item.yaw = NAN;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "GO TO RIGHT");
		break;
	}

	case FCF_STATE_RIGHT_BACK: {
		float bottom_alt = _param_bottom_alt.get();
		_fcf_item.x = get_target_local_position() -> x;
		_fcf_item.y = get_target_local_position() -> y;
		_fcf_item.z = -bottom_alt;
		_fcf_item.yaw = NAN;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "FCF: right back to base alt.");
		break;
	}

	case FCF_STATE_LEFT_GO: {
		float top_alt = _param_top_alt.get();
		_fcf_item.x = get_target_local_position() -> x - _param_hori_dist.get() * fabsf(cosf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
		_fcf_item.y = get_target_local_position() -> y - _param_hori_dist.get() * fabsf(sinf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
		_fcf_item.z = -top_alt;
		_fcf_item.yaw = NAN;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "FCF: left go to top alt.");
		break;
	}
	case FCF_STATE_LEFT_BACK: {
		float bottom_alt = _param_bottom_alt.get();
		_fcf_item.x = get_target_local_position() -> x;
		_fcf_item.y = get_target_local_position() -> y;
		_fcf_item.z = -bottom_alt;
		_fcf_item.yaw = NAN;

		mavlink_log_critical(_navigator->get_mavlink_fd(), "FCF: left back to base alt.");
		break;
	}

	default:
		break;
	}

	reset_fcf_item_reached();
}

void
FCF::advance_fcf()
{
	switch (_fcf_state) {
	case FCF_STATE_CLIMB:
		_fcf_state = FCF_STATE_RIGHT_GO;
		break;

	case FCF_STATE_RIGHT_GO:
		_fcf_state = FCF_STATE_RIGHT_BACK;
		break;

	case FCF_STATE_RIGHT_BACK:
		_fcf_state = FCF_STATE_LEFT_GO;
		break;

	case FCF_STATE_LEFT_GO:
		_fcf_state = FCF_STATE_LEFT_BACK;
		break;

	case FCF_STATE_LEFT_BACK:
		_fcf_state = FCF_STATE_RIGHT_GO;
		break;

	default:
		break;
	}
}

bool
FCF::is_fcf_item_reached()
{
	if(!_fcf_item_reached)
	{
		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f; 
		dist = mavlink_wpm_distance_to_point_local(
							  _navigator->get_local_position()->x,
							  _navigator->get_local_position()->y,
							  _navigator->get_local_position()->z,
							  _fcf_item.x, _fcf_item.y, _fcf_item.z, &dist_xy, &dist_z);
		if(dist <= _param_acceptance_radius.get())
		{
			_fcf_item_reached = true;
			return true;
		}
	}
	return false;
}

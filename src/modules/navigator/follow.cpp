/*copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
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
 * @file follow.cpp
 *
 * Helper class to access follow
 *
 * @author liang.tang<liang.tang@ck-telecom.com>
 */

#include <sys/types.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <float.h>

#include <drivers/drv_hrt.h>

#include <dataman/dataman.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

#include "navigator.h"
#include "follow.h"

Follow::Follow(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_yaw_mode(this, "FOL_YAW_MODE",false),
	_param_wp_hordist(this, "FOL_WP_HORDIST",false),
	_param_wp_verdist(this, "FOL_WP_VERDIST",false),
	_param_rel_alt(this, "FOL_RELATIVE_ALT",false),
	_waypoint({0}),
	_inited(false)
	{}

Follow::~Follow()
{
}

void
Follow::on_inactive()
{
	if (_inited) {
		reset_follow_item();
	} else {
		/* triplet has cleared*/
		_inited = false;
	}

}

void
Follow::on_activation()
{
	reset_follow_item();
	_inited = true;
}

void
Follow::on_active()
{
	bool waypoint_updated = false;
	orb_check(_navigator->get_waypoint_sub(), &waypoint_updated);
	if (waypoint_updated) {
		orb_copy(ORB_ID(waypoint), _navigator->get_waypoint_sub(), &_waypoint);

                //Make sure that the position setpoint is valid
                if (!isfinite(_waypoint.lat) || !isfinite(_waypoint.lon) || !isfinite(_waypoint.alt)) {
                        return;
                }

		/* make sure param is up to date */
		updateParams();
		set_follow_item();
	}
}

void
Follow::reset_follow_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.valid = false;
	pos_sp_triplet->next.valid = false;
}

void
Follow::set_follow_item()
{
	bool update_xy = false;
	bool update_z = false;
	float dist_xy = -1.0f;
	float dist_z = -1.0f;

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	float altitude_amsl = _waypoint.alt + _param_rel_alt.get();

	if (pos_sp_triplet->current.valid) {
		get_distance_to_point_global_wgs84(_waypoint.lat, _waypoint.lon, altitude_amsl,
				pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
				pos_sp_triplet->current.alt, &dist_xy, &dist_z);

		if (dist_xy > _param_wp_hordist.get()) {
			update_xy = true;
		}

		if (dist_z > _param_wp_verdist.get()) {
			update_z = true;
		}

	} else {
		update_xy = true;
		update_z = true;
	}

	if ((!update_xy) && (!update_z)) {
		return;
	}

	mavlink_log_critical(_navigator->get_mavlink_fd(),"update follow item");

	/* set current position setpoint to previous */
	set_previous_pos_setpoint();

	/* set current position setpoint from waypoint */
	set_waypoint_to_position_setpoint(&_waypoint, pos_sp_triplet, update_xy, update_z);

	/* next item is unused */
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

void
Follow::set_waypoint_to_position_setpoint(const struct waypoint_s *waypoint, struct position_setpoint_triplet_s *pos_sp_triplet, bool update_xy, bool update_z)
{
	if (update_xy) {
		pos_sp_triplet->current.lat = waypoint->lat;
		pos_sp_triplet->current.lon = waypoint->lon;

	} else {
		pos_sp_triplet->current.lat = pos_sp_triplet->previous.lat;
		pos_sp_triplet->current.lon = pos_sp_triplet->previous.lon;
	}

	if (update_z) {
		pos_sp_triplet->current.alt = waypoint->alt + _param_rel_alt.get();

	}else {
		pos_sp_triplet->current.alt = pos_sp_triplet->previous.alt;
	}

	if (_param_yaw_mode.get() == FOLLOW_YAWMODE_FRONT_TO_WAYPOINT) {
		pos_sp_triplet->current.yaw = get_bearing_to_next_waypoint(
			_navigator->get_global_position()->lat,
			_navigator->get_global_position()->lon,
			_waypoint.lat, _waypoint.lon);
	} else {
		pos_sp_triplet->current.yaw = NAN;
	}

	pos_sp_triplet->current.valid = true;
}

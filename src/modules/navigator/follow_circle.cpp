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
 * @file follow_circle.cpp
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

#include <mavlink/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include "follow_circle.h"
#include "navigator.h"

FollowCircle::FollowCircle(Navigator *navigator, const char *name) :
	FollowCommon(navigator, name),
	_param_alt_en(this, "F_CIR_ALT_EN",false),
	_param_alt(this, "F_CIR_ALT",false)
{

}

FollowCircle::~FollowCircle() {

}

void
FollowCircle::set_follow_item(const struct waypoint_s *waypoint) {

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* make sure param is up to date */
	updateParams();

	switch (_follow_state) {
		case FOLLOW_STATE_INIT:
		{
			float rel_alt = _navigator->get_sensor_combined()->baro_alt_meter[0] - waypoint->alt;
			float alt_sp = math::constrain(_param_alt.get(), MIN_FOLLOW_ALT, MAX_FOLLOW_ALT);

			if (_param_alt.get() < 0.0f) {
				_vehicle_ref_alt = _navigator->get_global_position()->alt;
			} else {
				_vehicle_ref_alt = _navigator->get_global_position()->alt + alt_sp - rel_alt;
			}

			_waypoint_ref_alt = waypoint->alt;

			mavlink_log_info(_navigator->get_mavlink_fd(),"v_ref_alt %d, w ref alt %d",
				(int)_vehicle_ref_alt, (int)_waypoint_ref_alt);
		break;
		}

		case FOLLOW_STATE_FOLLOW:
		{
			pos_sp_triplet->current.valid = true;
			pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_CIRCLE;
			pos_sp_triplet->current.vx = waypoint->vel_n_m_s;
			pos_sp_triplet->current.vy = waypoint->vel_e_m_s;
			pos_sp_triplet->current.vz = waypoint->vel_d_m_s;
			pos_sp_triplet->current.lat = waypoint->lat;
			pos_sp_triplet->current.lon = waypoint->lon;
			pos_sp_triplet->current.yaw = NAN;

			if(_param_alt_en.get()) {
				pos_sp_triplet->current.alt = _vehicle_ref_alt + waypoint->alt - _waypoint_ref_alt;
			} else {
				pos_sp_triplet->current.alt = _vehicle_ref_alt;
			}

			_navigator->set_position_setpoint_triplet_updated();

		break;
		}

		default:
		break;
	}
}

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

#include <mavlink/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include "navigator.h"
#include "follow.h"

Follow::Follow(Navigator *navigator, const char *name) :
    MissionBlock(navigator, name),
    _param_rel_alt(this, "FOL_RELATIVE_ALT",false),
    _inited(false),
    _ref_alt(0.0f),
    _follow_state(FOLLOW_STATE_NONE),
    _waypoint_sp({})
{
    updateParams();
    on_inactive();
}

Follow::~Follow() {

}

void
Follow::on_inactive() {
    if (_inited) {
        reset_follow_item();
        _inited = false;
    }

    _follow_state = FOLLOW_STATE_NONE;
}

void
Follow::on_activation() {
    reset_follow_item();
    _inited = true;
}

void
Follow::on_active() {
    if (!_navigator->get_vstatus()->condition_landed) {

	struct waypoint_s* waypoint_sp = _navigator->get_waypoint_sp();
	if (is_valid_follow_item(waypoint_sp)) {
		if (_follow_state != FOLLOW_STATE_CLIMB || (_follow_state == FOLLOW_STATE_CLIMB && is_alt_reached())) {
			advance_follow();
			memcpy(&_waypoint_sp, waypoint_sp, sizeof(_waypoint_sp));
			set_follow_item(waypoint_sp);
            }
        }
    }
}

void
Follow::reset_follow_item() {
    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
    pos_sp_triplet->previous.valid = false;
    pos_sp_triplet->current.valid = false;
    pos_sp_triplet->next.valid = false;
}

bool
Follow::is_valid_follow_item(const struct waypoint_s *waypoint) {

    /* always true by default, waypoint is filterd in mc_pos_ctrl */
    bool result = (waypoint->timestamp != 0 &&
                    waypoint->timestamp != _waypoint_sp.timestamp);

    if (hrt_elapsed_time(&waypoint->timestamp) > 1000000)
	result = false;

    return result;
}

void
Follow::set_follow_item(const struct waypoint_s *waypoint) {

    struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

    /* make sure param is up to date */
    updateParams();

    switch (_follow_state) {
        case FOLLOW_STATE_CLIMB:
            {
		_ref_alt = waypoint->alt;
                float climb_alt = _navigator->get_home_position()->alt + _param_rel_alt.get();

                pos_sp_triplet->current.valid = true;
                pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
                pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
                pos_sp_triplet->current.alt = climb_alt;
                pos_sp_triplet->current.yaw = NAN;
                pos_sp_triplet->current.type= position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

                /* climb status, don't care of velocity of horizon */
                pos_sp_triplet->current.vx = 1.0f;
                pos_sp_triplet->current.vy = 1.0f;

                _navigator->set_position_setpoint_triplet_updated();

                mavlink_log_critical(_navigator->get_mavlink_fd(), "FOLLOW: climb to %d m (%d m above home)",
                        (int)(climb_alt),
                        (int)(climb_alt - _navigator->get_home_position()->alt));
                break;
            }
        case FOLLOW_STATE_FOLLOW:
            {
		float alt = _navigator->get_home_position()->alt + _param_rel_alt.get() + waypoint->alt - _ref_alt;

                pos_sp_triplet->current.valid = true;
                pos_sp_triplet->current.lat = waypoint->lat;
                pos_sp_triplet->current.lon = waypoint->lon;
                pos_sp_triplet->current.alt = alt;
                pos_sp_triplet->current.yaw = get_bearing_to_next_waypoint(
                        _navigator->get_global_position()->lat,
                        _navigator->get_global_position()->lon,
                        waypoint->lat, waypoint->lon);

                pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
                pos_sp_triplet->current.vx = waypoint->vel_n_m_s;
                pos_sp_triplet->current.vy = waypoint->vel_e_m_s;

                _navigator->set_position_setpoint_triplet_updated();

                break;
            }

        default:
            break;
    }
}

    bool
Follow::is_alt_reached()
{
    float alt_ref = _navigator->get_home_position()->alt + _param_rel_alt.get();
    float alt_now = _navigator->get_global_position()->alt;

    float alt = fabsf(alt_now - alt_ref);
    if (alt < 1.0f) {
        mavlink_log_critical(_navigator->get_mavlink_fd(), "FOLLOW: climb finished");
        return true;
    }

    return false;
}

    void
Follow::advance_follow()
{
    switch (_follow_state) {
        case FOLLOW_STATE_NONE:
            _follow_state = FOLLOW_STATE_CLIMB;
            break;

        case FOLLOW_STATE_CLIMB:
            _follow_state = FOLLOW_STATE_FOLLOW;
            break;

        default:
            break;
    }
}

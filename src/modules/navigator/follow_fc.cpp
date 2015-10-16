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

#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>

#include <dataman/dataman.h>

#include "navigator.h"
#include "follow_fc.h"

#define DELAY_SIGMA	0.01f

FollowFC::FollowFC(Navigator *navigator, const char *name) :
    NavigatorMode(navigator, name),
    _first_run(true),
    _should_run_item(false),
    _state_current(FCF_STATE_NONE),
    _item_current({0}),
    _item_next({0}),
    _target_waypoint({0}),
    _target_local_pos({0}),
    _ref_pos({0}),
    _ref_alt(0),
    _onboard_mission {},
    _onboard_mission_pub(-1),
    _fcf_item_reached(0),
    _mission_onboard_enabled_old(0),
	_have_set_mission_onboard(false),
    _param_bottom_alt(this, "FCF_BOTTOM_ALT", false),
    _param_top_alt(this, "FCF_TOP_ALT", false),
    _param_hori_dist(this, "FCF_HORI_DIST", false),
    _param_acceptance_radius(this, "FCF_ACC_RAD", false),
    _param_rate_x(this, "FCF_RATE_X", false),
    _param_rate_y(this, "FCF_RATE_Y", false),
    _param_rate_z(this, "FCF_RATE_Z", false),
    _param_set_direction(this, "FCF_DIR", false),
    _param_onboard_enabled(this, "MIS_ONBOARD_EN")

{
    /* load initial params */
    updateParams();
    /* initial reset */
    on_inactive();
}

FollowFC::~FollowFC() {
}

    void
FollowFC::on_inactive() {
    _state_current = FCF_STATE_NONE;
    _first_run = true;
    memset(&_target_local_pos, 0, sizeof(_target_local_pos));
    if (_have_set_mission_onboard)
    {
    	param_set(param_find("MIS_ONBOARD_EN"), &_mission_onboard_enabled_old);
		_have_set_mission_onboard = false;
	}
}

void
FollowFC::on_activation() {

	_mission_onboard_enabled_old = _param_onboard_enabled.get();
    _have_set_mission_onboard = true;
    int set_onboard_enabled = 1;
    param_set(param_find("MIS_ONBOARD_EN"), &set_onboard_enabled);

    if (_navigator->get_vstatus()->condition_landed) {
        return;
    }

    if (_state_current == FCF_STATE_NONE) {
        _state_current = FCF_STATE_GOTO;
    }

    update_ref();
    if (get_waypoint_of_target(&_target_waypoint)) {
        get_local_position_of_target(&_target_waypoint, &_target_local_pos);
    }
   if (_target_local_pos.position_valid) {
        /* get current item */
        set_item(&_item_current, _target_local_pos, _state_current);

        /* get next item */
        enum fcf_state_e next_state = _state_current;
        transit_next_state(&next_state);
        set_item(&_item_next, _target_local_pos, next_state);
        /* publish item to mission mode */
        update_item_to_mission(&_item_current, &_item_next, nullptr);
        _state_current = next_state;
    }
}

void
FollowFC::on_active() {
    if (_navigator->get_vstatus()->condition_landed) {
        return;
    }
    if (_navigator->get_mission_result()->finished) {
        if(!_fcf_item_reached)
        {
            update_ref();
            if (get_waypoint_of_target(&_target_waypoint)) {
                get_local_position_of_target(&_target_waypoint, &_target_local_pos);
            }

            if (_target_local_pos.position_valid) {
                /* get current item */
                transit_next_state(&_state_current);
                set_item(&_item_current, _target_local_pos, _state_current);

                /* get next item */
                enum fcf_state_e next_state = _state_current;
                transit_next_state(&next_state);
                set_item(&_item_next, _target_local_pos, next_state);
                update_item_to_mission(&_item_current, &_item_next, nullptr);
                _state_current = next_state;
            }
            _fcf_item_reached = true;
        }
    }
    else
    {
        _fcf_item_reached = false;
    }
}

void
FollowFC::start_item() {


    if (_should_run_item) {
        return;
    }
    pthread_attr_t follow_fc_attr;
    pthread_attr_init(&follow_fc_attr);
    struct sched_param param;
    (void)pthread_attr_getschedparam(&follow_fc_attr, &param);
    param.sched_priority = SCHED_PRIORITY_DEFAULT;
    (void)pthread_attr_setschedparam(&follow_fc_attr, &param);

    pthread_attr_setstacksize(&follow_fc_attr, 2100);
    pthread_t thread;
    pthread_create(&thread, &follow_fc_attr, FollowFC::item_thread, this);

    pthread_attr_destroy(&follow_fc_attr);

    _should_run_item = true;
}

void
FollowFC::stop_item() {
    _should_run_item = false;

    on_inactive();
}

bool
FollowFC::should_run_item() {
    return _should_run_item;
}

bool
FollowFC::is_first_run() {
    return _first_run;
}

void
FollowFC::set_first_run(bool flag) {
    _first_run = flag;
}

void* FollowFC::item_thread(void* arg) {

    FollowFC* fc = (FollowFC*)arg;
    while (fc->should_run_item()) {
        if (fc->is_first_run()) {
            fc->on_activation();
            fc->set_first_run(false);
        }

        fc->on_active();

        usleep(10*1000);
    }

    return nullptr;
}

bool
FollowFC::is_running_item() {
    if (_state_current != FCF_STATE_NONE) {
        return true;
    }
    return false;
}

    bool
FollowFC::is_item_reached(bool item_reached) {
    if(!item_reached)
    {
        float dist = -1.0f;
        float dist_xy = -1.0f;
        float dist_z = -1.0f;
        dist = mavlink_wpm_distance_to_point_local(
                _navigator->get_local_position()->x,
                _navigator->get_local_position()->y,
                _navigator->get_local_position()->z,
                _item_current.x, _item_current.y, _item_current.z, &dist_xy, &dist_z);
        if(dist <= _param_acceptance_radius.get())
        {
            return true;
        }
    }
    return false;
}

    void
FollowFC::update_ref() {
    map_projection_init(&_ref_pos,
            _navigator->get_local_position()->ref_lat,
            _navigator->get_local_position()->ref_lon);
    _ref_alt = _navigator->get_local_position()->ref_alt;
}

bool
FollowFC::get_waypoint_of_target(struct waypoint_s* target) {
    struct waypoint_s *waypoint = _navigator->get_waypoint_sp();
    if (waypoint->timestamp != 0 &&
        hrt_elapsed_time(&waypoint->timestamp) < 1*1000*1000) {
        memcpy(target, waypoint, sizeof(struct waypoint_s));
        return true;
    }

    return false;
}

void
FollowFC::get_local_position_of_target(struct waypoint_s* target, struct fcf_item_s* local_pos) {
    double lat_sp = target->lat;
    double lon_sp = target->lon;
    // double lat_sp = _navigator->get_home_position()->lat;
    // double lon_sp = _navigator->get_home_position()->lon;
    float alt_sp = target->alt;
    map_projection_project(&_ref_pos, lat_sp, lon_sp, &local_pos->x, &local_pos->y);
    local_pos->z = -(alt_sp - _ref_alt);
    local_pos->position_valid = true;
}

    void
FollowFC::set_item(struct fcf_item_s *item, const struct fcf_item_s target_local_pos, const enum fcf_state_e state) {
    /* make sure we have the latest params */
    updateParams();

    _navigator->set_can_loiter_at_sp(false);

    switch (state) {
        case FCF_STATE_GOTO:
            {
                float climb_alt = _param_bottom_alt.get();
                item->x = target_local_pos.x;
                item->y = target_local_pos.y;
                item->z = -climb_alt;
                item->yaw = NAN;

                mavlink_log_info(_navigator->get_mavlink_fd(),
                    "FCF: Move to,item->x is %.2lf item->y is %.2lf",
                    (double)item->x, (double)item->y);
                break;
            }

        case FCF_STATE_RIGHT_GO:
            {
                float top_alt = _param_top_alt.get();
                item->x = target_local_pos.x +
                    _param_hori_dist.get() * fabsf(cosf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
                item->y = target_local_pos.y +
                    _param_hori_dist.get() * fabsf(sinf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
                item->z = -top_alt;
                item->yaw = NAN;

                mavlink_log_info(_navigator->get_mavlink_fd(), "Right go item->x is %.2lf item->y is %.2lf",
                    (double)item->x, (double)item->y);
                break;
            }

        case FCF_STATE_RIGHT_BACK:
            {
                float bottom_alt = _param_bottom_alt.get();
                item->x = target_local_pos.x;
                item->y = target_local_pos.y;
                item->z = -bottom_alt;
                item->yaw = NAN;

                mavlink_log_info(_navigator->get_mavlink_fd(), "Right back item->x is %.2lf item->y is %.2lf",
                    (double)item->x, (double)item->y);
                break;
            }

        case FCF_STATE_LEFT_GO:
            {
                float top_alt = _param_top_alt.get();
                item->x = target_local_pos.x -
                    _param_hori_dist.get() * fabsf(cosf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
                item->y = target_local_pos.y -
                    _param_hori_dist.get() * fabsf(sinf(_wrap_pi(_param_set_direction.get() * M_DEG_TO_RAD_F)));
                item->z = -top_alt;
                item->yaw = NAN;

                mavlink_log_info(_navigator->get_mavlink_fd(), "Left go item->x is %.2lf item->y is %.2lf",
                    (double)item->x, (double)item->y);
                break;
            }
        case FCF_STATE_LEFT_BACK:
            {
                float bottom_alt = _param_bottom_alt.get();
                item->x = target_local_pos.x;
                item->y = target_local_pos.y;
                item->z = -bottom_alt;
                item->yaw = NAN;

                mavlink_log_info(_navigator->get_mavlink_fd(), "Left back item->x is %.2lf item->y is %.2lf",
                    (double)item->x, (double)item->y);
                break;
            }

        default:
            break;
    }
}

    void
FollowFC::transit_next_state(enum fcf_state_e *state) {
    switch (*state) {
        case FCF_STATE_GOTO:
            *state = FCF_STATE_RIGHT_GO;
            break;

        case FCF_STATE_RIGHT_GO:
            *state = FCF_STATE_RIGHT_BACK;
            break;

        case FCF_STATE_RIGHT_BACK:
            *state = FCF_STATE_LEFT_GO;
            break;

        case FCF_STATE_LEFT_GO:
            *state = FCF_STATE_LEFT_BACK;
            break;

        case FCF_STATE_LEFT_BACK:
            *state = FCF_STATE_RIGHT_GO;
            break;

        default:
            break;
    }
}

    void
FollowFC::update_item_to_mission(const struct fcf_item_s *item_current, const struct fcf_item_s *item_next, const struct fcf_item_s *item_gap)
{
    struct mission_item_s flight_vector_s {};
    flight_vector_s.nav_cmd = NAV_CMD_WAYPOINT;
    flight_vector_s.acceptance_radius = _param_acceptance_radius.get();
    flight_vector_s.autocontinue = true;
    flight_vector_s.altitude_is_relative = true;
    flight_vector_s.altitude = -item_current->z;
    map_projection_reproject(&_ref_pos,
        item_current->x, item_current->y,
        &flight_vector_s.lat, &flight_vector_s.lon);

    const ssize_t len = sizeof(struct mission_item_s);

    struct mission_item_s flight_vector_e {};
    flight_vector_e.nav_cmd = NAV_CMD_WAYPOINT;
    flight_vector_e.acceptance_radius = _param_acceptance_radius.get();
    flight_vector_e.autocontinue = true;
    flight_vector_e.altitude_is_relative = true;
    flight_vector_e.altitude = -item_next->z;
    map_projection_reproject(&_ref_pos,
        item_next->x, item_next->y,
        &flight_vector_e.lat, &flight_vector_e.lon);
    if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 0, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_s, len) != len) {
        warnx("ERROR: could not save onboard WP");
    }

    if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 1, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_e, len) != len) {\
        warnx("ERROR: could not save onboard WP");
    }
    if (item_gap != nullptr)
    {
        struct mission_item_s flight_vector_g {};
        flight_vector_g.nav_cmd = NAV_CMD_WAYPOINT;
        flight_vector_g.acceptance_radius = _param_acceptance_radius.get();
        flight_vector_g.autocontinue = true;
        flight_vector_g.altitude_is_relative = true;
        flight_vector_g.altitude = -item_gap->z;
        map_projection_reproject(&_ref_pos,
            item_gap->x, item_gap->y,
            &flight_vector_g.lat, &flight_vector_g.lon);

        if (dm_write(DM_KEY_WAYPOINTS_ONBOARD, 2, DM_PERSIST_IN_FLIGHT_RESET, &flight_vector_g, len) != len) {\
            warnx("ERROR: could not save onboard WP");
        }
        _onboard_mission.count = 3;
        _onboard_mission.current_seq = 0;
    }
    else
    {
        _onboard_mission.count = 2;
        _onboard_mission.current_seq = 0;

    }

    if (_onboard_mission_pub > 0) {
        orb_publish(ORB_ID(onboard_mission), _onboard_mission_pub, &_onboard_mission);
    } else {
        _onboard_mission_pub = orb_advertise(ORB_ID(onboard_mission), &_onboard_mission);
    }
}
    void
FollowFC::reset_item_reached(bool *item_reached)
{
    *item_reached = false;
}
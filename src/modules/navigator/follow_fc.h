/***************************************************************************
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
 * @file navigator_fcf.h
 * Helper class for FCF
 *
 * @author dong chen <dong.chen@ck-telecom.com>
 */

#ifndef NAVIGATOR_FCF_H
#define NAVIGATOR_FCF_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/topics/mission.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/waypoint.h>
#include <geo/geo.h>
#include "navigator_mode.h"
#include "mission_block.h"

class Navigator;

class FollowFC : public NavigatorMode
{
    public:
        FollowFC(Navigator *navigator, const char *name);

        ~FollowFC();

        virtual void on_inactive();

        virtual void on_activation();

        virtual void on_active();

        void start_item();

        void stop_item();

        bool should_run_item();

        bool is_first_run();

        void set_first_run(bool flag);
    private:
        enum fcf_state_e {
            FCF_STATE_NONE = 0,
            FCF_STATE_GOTO,
            FCF_STATE_RIGHT_GO,
            FCF_STATE_RIGHT_BACK,
            FCF_STATE_LEFT_GO,
            FCF_STATE_LEFT_BACK,
        };

        /**
         * Set the FCF item
         */
        struct fcf_item_s {
            float x;
            float y;
            float z;
            bool position_valid;
            float vx;
            float vy;
            float vz;
            bool velocity_valid;
            float yaw;
            bool yaw_valid;
            float yawspeed;
            bool yawspeed_valid;
        };

        static void*       item_thread(void* arg);

        bool		is_running_item();
        void		set_item(struct fcf_item_s *item, const struct fcf_item_s target_local_pos, const enum fcf_state_e state);
        void		transit_next_state(enum fcf_state_e *state);
        bool 		is_item_reached(bool item_reached);
        void 		update_ref();
        bool		get_waypoint_of_target(struct waypoint_s* target);
        void 		get_local_position_of_target(struct waypoint_s* target, struct fcf_item_s* local_pos);
        void 		update_item_to_mission(const struct fcf_item_s *item_current, const struct fcf_item_s *item_next);
        void        reset_item_reached(bool *item_reached);

        bool                                _first_run;
        bool                                _should_run_item;

        enum fcf_state_e				    _state_current;
        struct fcf_item_s 					_item_current;
        struct fcf_item_s					_item_next;

        struct waypoint_s 					_target_waypoint;
        struct fcf_item_s 					_target_local_pos;

        struct map_projection_reference_s 	_ref_pos;
        float 								_ref_alt;

        struct mission_s					_onboard_mission;
        orb_advert_t						_onboard_mission_pub;
        bool                                _fcf_item_reached;
        int 		_mission_onboard_enabled_old;                   /**< To store the previous onboard status */
		bool		_have_set_mission_onboard;						/**< If We have set the mission status */


        control::BlockParamFloat 			_param_bottom_alt;
        control::BlockParamFloat 			_param_top_alt;
        control::BlockParamFloat 			_param_hori_dist;
        control::BlockParamFloat 			_param_acceptance_radius;
        control::BlockParamFloat 			_param_rate_x;
        control::BlockParamFloat 			_param_rate_y;
        control::BlockParamFloat 			_param_rate_z;
        control::BlockParamFloat 			_param_set_direction;
        control::BlockParamInt              _param_onboard_enabled;  /**< if true: mission onboard enabled */

};

#endif

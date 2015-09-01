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

class FCF : public NavigatorMode
{
public:
	FCF(Navigator *navigator, const char *name);

	~FCF();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

private:
	/**
	 * Set the FCF item
	 */
	void		set_fcf_item();

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
	/**
	 * Move to next FCF item
	 */
	void		advance_fcf();
	void		fcf_item_to_position_setpoint(const struct fcf_item_s *item, struct position_setpoint_s *sp);
	bool 		is_fcf_item_reached();
	void 		reset_fcf_item_reached()
	{
		_fcf_item_reached = false;
	}
	void 		set_triplet_offboard_rate(float next, float current, float set_value, float *setting_value);
	void 		waypoint_update();
	void 		update_ref();
	void 		target_local_position_update();
	struct fcf_item_s* get_target_local_position() {return &_local_pos;}
	void 		follow_strategy();

	enum FCFState {
		FCF_STATE_NONE = 0,
		FCF_STATE_CLIMB,
		FCF_STATE_RIGHT_GO,
		FCF_STATE_RIGHT_BACK,
		FCF_STATE_LEFT_GO,
		FCF_STATE_LEFT_BACK,
	} _fcf_state;
	struct fcf_item_s 				_fcf_item;
	struct waypoint_s 				_waypoint;
	struct fcf_item_s 				_local_pos;
	struct map_projection_reference_s _ref_pos;
	float 							_ref_alt;
	bool 							_fcf_item_reached;
	control::BlockParamFloat _param_bottom_alt;
	control::BlockParamFloat _param_top_alt;
	control::BlockParamFloat _param_hori_dist;
	control::BlockParamFloat _param_acceptance_radius;
	control::BlockParamFloat _param_rate_x;
	control::BlockParamFloat _param_rate_y;
	control::BlockParamFloat _param_rate_z;
	control::BlockParamFloat _param_set_direction;
};

#endif

/*copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file circle.h
 *
 * Navigator mode to access missions
 *
 * @author liang.tang <liang.tang@ck-telecom.com>
 */

#ifndef NAVIGATOR_FOLLOW_H
#define NAVIGATOR_FOLLOW_H

#include <drivers/drv_hrt.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <dataman/dataman.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/waypoint.h>

#include "navigator_mode.h"
#include "mission_block.h"
#include "mission_feasibility_checker.h"

class Navigator;

class Follow : public MissionBlock
{
public:
	Follow(Navigator *navigator, const char *name);

	virtual ~Follow();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

	enum follow_yaw_mode {
		FOLLOW_YAWMODE_NONE = 0,
		FOLLOW_YAWMODE_FRONT_TO_WAYPOINT = 1
	};

private:
	void reset_follow_item();

	void set_follow_item();

	void set_waypoint_to_position_setpoint(const struct waypoint_s *waypoint, struct position_setpoint_triplet_s *pos_sp_triplet, bool update_xy, bool update_z);

	control::BlockParamInt _param_yaw_mode;
	control::BlockParamFloat _param_wp_hordist;
	control::BlockParamFloat _param_wp_verdist;
	control::BlockParamFloat _param_rel_alt;
	control::BlockParamInt _param_enable_alt_update;

	struct waypoint_s _waypoint;

	bool _inited;

#ifdef __WAYPOINT_DEBUG__
	void publish_waypoint_received(const struct waypoint_s& waypoint);
	void publish_waypoint_excuted(const struct waypoint_s& waypoint);
	orb_advert_t   _waypoint_received_report_pub;
	orb_advert_t   _waypoint_excuted_report_pub;
#endif
};

#endif

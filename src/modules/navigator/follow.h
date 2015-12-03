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
 * @file follow.h
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

#include "navigator_mode.h"
#include "mission_block.h"
#include <uORB/topics/waypoint.h>

class Navigator;

class Follow : public MissionBlock
{
public:
	Follow(Navigator *navigator, const char *name);

	virtual ~Follow();

	virtual void on_inactive();

	virtual void on_activation();

	virtual void on_active();

	virtual void reset_follow_item();

    virtual bool is_valid_follow_item(const struct waypoint_s *waypoint);

	virtual void set_follow_item(const struct waypoint_s *waypoint);

protected:
	control::BlockParamFloat _param_rel_alt;

	bool _inited;
	float _ref_alt;

private:
	enum FollowState {
		FOLLOW_STATE_NONE = 0,
		FOLLOW_STATE_CLIMB,
		FOLLOW_STATE_FOLLOW,
	} _follow_state;

	bool is_alt_reached();

	void advance_follow();

    struct waypoint_s _waypoint_sp;
};

#endif

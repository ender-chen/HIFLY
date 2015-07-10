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
 * @file navigator_takeoff.h
 * Helper class for TAKEOFF
 *
 * @author Pengyin.huang <pengyin.huang@ck-telecom.com>
 */

#ifndef NAVIGATOR_TAKEOFF_H
#define NAVIGATOR_TAKEOFF_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <uORB/topics/mission.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>

#include "navigator_mode.h"
#include "mission_block.h"

 class Navigator;

 class TAKEOFF : public MissionBlock
 {
 public:
 	TAKEOFF(Navigator *navigator, const char *name);

 	~TAKEOFF();

 	virtual void on_inactive();

 	virtual void on_activation();

 	virtual void on_active();

 private:
	/**
	* Set the TAKEOFF item
	*/
	void		set_takeoff_item();

	/**
	* Move to next TAKEOFF item
	*/
	void		advance_takeoff();

	enum TAKEOFFState {
	 	TAKEOFF_STATE_NONE = 0,
	 	TAKEOFF_STATE_CLIMB,
	 	TAKEOFF_STATE_LOITER,
	 	TAKEOFF_STATE_FINISHED,
	} _takeoff_state;


	control::BlockParamFloat _param_takeoff_alt;
	control::BlockParamFloat _param_takeoff_delay;
	};

#endif

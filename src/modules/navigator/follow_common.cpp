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
#include "follow_common.h"

FollowCommon::FollowCommon(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_follow_state(FOLLOW_STATE_NONE)
{
	updateParams();
	on_inactive();
}

FollowCommon::~FollowCommon() {

}

void
FollowCommon::on_inactive() {

}

void
FollowCommon::on_activation() {
	_follow_state = FOLLOW_STATE_NONE;
}

void
FollowCommon::on_active() {
	if (!_navigator->get_vstatus()->condition_landed) {
		struct follow_target_s *target = nullptr;
		target = _navigator->get_valid_target();
		if (target != nullptr) {
			advance_follow();
			set_follow_item(target);
		}
	}
}

void
FollowCommon::set_follow_item(const struct follow_target_s *target) {

}

void
FollowCommon::advance_follow()
{
	switch (_follow_state) {
		case FOLLOW_STATE_NONE:
			_follow_state = FOLLOW_STATE_INIT;
		break;

		case FOLLOW_STATE_INIT:
			_follow_state = FOLLOW_STATE_FOLLOW;
		break;

		default:
		break;
	}
}

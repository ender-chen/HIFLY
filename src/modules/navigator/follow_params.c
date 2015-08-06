/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *
 * opyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file follow_params.c
 *
 * Parameters for follow
 *
 * @author Thomas Gubler <liang.tang@ck-telecom.com>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * Follow parameters, accessible via MAVLink
 */

/**
 * yaw setpoint mode.
 *
 * 0: NAN
 * 1: Maintain a yaw heading pointing towards the next waypoint.
 * @min 0
 * @max 1
 * @group Follow
 */
PARAM_DEFINE_INT32(FOL_YAW_MODE, 1);

/**
 * Waypoint vertical distance.
 *
 * Set to > 0 to limit vertical distance, if the distance of previous waypoint and current waypoint 
 * is large than FOL_WP_VERDIST, update new waypoint.
 *
 * @unit meters
 * @group Follow
 */
PARAM_DEFINE_FLOAT(FOL_WP_VERDIST, 3.0f);

/**
 * Follow alt.
 *
 * relative altitude
 *
 * @unit meters
 * @group Follow
 */
PARAM_DEFINE_FLOAT(FOL_RELATIVE_ALT, 7.0f);

/**
 * Follow enable alt update.
 * enable flag
 *
 * 0: keep alt:home alt + relative alt
 * 1: update to current waypoint's alt + relative alt
 * @min 0
 * @max 1
 * @group Follow
 */
PARAM_DEFINE_INT32(FOL_ENABLE_ALT_UPDATE, 0);

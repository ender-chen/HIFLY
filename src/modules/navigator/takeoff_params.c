/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file takeoff_params.c
 *
 * Parameters for TAKEOFF
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * TAKEOFF parameters, accessible via MAVLink
 */

/**
 * Loiter radius after TAKEOFF (FW only)
 *
 * Default value of loiter radius after TAKEOFF (fixedwing only).
 *
 * @unit meters
 * @min 0.0
 * @group TAKEOFF
 */
PARAM_DEFINE_FLOAT(TAKEOFF_RAD, 2.0f);

/**
 * TAKEOFF altitude
 *
 * Altitude to fly back in TAKEOFF in meters
 *
 * @unit meters
 * @min 0
 * @max 1
 * @group TAKEOFF
 */
PARAM_DEFINE_FLOAT(TAKEOFF_ALT, 5);

/**
 * Takeoff delay
 *
 * Delay after takeoff before switch to other mode.
 *
 * @unit seconds
 * @min -1
 * @max 300
 * @group Return To Land
 */
PARAM_DEFINE_FLOAT(TAKEOFF_DELAY, 2.0f);

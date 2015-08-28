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
 * @file fcf_params.c
 *
 * Parameters for FCF
 *
 * @author dong chen <dong.chen@ck-telecom>
 */

#include <nuttx/config.h>

#include <systemlib/param/param.h>

/*
 * FCF parameters, accessible via MAVLink
 */

/**
 * Far close follow bottom altitude
 */
PARAM_DEFINE_FLOAT(FCF_BOTTOM_ALT, 5);
/**
 * FCF top altitude
 */
PARAM_DEFINE_FLOAT(FCF_TOP_ALT, 7);

/**
 * FCF horizon distance
 */
PARAM_DEFINE_FLOAT(FCF_HORI_DIST, 6);

/**
 *FCF acceptace radius
 */
PARAM_DEFINE_FLOAT(FCF_ACC_RAD, 1.0f);

/**
 *FCF move rate in x direction
 */
PARAM_DEFINE_FLOAT(FCF_RATE_X, 1);

/**
 *FCF move rate in y direction
 */
PARAM_DEFINE_FLOAT(FCF_RATE_Y, 1);

/**
 *FCF move rate in z direction
 */
PARAM_DEFINE_FLOAT(FCF_RATE_Z, 1);

/**
 *FCF move offset position in z direction
 */
PARAM_DEFINE_FLOAT(FCF_OFFSET_POS, 0.5);


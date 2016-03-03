/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file accelerometer_calibration.cpp
 *
 * Implementation of accelerometer calibration.
 *
 */

// FIXME: Can some of these headers move out with detect_ move?

#include "accelerometer_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_posix.h>
#include <px4_time.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <fcntl.h>
//#include <sys/prctl.h>
#include <math.h>
#include <poll.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <geo/geo.h>
#include <conversion/rotation.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <mavlink/mavlink_log.h>
#include <uORB/topics/vehicle_attitude.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "accel";

static int32_t device_id[max_accel_sens];
static int32_t device_id_primary = 0;

calibrate_return do_accel_calibration_measurements(int mavlink_fd, float (&accel_offs)[max_accel_sens][3], unsigned *active_sensors);
calibrate_return read_accelerometer_avg(int (&subs)[max_accel_sens], float (&accel_avg)[max_accel_sens][3], unsigned samples_num);

int do_accel_calibration(int mavlink_fd)
{
	int fd;

	mavlink_and_console_log_info(mavlink_fd, CAL_QGC_STARTED_MSG, sensor_name);

	struct accel_scale accel_scale = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	int res = OK;

	char str[30];

	/* reset all sensors */
	for (unsigned s = 0; s < max_accel_sens; s++) {
		sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);
		/* reset all offsets to zero and all scales to one */
		fd = px4_open(str, 0);

		if (fd < 0) {
			continue;
		}

		device_id[s] = px4_ioctl(fd, DEVIOCGDEVICEID, 0);

		res = px4_ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&accel_scale);
		px4_close(fd);

		if (res != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_RESET_CAL_MSG, s);
		}
	}

	float accel_offs[max_accel_sens][3];
	unsigned active_sensors;

	/* measure and calculate offsets & scales */
	if (res == OK) {
		calibrate_return cal_return = do_accel_calibration_measurements(mavlink_fd, accel_offs, &active_sensors);
		if (cal_return == calibrate_return_cancelled) {
			// Cancel message already displayed, nothing left to do
			return ERROR;
		} else if (cal_return == calibrate_return_ok) {
			res = OK;
		} else {
			res = ERROR;
		}
	}

	if (res != OK) {
		if (active_sensors == 0) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_SENSOR_MSG);
		}
		return ERROR;
	}

	/* measurements completed successfully, rotate calibration values */
	param_t board_rotation_h = param_find("SENS_BOARD_ROT");
	int32_t board_rotation_int;
	param_get(board_rotation_h, &(board_rotation_int));
	enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;
	math::Matrix<3, 3> board_rotation;
	get_rot_matrix(board_rotation_id, &board_rotation);
	math::Matrix<3, 3> board_rotation_t = board_rotation.transposed();

	for (unsigned i = 0; i < active_sensors; i++) {

		/* handle individual sensors, one by one */
		math::Vector<3> accel_offs_vec(accel_offs[i]);
		math::Vector<3> accel_offs_rotated = board_rotation_t * accel_offs_vec;

		accel_scale.x_offset = accel_offs_rotated(0);
		accel_scale.x_scale = 1;
		accel_scale.y_offset = accel_offs_rotated(1);
		accel_scale.y_scale = 1;
		accel_scale.z_offset = accel_offs_rotated(2);
		accel_scale.z_scale = 1;
		
		bool failed = false;

		failed = failed || (OK != param_set_no_notification(param_find("CAL_ACC_PRIME"), &(device_id_primary)));

		/* set parameters */
		(void)sprintf(str, "CAL_ACC%u_XOFF", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(accel_scale.x_offset)));
		(void)sprintf(str, "CAL_ACC%u_YOFF", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(accel_scale.y_offset)));
		(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(accel_scale.z_offset)));
		(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(accel_scale.x_scale)));
		(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(accel_scale.y_scale)));
		(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(accel_scale.z_scale)));
		(void)sprintf(str, "CAL_ACC%u_ID", i);
		failed |= (OK != param_set_no_notification(param_find(str), &(device_id[i])));
		
		if (failed) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_SET_PARAMS_MSG, i);
			return ERROR;
		}

		sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, i);
		fd = px4_open(str, 0);

		if (fd < 0) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, "sensor does not exist");
			res = ERROR;
		} else {
			res = px4_ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&accel_scale);
			px4_close(fd);
		}

		if (res != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_APPLY_CAL_MSG, i);
		}
	}

	if (res == OK) {
		/* auto-save to EEPROM */
		res = param_save_default();

		if (res != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_SAVE_PARAMS_MSG);
		}

		/* if there is a any preflight-check system response, let the barrage of messages through */
		usleep(200000);

		mavlink_and_console_log_info(mavlink_fd, CAL_QGC_DONE_MSG, sensor_name);

	} else {
		mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, sensor_name);
	}
	
	/* give this message enough time to propagate */
	usleep(600000);

	return res;
}

calibrate_return do_accel_calibration_measurements(int mavlink_fd, float (&accel_offs)[max_accel_sens][3], unsigned *active_sensors)
{
	calibrate_return result = calibrate_return_ok;
	const unsigned samples_num = 3000;
	int     subs[max_accel_sens];
	float       accel_ref[max_accel_sens][3];
	*active_sensors = 0;
	uint64_t timestamps[max_accel_sens];
	/*Read the average value of the accelerometer*/
	for (unsigned i = 0; i < max_accel_sens; i++) {
		subs[i] = orb_subscribe_multi(ORB_ID(sensor_accel), i);
		if (subs[i] < 0) {
			result = calibrate_return_error;
			break;
		}
		
		/* store initial timestamp - used to infer which sensors are active */
		struct accel_report arp = {};
		(void)orb_copy(ORB_ID(sensor_accel), subs[i], &arp);
		timestamps[i] = arp.timestamp;
	}

	if (result == calibrate_return_ok) 
	{
		int cancel_sub = calibrate_cancel_subscribe();
		read_accelerometer_avg(subs, accel_ref, samples_num);
		mavlink_and_console_log_info(mavlink_fd, "[cal]result: [%8.4f %8.4f %8.4f]", (double)accel_ref[0][0]
																				, (double)accel_ref[0][1]
																				, (double)accel_ref[0][2]);
		mavlink_and_console_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, 100);
		calibrate_cancel_unsubscribe(cancel_sub);
	}

	/* close all subscriptions */
	for (unsigned i = 0; i < max_accel_sens; i++) {
		if (subs[i] >= 0) {
			/* figure out which sensors were active */
			struct accel_report arp = {};
			(void)orb_copy(ORB_ID(sensor_accel), subs[i], &arp);
			if (arp.timestamp != 0 && timestamps[i] != arp.timestamp) {
				(*active_sensors)++;
			}
			close(subs[i]);
		}
	}
	/*Determine the direction of gravity*/
	int	accel_max = 0;
	for(int i = 1; i< 3; i++)
	{
		if(fabs(accel_ref[0][i]) > 5)
		{
			accel_max = i;	
		}
	}
	/*Determine offset*/
	for(int i = 0; i < 3; i++)
	{
		accel_offs[i][0] = accel_ref[i][0];
		accel_offs[i][1] = accel_ref[i][1];
		accel_offs[i][2] = accel_ref[i][2];

		if(accel_offs[i][accel_max] > 0)
		{
			accel_offs[i][accel_max] += -CONSTANTS_ONE_G;  
		}
		else
		{
			accel_offs[i][accel_max] += CONSTANTS_ONE_G;
		}
	}

	return result;
}

/*
 * Read specified number of accelerometer samples, calculate average and dispersion.
 */
calibrate_return read_accelerometer_avg(int (&subs)[max_accel_sens], float (&accel_avg)[max_accel_sens][3], unsigned samples_num)
{
	/* get total sensor board rotation matrix */
	param_t board_rotation_h = param_find("SENS_BOARD_ROT");
	param_t board_offset_x = param_find("SENS_BOARD_X_OFF");
	param_t board_offset_y = param_find("SENS_BOARD_Y_OFF");
	param_t board_offset_z = param_find("SENS_BOARD_Z_OFF");

	float board_offset[3];
	param_get(board_offset_x, &board_offset[0]);
	param_get(board_offset_y, &board_offset[1]);
	param_get(board_offset_z, &board_offset[2]);

	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * board_offset[0],
			M_DEG_TO_RAD_F * board_offset[1],
			M_DEG_TO_RAD_F * board_offset[2]);

	int32_t board_rotation_int;
	param_get(board_rotation_h, &(board_rotation_int));
	enum Rotation board_rotation_id = (enum Rotation)board_rotation_int;
	math::Matrix<3, 3> board_rotation;
	get_rot_matrix(board_rotation_id, &board_rotation);

	/* combine board rotation with offset rotation */
	board_rotation = board_rotation_offset * board_rotation;

	px4_pollfd_struct_t fds[max_accel_sens];

	for (unsigned i = 0; i < max_accel_sens; i++) {
		fds[i].fd = subs[i];
		fds[i].events = POLLIN;
	}

	unsigned counts[max_accel_sens] = { 0 };
	float accel_sum[max_accel_sens][3];
	memset(accel_sum, 0, sizeof(accel_sum));

	unsigned errcount = 0;

	/* use the first sensor to pace the readout, but do per-sensor counts */
	while (counts[0] < samples_num) {
		int poll_ret = px4_poll(&fds[0], max_accel_sens, 1000);

		if (poll_ret > 0) {

			for (unsigned s = 0; s < max_accel_sens; s++) {
				bool changed;
				orb_check(subs[s], &changed);

				if (changed) {

					struct accel_report arp;
					orb_copy(ORB_ID(sensor_accel), subs[s], &arp);

					accel_sum[s][0] += arp.x;
					accel_sum[s][1] += arp.y;
					accel_sum[s][2] += arp.z;

					counts[s]++;
				}
			}

		} else {
			errcount++;
			continue;
		}

		if (errcount > samples_num / 10) {
			return calibrate_return_error;
		}
	}

	// rotate sensor measurements from body frame into sensor frame using board rotation matrix
	for (unsigned i = 0; i < max_accel_sens; i++) {
		math::Vector<3> accel_sum_vec(&accel_sum[i][0]);
		accel_sum_vec = board_rotation * accel_sum_vec;
		memcpy(&accel_sum[i][0], &accel_sum_vec.data[0], sizeof(accel_sum[i]));
	}

	for (unsigned s = 0; s < max_accel_sens; s++) {
		for (unsigned i = 0; i < 3; i++) {
			accel_avg[s][i] = accel_sum[s][i] / counts[s];
		}
	}

	return calibrate_return_ok;
}

int do_level_calibration(int mavlink_fd) {
	const unsigned cal_time = 5;
	const unsigned cal_hz = 100;
	unsigned settle_time = 30;

	bool success = false;
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));

	mavlink_and_console_log_info(mavlink_fd, CAL_QGC_STARTED_MSG, "level");

	param_t roll_offset_handle = param_find("SENS_BOARD_X_OFF");
	param_t pitch_offset_handle = param_find("SENS_BOARD_Y_OFF");
	param_t board_rot_handle = param_find("SENS_BOARD_ROT");

	// save old values if calibration fails
	float roll_offset_current;
	float pitch_offset_current;
	int32_t board_rot_current = 0;
	param_get(roll_offset_handle, &roll_offset_current);
	param_get(pitch_offset_handle, &pitch_offset_current);
	param_get(board_rot_handle, &board_rot_current);

	// give attitude some time to settle if there have been changes to the board rotation parameters
	if (board_rot_current == 0 && fabsf(roll_offset_current) < FLT_EPSILON && fabsf(pitch_offset_current) < FLT_EPSILON ) {
		settle_time = 0;
	}

	float zero = 0.0f;
	param_set(roll_offset_handle, &zero);
	param_set(pitch_offset_handle, &zero);

	px4_pollfd_struct_t fds[1];

	fds[0].fd = att_sub;
	fds[0].events = POLLIN;

	float roll_mean = 0.0f;
	float pitch_mean = 0.0f;
	unsigned counter = 0;

	// sleep for some time
	hrt_abstime start = hrt_absolute_time();
	while(hrt_elapsed_time(&start) < settle_time * 1000000) {
		mavlink_and_console_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, (int)(90*hrt_elapsed_time(&start)/1e6f/(float)settle_time));
		sleep(settle_time / 10);
	}

	start = hrt_absolute_time();
	// average attitude for 5 seconds
	while(hrt_elapsed_time(&start) < cal_time * 1000000) {
		int pollret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		if (pollret <= 0) {
			// attitude estimator is not running
			mavlink_and_console_log_critical(mavlink_fd, "attitude estimator not running - check system boot");
			mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, "level");
			goto out;
		}

		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		roll_mean += att.roll;
		pitch_mean += att.pitch;
		counter++;
	}

	mavlink_and_console_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, 100);

	if (counter > (cal_time * cal_hz / 2 )) {
		roll_mean /= counter;
		pitch_mean /= counter;
	} else {
		mavlink_and_console_log_info(mavlink_fd, "not enough measurements taken");
		success = false;
		goto out;
	}

	if (fabsf(roll_mean) > 0.8f ) {
		mavlink_and_console_log_critical(mavlink_fd, "excess roll angle");
	} else if (fabsf(pitch_mean) > 0.8f ) {
		mavlink_and_console_log_critical(mavlink_fd, "excess pitch angle");
	} else {
		roll_mean *= (float)M_RAD_TO_DEG;
		pitch_mean *= (float)M_RAD_TO_DEG;
		param_set(roll_offset_handle, &roll_mean);
		param_set(pitch_offset_handle, &pitch_mean);
		success = true;
	}

out:
	if (success) {
		mavlink_and_console_log_info(mavlink_fd, CAL_QGC_DONE_MSG, "level");
		return 0;
	} else {
		// set old parameters
		param_set(roll_offset_handle, &roll_offset_current);
		param_set(pitch_offset_handle, &pitch_offset_current);
		mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, "level");
		return 1;
	}
}

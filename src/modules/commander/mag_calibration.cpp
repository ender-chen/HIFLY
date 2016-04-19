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
 * @file mag_calibration.cpp
 *
 * Magnetometer calibration routine
 */

#include "mag_calibration.h"
#include "commander_helper.h"
#include "calibration_routines.h"
#include "calibration_messages.h"

#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <cmath>
#include <fcntl.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <uORB/topics/sensor_combined.h>
#include <drivers/drv_mag.h>
#include <mavlink/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

static const char *sensor_name = "mag";
static constexpr unsigned max_mags = 3;
static constexpr float mag_sphere_radius = 0.2f;
static constexpr unsigned int calibration_sides = 2;			///< The total number of sides
static constexpr unsigned int calibration_total_points = 80;		///< The total points per magnetometer
static constexpr unsigned int calibraton_duration_seconds = 10; 	///< The total duration the routine is allowed to take

static constexpr float MAG_MAX_OFFSET_LEN = 0.9f;	///< The maximum measurement range is ~1.4 Ga, the earth field is ~0.6 Ga, so an offset larger than ~0.8-0.6 Ga means the mag will saturate in some directions.
static constexpr float GYRO_INTEGRAL_RAD = 13.0f;

int32_t	device_ids[max_mags];
int device_prio_max = 0;
int32_t device_id_primary = 0;

calibrate_return mag_calibrate_all(int mavlink_fd, int32_t (&device_ids)[max_mags]);

/// Data passed to calibration worker routine
typedef struct  {
	int		mavlink_fd;
	unsigned	done_count;
	int		sub_mag[max_mags];
	unsigned int	calibration_points_perside;
	unsigned int	calibration_interval_perside_seconds;
	uint64_t	calibration_interval_perside_useconds;
	unsigned int	calibration_counter_total[max_mags];
	bool		side_data_collected[detect_orientation_side_count];
	float*		x[max_mags];
	float*		y[max_mags];
	float*		z[max_mags];
} mag_worker_data_t;


int do_mag_calibration(int mavlink_fd)
{
	mavlink_and_console_log_info(mavlink_fd, CAL_QGC_STARTED_MSG, sensor_name);

	struct mag_scale mscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	int result = OK;
	
	// Determine which mags are available and reset each

	char str[30];

	for (size_t i=0; i<max_mags; i++) {
		device_ids[i] = 0; // signals no mag
	}
	
	for (unsigned cur_mag = 0; cur_mag < max_mags; cur_mag++) {
		// Reset mag id to mag not available
		(void)sprintf(str, "CAL_MAG%u_ID", cur_mag);
		result = param_set_no_notification(param_find(str), &(device_ids[cur_mag]));
		if (result != OK) {
			mavlink_and_console_log_info(mavlink_fd, "[cal] Unable to reset CAL_MAG%u_ID", cur_mag);
			break;
		}

		// Attempt to open mag
		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
		int fd = px4_open(str, O_RDONLY);
		if (fd < 0) {
			continue;
		}

		// Get device id for this mag
		device_ids[cur_mag] = px4_ioctl(fd, DEVIOCGDEVICEID, 0);

		// Reset mag scale
		result = px4_ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale_null);

		if (result != OK) {
			mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_RESET_CAL_MSG, cur_mag);
		}

		/* calibrate range */
		if (result == OK) {
			result = px4_ioctl(fd, MAGIOCCALIBRATE, fd);

			if (result != OK) {
				mavlink_and_console_log_info(mavlink_fd, "[cal] Skipped scale calibration, sensor %u", cur_mag);
				/* this is non-fatal - mark it accordingly */
				result = OK;
			}
		}

		px4_close(fd);
	}

	// Calibrate all mags at the same time
	if (result == OK) {
		switch (mag_calibrate_all(mavlink_fd, device_ids)) {
			case calibrate_return_cancelled:
				// Cancel message already displayed, we're done here
				result = ERROR;
				break;
				
			case calibrate_return_ok:
				/* auto-save to EEPROM */
				result = param_save_default();

				/* if there is a any preflight-check system response, let the barrage of messages through */
				usleep(200000);

				if (result == OK) {
					mavlink_and_console_log_info(mavlink_fd, CAL_QGC_PROGRESS_MSG, 100);
					mavlink_and_console_log_info(mavlink_fd, CAL_QGC_DONE_MSG, sensor_name);
					break;
				} else {
					mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_SAVE_PARAMS_MSG);
				}
				// Fall through
				
			default:
				mavlink_and_console_log_critical(mavlink_fd, CAL_QGC_FAILED_MSG, sensor_name);
				break;
		}
	}

	/* give this message enough time to propagate */
	usleep(600000);
	
	return result;
}

static bool reject_sample(float sx, float sy, float sz, float x[], float y[], float z[], unsigned count, unsigned max_count)
{
	float min_sample_dist = fabsf(5.4f * mag_sphere_radius / sqrtf(max_count)) / 3.0f;

	for (size_t i = 0; i < count; i++) {
		float dx = sx - x[i];
		float dy = sy - y[i];
		float dz = sz - z[i];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);

		if (dist < min_sample_dist) {
			return true;
		}
	}

	return false;
}

static unsigned progress_percentage(mag_worker_data_t* worker_data) {
	return 100 * ((float)worker_data->done_count) / calibration_sides;
}

static calibrate_return mag_calibration_worker(fist_orientation_axis* orient_axis, int cancel_sub, void* data)
{
	calibrate_return result = calibrate_return_ok;
	
	unsigned int calibration_counter_side;

	mag_worker_data_t* worker_data = (mag_worker_data_t*)(data);
	
	/*
	 * Detect if the system is rotating.
	 *
	 * We're detecting this as a general rotation on any axis, not necessary on the one we
	 * asked the user for. This is because we really just need two roughly orthogonal axes
	 * for a good result, so we're not constraining the user more than we have to.
	 */

	hrt_abstime last_gyro = 0;
	float gyro_x_integral = 0.0f;
	float gyro_y_integral = 0.0f;
	float gyro_z_integral = 0.0f;


	int sub_gyro = orb_subscribe(ORB_ID(sensor_gyro));

	
	unsigned poll_errcount = 0;
	
	calibration_counter_side = 0;
	
	while (calibration_counter_side < worker_data->calibration_points_perside) {
		
		if (calibrate_cancel_check(worker_data->mavlink_fd, cancel_sub)) {
			result = calibrate_return_cancelled;
			break;
		}

		if (*orient_axis == ORIENTATION_AXIS_NO)
		{
			if(fabs(gyro_x_integral) > (double)GYRO_INTEGRAL_RAD) {
				*orient_axis = ORIENTATION_AXIS_X;
				break;
			} else if (fabs(gyro_y_integral) > (double)GYRO_INTEGRAL_RAD) {
				*orient_axis = ORIENTATION_AXIS_Y;
				break;
			} else if (fabs(gyro_z_integral) > (double)GYRO_INTEGRAL_RAD) {
				*orient_axis = ORIENTATION_AXIS_Z;
				break;
			}
		} else if (*orient_axis == ORIENTATION_AXIS_X) {
			if (fabs(gyro_y_integral) > (double)GYRO_INTEGRAL_RAD) {
				break;
			} else if (fabs(gyro_z_integral) > (double)GYRO_INTEGRAL_RAD) {
				break;
			}
		} else if (*orient_axis == ORIENTATION_AXIS_Y) {
			if (fabs(gyro_x_integral) > (double)GYRO_INTEGRAL_RAD) {
				break;
			} else if(fabs(gyro_z_integral) > (double)GYRO_INTEGRAL_RAD) {
				break;
			}
		} else if (*orient_axis == ORIENTATION_AXIS_Z) {
			if (fabs(gyro_x_integral) > (double)GYRO_INTEGRAL_RAD) {
				break;
			} else if (fabs(gyro_y_integral) > (double)GYRO_INTEGRAL_RAD) {
				break;
			}
		}
		
		// Wait clocking for new data on all mags
		px4_pollfd_struct_t fds[max_mags];
		size_t fd_count = 0;
		for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (worker_data->sub_mag[cur_mag] >= 0) {
				fds[fd_count].fd = worker_data->sub_mag[cur_mag];
				fds[fd_count].events = POLLIN;
				fd_count++;
			}
		}
		/* Wait clocking for new data on all gyro */
		px4_pollfd_struct_t fds_gyro[1];
		fds_gyro[0].fd = sub_gyro;
		fds_gyro[0].events = POLLIN;
		size_t fd_count_gyro = 1;

		int poll_ret = px4_poll(fds_gyro, fd_count_gyro, 1000);

		if (poll_ret > 0) {
			poll_ret = px4_poll(fds, fd_count, 1000);

			if(poll_ret > 0) {
				struct gyro_report gyro;
				orb_copy(ORB_ID(sensor_gyro), sub_gyro, &gyro);

				int prev_count[max_mags];
				float last_gyro_x_integral, last_gyro_y_integral, last_gyro_z_integral;
				bool rejected = false;

				for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
					prev_count[cur_mag] = worker_data->calibration_counter_total[cur_mag];
					last_gyro_x_integral = gyro_x_integral;
					last_gyro_y_integral = gyro_y_integral;
					last_gyro_z_integral = gyro_z_integral;

					if (worker_data->sub_mag[cur_mag] >= 0) {
						struct mag_report mag;

						orb_copy(ORB_ID(sensor_mag), worker_data->sub_mag[cur_mag], &mag);

						if (last_gyro > 0) {
							float delta_t = (gyro.timestamp - last_gyro) / 1e6f;
							gyro_x_integral += gyro.x * delta_t;
							gyro_y_integral += gyro.y * delta_t;
							gyro_z_integral += gyro.z * delta_t;
						}
						last_gyro = gyro.timestamp;

						// Check if this measurement is good to go in
						rejected = rejected || reject_sample(mag.x, mag.y, mag.z,
									worker_data->x[cur_mag], worker_data->y[cur_mag], worker_data->z[cur_mag],
									worker_data->calibration_counter_total[cur_mag],
									calibration_sides * worker_data->calibration_points_perside);

						worker_data->x[cur_mag][worker_data->calibration_counter_total[cur_mag]] = mag.x;
						worker_data->y[cur_mag][worker_data->calibration_counter_total[cur_mag]] = mag.y;
						worker_data->z[cur_mag][worker_data->calibration_counter_total[cur_mag]] = mag.z;
						worker_data->calibration_counter_total[cur_mag]++;
					}
				}

				// Keep calibration of all mags in lockstep
				if (rejected) {
					// Reset counts, since one of the mags rejected the measurement
					for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {
						worker_data->calibration_counter_total[cur_mag] = prev_count[cur_mag];
						gyro_x_integral = last_gyro_x_integral;
						gyro_y_integral = last_gyro_y_integral;
						gyro_z_integral = last_gyro_z_integral;
					}
				} else {
					calibration_counter_side++;
				} 
			}else {
				poll_errcount++;
			}
		}

		if (poll_errcount > worker_data->calibration_points_perside * 3) {
			result = calibrate_return_error;
			mavlink_and_console_log_info(worker_data->mavlink_fd, CAL_ERROR_SENSOR_MSG);
			break;
		}
	}
	px4_close(sub_gyro);
	
	if (result == calibrate_return_ok) {
		
		worker_data->done_count++;
		mavlink_and_console_log_info(worker_data->mavlink_fd, CAL_QGC_PROGRESS_MSG, progress_percentage(worker_data));
	}
	
	return result;
}

calibrate_return mag_calibrate_all(int mavlink_fd, int32_t (&device_ids)[max_mags])
{
	calibrate_return result = calibrate_return_ok;

	mag_worker_data_t worker_data;
	
	worker_data.mavlink_fd = mavlink_fd;
	worker_data.done_count = 0;
	worker_data.calibration_points_perside = calibration_total_points / calibration_sides;
	worker_data.calibration_interval_perside_seconds = calibraton_duration_seconds / calibration_sides;
	worker_data.calibration_interval_perside_useconds = worker_data.calibration_interval_perside_seconds * 1000 * 1000;

	// Collect: Right-side up, Left Side, Nose down
	worker_data.side_data_collected[DETECT_ORIENTATION_RIGHTSIDE_UP] = false;
	worker_data.side_data_collected[DETECT_ORIENTATION_NOSE_DOWN] = false;
	
	for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
		// Initialize to no subscription
		worker_data.sub_mag[cur_mag] = -1;
		
		// Initialize to no memory allocated
		worker_data.x[cur_mag] = NULL;
		worker_data.y[cur_mag] = NULL;
		worker_data.z[cur_mag] = NULL;
		worker_data.calibration_counter_total[cur_mag] = 0;
	}

	const unsigned int calibration_points_maxcount = calibration_sides * worker_data.calibration_points_perside;
	
	char str[30];
	
	for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
		worker_data.x[cur_mag] = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
		worker_data.y[cur_mag] = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
		worker_data.z[cur_mag] = reinterpret_cast<float *>(malloc(sizeof(float) * calibration_points_maxcount));
		if (worker_data.x[cur_mag] == NULL || worker_data.y[cur_mag] == NULL || worker_data.z[cur_mag] == NULL) {
			mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: out of memory");
			result = calibrate_return_error;
		}
	}

	
	// Setup subscriptions to mag sensors
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available
				worker_data.sub_mag[cur_mag] = orb_subscribe_multi(ORB_ID(sensor_mag), cur_mag);
				if (worker_data.sub_mag[cur_mag] < 0) {
					mavlink_and_console_log_critical(mavlink_fd, "[cal] Mag #%u not found, abort", cur_mag);
					result = calibrate_return_error;
					break;
				}

				// Get priority
				int32_t prio;
				orb_priority(worker_data.sub_mag[cur_mag], &prio);

				if (prio > device_prio_max) {
					device_prio_max = prio;
					device_id_primary = device_ids[cur_mag];
				}
			}
		}
	}
	
	// Limit update rate to get equally spaced measurements over time (in ms)
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available
				unsigned int orb_interval_msecs = (worker_data.calibration_interval_perside_useconds / 1000) / worker_data.calibration_points_perside;
				
				//mavlink_and_console_log_info(mavlink_fd, "Orb interval %u msecs", orb_interval_msecs);
				orb_set_interval(worker_data.sub_mag[cur_mag], orb_interval_msecs);
			}
		}
		
	}
    
	if (result == calibrate_return_ok) {
		int cancel_sub  = calibrate_cancel_subscribe();

		result = calibrate_from_orientation(mavlink_fd,                         // Mavlink fd to write output
						    cancel_sub,                         // Subscription to vehicle_command for cancel support
						    worker_data.side_data_collected,    // Sides to calibrate
						    mag_calibration_worker,             // Calibration worker
						    &worker_data,			// Opaque data for calibration worked
						    true);				// true: lenient still detection
		calibrate_cancel_unsubscribe(cancel_sub);
	}
	
	// Close subscriptions
	for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
		if (worker_data.sub_mag[cur_mag] >= 0) {
			px4_close(worker_data.sub_mag[cur_mag]);
		}
	}
	
	// Calculate calibration values for each mag
	
	
	float sphere_x[max_mags];
	float sphere_y[max_mags];
	float sphere_z[max_mags];
	float sphere_radius[max_mags];
	
	// Sphere fit the data to get calibration values
	if (result == calibrate_return_ok) {
		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				// Mag in this slot is available and we should have values for it to calibrate
				
				sphere_fit_least_squares(worker_data.x[cur_mag], worker_data.y[cur_mag], worker_data.z[cur_mag],
							 worker_data.calibration_counter_total[cur_mag],
							 100, 0.0f,
							 &sphere_x[cur_mag], &sphere_y[cur_mag], &sphere_z[cur_mag],
							 &sphere_radius[cur_mag]);
				
				if (!PX4_ISFINITE(sphere_x[cur_mag]) || !PX4_ISFINITE(sphere_y[cur_mag]) || !PX4_ISFINITE(sphere_z[cur_mag])) {
					mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: NaN in sphere fit for mag #%u", cur_mag);
					result = calibrate_return_error;
				}

				if (sqrtf(sphere_x[cur_mag] * sphere_x[cur_mag] +
					sphere_y[cur_mag] * sphere_y[cur_mag] + sphere_z[cur_mag] * sphere_z[cur_mag])
					> MAG_MAX_OFFSET_LEN) {
					mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: Excessive offset for mag #%u", cur_mag);
					result = calibrate_return_error;
				}
			}
		}
	}

	// Print uncalibrated data points
	if (result == calibrate_return_ok) {

		printf("RAW DATA:\n--------------------\n");
		for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {

			if (worker_data.calibration_counter_total[cur_mag] == 0) {
				continue;
			}

			printf("RAW: MAG %u with %u samples:\n", (unsigned)cur_mag, (unsigned)worker_data.calibration_counter_total[cur_mag]);

			for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
				float x = worker_data.x[cur_mag][i];
				float y = worker_data.y[cur_mag][i];
				float z = worker_data.z[cur_mag][i];
				printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
			}

			printf(">>>>>>>\n");
		}

		printf("CALIBRATED DATA:\n--------------------\n");
		for (size_t cur_mag = 0; cur_mag < max_mags; cur_mag++) {

			if (worker_data.calibration_counter_total[cur_mag] == 0) {
				continue;
			}

			printf("Calibrated: MAG %u with %u samples:\n", (unsigned)cur_mag, (unsigned)worker_data.calibration_counter_total[cur_mag]);

			for (size_t i = 0; i < worker_data.calibration_counter_total[cur_mag]; i++) {
				float x = worker_data.x[cur_mag][i] - sphere_x[cur_mag];
				float y = worker_data.y[cur_mag][i] - sphere_y[cur_mag];
				float z = worker_data.z[cur_mag][i] - sphere_z[cur_mag];
				printf("%8.4f, %8.4f, %8.4f\n", (double)x, (double)y, (double)z);
			}

			printf("SPHERE RADIUS: %8.4f\n", (double)sphere_radius[cur_mag]);
			printf(">>>>>>>\n");
		}
	}
	
	// Data points are no longer needed
	for (size_t cur_mag=0; cur_mag<max_mags; cur_mag++) {
		free(worker_data.x[cur_mag]);
		free(worker_data.y[cur_mag]);
		free(worker_data.z[cur_mag]);
	}
	
	if (result == calibrate_return_ok) {

		(void)param_set_no_notification(param_find("CAL_MAG_PRIME"), &(device_id_primary));

		for (unsigned cur_mag=0; cur_mag<max_mags; cur_mag++) {
			if (device_ids[cur_mag] != 0) {
				int fd_mag = -1;
				struct mag_scale mscale;
				
				// Set new scale
				
				(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, cur_mag);
				fd_mag = px4_open(str, 0);
				if (fd_mag < 0) {
					mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: unable to open mag device #%u", cur_mag);
					result = calibrate_return_error;
				}
				
				if (result == calibrate_return_ok) {
					if (px4_ioctl(fd_mag, MAGIOCGSCALE, (long unsigned int)&mscale) != OK) {
						mavlink_and_console_log_critical(mavlink_fd, "[cal] ERROR: failed to get current calibration #%u", cur_mag);
						result = calibrate_return_error;
					}
				}

				if (result == calibrate_return_ok) {
					mscale.x_offset = sphere_x[cur_mag];
					mscale.y_offset = sphere_y[cur_mag];
					mscale.z_offset = sphere_z[cur_mag];
					mscale.x_scale = mag_sphere_radius/sphere_radius[cur_mag];
					mscale.y_scale = mag_sphere_radius/sphere_radius[cur_mag];
					mscale.z_scale = mag_sphere_radius/sphere_radius[cur_mag];

					if (px4_ioctl(fd_mag, MAGIOCSSCALE, (long unsigned int)&mscale) != OK) {
						mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_APPLY_CAL_MSG, cur_mag);
						result = calibrate_return_error;
					}
				}
				
				// Mag device no longer needed
				if (fd_mag >= 0) {
					px4_close(fd_mag);
				}

				if (result == calibrate_return_ok) {
					bool failed = false;
					
					/* set parameters */
					(void)sprintf(str, "CAL_MAG%u_ID", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(device_ids[cur_mag])));
					(void)sprintf(str, "CAL_MAG%u_XOFF", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(mscale.x_offset)));
					(void)sprintf(str, "CAL_MAG%u_YOFF", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(mscale.y_offset)));
					(void)sprintf(str, "CAL_MAG%u_ZOFF", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(mscale.z_offset)));
					(void)sprintf(str, "CAL_MAG%u_XSCALE", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(mscale.x_scale)));
					(void)sprintf(str, "CAL_MAG%u_YSCALE", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(mscale.y_scale)));
					(void)sprintf(str, "CAL_MAG%u_ZSCALE", cur_mag);
					failed |= (OK != param_set_no_notification(param_find(str), &(mscale.z_scale)));

					if (failed) {
						mavlink_and_console_log_critical(mavlink_fd, CAL_ERROR_SET_PARAMS_MSG, cur_mag);
						result = calibrate_return_error;
					} else {
						mavlink_and_console_log_info(mavlink_fd, "[cal] mag #%u off: x:%.2f y:%.2f z:%.2f Ga",
									     cur_mag,
									     (double)mscale.x_offset, (double)mscale.y_offset, (double)mscale.z_offset);
						mavlink_and_console_log_info(mavlink_fd, "[cal] mag #%u scale: x:%.2f y:%.2f z:%.2f",
									     cur_mag,
									     (double)mscale.x_scale, (double)mscale.y_scale, (double)mscale.z_scale);
					}
				}
			}
		}
	}

	return result;
}

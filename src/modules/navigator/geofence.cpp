/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file geofence.cpp
 * Provides functions for handling the geofence
 *
 * @author Jean Cyr <jean.m.cyr@gmail.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#include "geofence.h"

#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <string.h>
#include <dataman/dataman.h>
#include <systemlib/err.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <nuttx/config.h>
#include <unistd.h>
#include <mavlink/mavlink_log.h>
#include <geo/geo.h>
#include <drivers/drv_hrt.h>

#define GEOFENCE_OFF 0
#define GEOFENCE_FILE_ONLY 1
#define GEOFENCE_MAX_DISTANCES_ONLY 2
#define GEOFENCE_FILE_AND_MAX_DISTANCES 3

#define GEOFENCE_RANGE_WARNING_LIMIT 3000000
#define RESTRICTED_AREA_WARNING_LIMIT 3000000

#define safe_dis 30.0f

/* Oddly, ERROR is not defined for C++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Geofence::Geofence() :
	SuperBlock(NULL, "GF"),
	_fence_pub(-1),
	_geofence_result_pub(-1),
	_geofence_result{},
	_home_pos{},
	_home_pos_set(false),
	_last_horizontal_range_warning(0),
	_last_vertical_range_warning(0),
	_last_restricted_area_warning(0),
	_altitude_min(0),
	_altitude_max(0),
	_verticesCount(0),
	_param_geofence_mode(this, "MODE"),
	_param_altitude_mode(this, "ALTMODE"),
	_param_source(this, "SOURCE"),
	_param_counter_threshold(this, "COUNT"),
	_param_max_hor_distance(this, "MAX_HOR_DIST"),
	_param_max_ver_distance(this, "MAX_VER_DIST"),
	_outside_counter(0),
	_mavlinkFd(-1)
{
	/* Load initial params */
	updateParams();
}

Geofence::~Geofence()
{

}


bool Geofence::inside(const struct vehicle_global_position_s &global_position)
{
	return inside(global_position.lat, global_position.lon, global_position.alt);
}

bool Geofence::inside(const struct vehicle_global_position_s &global_position, float baro_altitude_amsl)
{
	return inside(global_position.lat, global_position.lon, baro_altitude_amsl);
}


bool Geofence::inside(const struct vehicle_global_position_s &global_position,
		      const struct vehicle_gps_position_s &gps_position, float baro_altitude_amsl,
		      const struct home_position_s home_pos, bool home_position_set)
{
	updateParams();

	_home_pos = home_pos;
	_home_pos_set = home_position_set;

	if (getAltitudeMode() == Geofence::GF_ALT_MODE_WGS84) {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return inside(global_position);

		} else {
			return inside((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
				      (double)gps_position.alt * 1.0e-3);
		}

	} else {
		if (getSource() == Geofence::GF_SOURCE_GLOBALPOS) {
			return inside(global_position, baro_altitude_amsl);

		} else {
			return inside((double)gps_position.lat * 1.0e-7, (double)gps_position.lon * 1.0e-7,
				      baro_altitude_amsl);
		}
	}
}

bool Geofence::inside(double lat, double lon, float altitude)
{
	if (_param_geofence_mode.get() >= GEOFENCE_MAX_DISTANCES_ONLY) {
		int32_t max_horizontal_distance = _param_max_hor_distance.get();
		int32_t max_vertical_distance = _param_max_ver_distance.get();

		if (max_horizontal_distance > 0 || max_vertical_distance > 0) {
			if (_home_pos_set) {
				float dist_xy = -1.0f;
				float dist_z = -1.0f;
				get_distance_to_point_global_wgs84(lat, lon, altitude,
								   _home_pos.lat, _home_pos.lon, _home_pos.alt,
								   &dist_xy, &dist_z);

				if (max_vertical_distance > 0 && (dist_z > max_vertical_distance)) {
					if (hrt_elapsed_time(&_last_vertical_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
						mavlink_log_critical(_mavlinkFd, "Geofence exceeded max vertical distance by %.1f m",
								     (double)(dist_z - max_vertical_distance));
						_last_vertical_range_warning = hrt_absolute_time();
					}

					return false;
				}

				if (max_horizontal_distance > 0 && (dist_xy > max_horizontal_distance)) {
					if (hrt_elapsed_time(&_last_horizontal_range_warning) > GEOFENCE_RANGE_WARNING_LIMIT) {
						mavlink_log_critical(_mavlinkFd, "Geofence exceeded max horizontal distance by %.1f m",
								     (double)(dist_xy - max_horizontal_distance));
						_last_horizontal_range_warning = hrt_absolute_time();
					}

					return false;
				}
			}
		}
	}

	bool is_inside = inside_restricted_area(lat, lon, altitude);

	if (is_inside) {
		_geofence_result.fly_in_restricted_area = true;

	} else {
		_geofence_result.fly_in_restricted_area = false;
	}

	/* publish geofence result */
	publish_geofence_result();

	if (_geofence_result.fly_in_restricted_area) {
		return false;

	} else {
		return true;
	}
}

bool Geofence:: inside_restricted_area(double lat, double lon, float altitude)
{
        struct fence_vertex_s vertex;

        float dist_xy = -1.0f;
        float dist_z = -1.0f;

        int low = 0;
        int high = 0;
        int mid = 0;
        int sel = 0;

        dm_item_t _dm_item_t;

        if(dm_read(DM_KEY_RESTRICTED_AREA_NUM, 0, &high, sizeof(int)) != sizeof(int)) {
                return false;
        }

        //Binary-Search
        while (low <= high)
        {
                mid = low + (high - low) / 2;
                warnx("low %d, mid %d, high %d", low, mid, high);
                if ((mid / 256) == 0) {
                        _dm_item_t = DM_KEY_RESTRICTED_AREA_0;
                            sel = mid;

                } else if ((mid / 256) == 1) {

                        _dm_item_t = DM_KEY_RESTRICTED_AREA_1;
                            sel = mid - 256;

                } else {
                        _dm_item_t = DM_KEY_RESTRICTED_AREA_2;
                            sel = mid - 512;

                }

                if (dm_read(_dm_item_t, sel, &vertex, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
                        break;
                }

                get_distance_to_point_global_wgs84(lat, lon, altitude,
                                   vertex.lat, vertex.lon, 0.0f,
                                   &dist_xy, &dist_z);

                if (dist_xy < safe_dis )
                {
                        if (hrt_elapsed_time(&_last_restricted_area_warning) > RESTRICTED_AREA_WARNING_LIMIT) {
                                mavlink_log_critical(_mavlinkFd, "fly in restricted area");
                                _last_restricted_area_warning = hrt_absolute_time();
                        }

                        return true;
                }

                if (lon > (double)vertex.lon) {
                        low = mid + 1;

                } else {
                        high = mid - 1;

                }
        }

        /* fly safe */
        return false;
}

bool Geofence::inside_polygon(double lat, double lon, float altitude)
{
	/* Return true if geofence is disabled or only checking max distances */
	if ((_param_geofence_mode.get() == GEOFENCE_OFF)
	    || (_param_geofence_mode.get() == GEOFENCE_MAX_DISTANCES_ONLY)) {
		return true;
	}

	if (valid()) {

		if (!isEmpty()) {
			/* Vertical check */
			if (altitude > _altitude_max || altitude < _altitude_min) {
				return false;
			}

			/*Horizontal check */
			/* Adaptation of algorithm originally presented as
			 * PNPOLY - Point Inclusion in Polygon Test
			 * W. Randolph Franklin (WRF) */

			bool c = false;

			struct fence_vertex_s temp_vertex_i;
			struct fence_vertex_s temp_vertex_j;

			/* Red until fence is finished */
			for (unsigned i = 0, j = _verticesCount - 1; i < _verticesCount; j = i++) {
				if (dm_read(DM_KEY_FENCE_POINTS, i, &temp_vertex_i, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
					break;
				}

				if (dm_read(DM_KEY_FENCE_POINTS, j, &temp_vertex_j, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
					break;
				}

				// skip vertex 0 (return point)
				if (((double)temp_vertex_i.lon >= lon) != ((double)temp_vertex_j.lon >= lon) &&
				    (lat <= (double)(temp_vertex_j.lat - temp_vertex_i.lat) * (lon - (double)temp_vertex_i.lon) /
				     (double)(temp_vertex_j.lon - temp_vertex_i.lon) + (double)temp_vertex_i.lat)) {
					c = !c;
				}

			}

			return c;

		} else {
			/* Empty fence --> accept all points */
			return true;
		}

	} else {
		/* Invalid fence --> accept all points */
		return true;
	}
}

bool
Geofence::valid()
{
	// NULL fence is valid
	if (isEmpty()) {
		return true;
	}

	// Otherwise
	if ((_verticesCount < 4) || (_verticesCount > fence_s::GEOFENCE_MAX_VERTICES)) {
		warnx("Fence must have at least 3 sides and not more than %d", fence_s::GEOFENCE_MAX_VERTICES - 1);
		return false;
	}

	return true;
}

void
Geofence::addPoint(int argc, char *argv[])
{
	int ix, last;
	double lon, lat;
	struct fence_vertex_s vertex;
	char *end;

	if ((argc == 1) && (strcmp("-clear", argv[0]) == 0)) {
		dm_clear(DM_KEY_FENCE_POINTS);
		publishFence(0);
		return;
	}

	if (argc < 3) {
		errx(1, "Specify: -clear | sequence latitude longitude [-publish]");
	}

	ix = atoi(argv[0]);

	if (ix >= DM_KEY_FENCE_POINTS_MAX) {
		errx(1, "Sequence must be less than %d", DM_KEY_FENCE_POINTS_MAX);
	}

	lat = strtod(argv[1], &end);
	lon = strtod(argv[2], &end);

	last = 0;

	if ((argc > 3) && (strcmp(argv[3], "-publish") == 0)) {
		last = 1;
	}

	vertex.lat = (float)lat;
	vertex.lon = (float)lon;

	if (dm_write(DM_KEY_FENCE_POINTS, ix, DM_PERSIST_POWER_ON_RESET, &vertex, sizeof(vertex)) == sizeof(vertex)) {
		if (last) {
			publishFence((unsigned)ix + 1);
		}

		return;
	}

	errx(1, "can't store fence point");
}

void
Geofence::publishFence(unsigned vertices)
{
	if (_fence_pub == -1) {
		_fence_pub = orb_advertise(ORB_ID(fence), &vertices);

	} else {
		orb_publish(ORB_ID(fence), _fence_pub, &vertices);
	}
}

void
Geofence::publish_geofence_result()
{
        if (_geofence_result_pub == -1) {
                _geofence_result_pub = orb_advertise(ORB_ID(geofence_result), &_geofence_result);

        } else {
                orb_publish(ORB_ID(geofence_result), _geofence_result_pub, &_geofence_result);
        }
}

int
Geofence::loadFromFile(const char *filename)
{
	FILE		*fp;
	char		line[120];
	int			pointCounter = 0;
	const char commentChar = '#';
	int rc = ERROR;
	int sel = 0;
	int newVersion = -1;
	bool reloadData = false;

	dm_item_t _dm_item_t;

	 /* open the mixer definition file */
        fp = fopen(GEOFENCE_FILENAME, "r");

        if (fp == NULL) {
                return ERROR;
        }

        /* create geofence points from valid lines and store in DM */
        for (;;) {

                /* get a line, bail on error/EOF */
                if (fgets(line, sizeof(line), fp) == NULL) {
                        break;
                }

                /* Trim leading whitespace */
                size_t textStart = 0;

                while ((textStart < sizeof(line) / sizeof(char)) && isspace(line[textStart])) { textStart++; }

                /* if the line starts with #, skip */
                if (line[textStart] == commentChar) {
                        continue;
                }

                /* if there is only a linefeed, skip it */
                if (line[0] == '\n') {
                        continue;
                }

                if (reloadData) {
                        /* Parse the line as a geofence point */
                        struct fence_vertex_s vertex;

                        /* Handle decimal degree format */
                        if (sscanf(line, "%f %f", &(vertex.lon), &(vertex.lat)) != 2) {
                                warnx("Scanf to parse geofence vertex failed.");
                                goto error;
                        }

                        if ((pointCounter / 256) == 0) {
                                _dm_item_t = DM_KEY_RESTRICTED_AREA_0;
                                sel = pointCounter;

                        } else if ((pointCounter / 256) == 1) {
                                _dm_item_t = DM_KEY_RESTRICTED_AREA_1;
                                sel = pointCounter - 256;

                        } else {
                                _dm_item_t = DM_KEY_RESTRICTED_AREA_2;
                                sel = pointCounter - 512;
                        }

                        if (dm_write(_dm_item_t, sel, DM_PERSIST_POWER_ON_RESET, &vertex, sizeof(vertex)) != sizeof(vertex)) {
                                goto error;
                        }

                        pointCounter++;

                } else {
                        /* Parse the line as the database version */
                        if (sscanf(line, "%d", &newVersion) != 1) {
                                warnx("Scanf to parse database version failed.");
                                goto error;
                        }

                        if (checkDm(newVersion)) {
                                mavlink_log_info(_mavlinkFd, "geofence.txt had been loaded, stop loading");
                                warnx("geofence.txt had been loaded, stop loading");
                                return true;

                        }else {
                                mavlink_log_info(_mavlinkFd, "geofence.txt has not been loaded, continue loading");
                                warnx("geofence.txt has not been loaded, continue loading");
                        }

                        /* Make sure no data is left in the datamanager */
                        clearDm();

                        if (dm_write(DM_KEY_RESTRICTED_AREA_VERSION, 0, DM_PERSIST_POWER_ON_RESET, &newVersion, sizeof(int)) != sizeof(int)) {
                                goto error;
                        }

                        warnx("geofence database version %d", newVersion);
                        reloadData = true;
                }
        }

        if (dm_write(DM_KEY_RESTRICTED_AREA_NUM, 0, DM_PERSIST_POWER_ON_RESET, &pointCounter, sizeof(int)) != sizeof(int)) {
                goto error;
        }

        /* Check if Noflyzones import was successful */
        if ( pointCounter > 0) {
                warnx("geofence imported %d area", pointCounter);
                mavlink_log_info(_mavlinkFd, "geofence imported %d area", pointCounter);
                rc = OK;

        } else {
                warnx("geofence import error");
                mavlink_log_critical(_mavlinkFd, "geofence import error");
        }

error:
        fclose(fp);
        return rc;
}

bool Geofence::checkDm(int newVersion)
{
        struct fence_vertex_s vertex;

        int sel = 0;
        int area_cnt = 0;
        int oldVersion = -1;

        float lon_tmp = 180.0f;

        dm_item_t _dm_item_t;

        if (dm_read(DM_KEY_RESTRICTED_AREA_VERSION, 0, &oldVersion, sizeof(int)) != sizeof(int)) {
                return false;
        }

        if (newVersion != oldVersion) {
                return false;
        }


        if (dm_read(DM_KEY_RESTRICTED_AREA_NUM, 0, &area_cnt, sizeof(int)) != sizeof(int)) {
                return false;
        }

        area_cnt = area_cnt - 1;

        while(area_cnt >= 0) {

                if ((area_cnt / 256) == 0) {
                        _dm_item_t = DM_KEY_RESTRICTED_AREA_0;
                        sel = area_cnt;

                } else if ((area_cnt / 256) == 1) {
                        _dm_item_t = DM_KEY_RESTRICTED_AREA_1;
                        sel = area_cnt - 256;

                } else {
                        _dm_item_t = DM_KEY_RESTRICTED_AREA_2;
                        sel = area_cnt - 512;
                }

                if (dm_read(_dm_item_t, sel, &vertex, sizeof(struct fence_vertex_s)) != sizeof(struct fence_vertex_s)) {
                        return false;
                }

                if (vertex.lon > lon_tmp) {
                        return false;
                }

                lon_tmp = vertex.lon;

                area_cnt--;
        }

        return true;
}

int Geofence::clearDm()
{
        dm_clear(DM_KEY_RESTRICTED_AREA_VERSION);
        dm_clear(DM_KEY_RESTRICTED_AREA_0);
        dm_clear(DM_KEY_RESTRICTED_AREA_1);
        dm_clear(DM_KEY_RESTRICTED_AREA_2);
        dm_clear(DM_KEY_RESTRICTED_AREA_NUM);

        return OK;
}

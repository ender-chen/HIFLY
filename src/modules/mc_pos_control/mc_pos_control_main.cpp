/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4.h>
#include <functional>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/waypoint.h>


#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	const float alt_ctl_dz = 0.2f;

	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	int		_mavlink_fd;			/**< mavlink fd */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_att_sub;			/**< vehicle attitude subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */
	int		_geofence_result_sub;	/**< geofence result*/
	int 	_waypoint_sub;			/**< waypoint */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
	struct geofence_result_s			_geofence_result;	/**< geofence result */
	struct waypoint_s _waypoint_sp;							/**<waypoint stash */


	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t takeoff_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t mc_att_yaw_p;
		param_t follow_dist;
		param_t follow_yaw;
		param_t circle_radius;
		param_t circle_vel;
		param_t circle_rot;
		param_t fol_vel_min;
		param_t fol_vel_p;
		param_t fol_vel_dv;
		param_t fol_acc_max;
		param_t follow_fc_horizon_distance;
		param_t follow_fc_circle_angle_limit;
		param_t follow_fc_circle_accelerate;
		param_t follow_fc_direction;
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float tilt_max_air;
		float land_speed;
		float takeoff_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float mc_att_yaw_p;
		float follow_dist;
		float follow_yaw;
		float circle_radius;
		float circle_vel;
		float circle_rot;
		float fol_vel_min;
		float fol_vel_p;
		float fol_vel_dv;
		float fol_acc_max;
		float follow_fc_horizon_distance;
		float follow_fc_circle_angle_limit;
		float follow_fc_circle_accelerate;
		float follow_fc_direction;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _mode_auto;

	bool _circle_fixed_init;

	bool _circle_follow_init;
	bool _move_to_edge;

        /* circle parameters */
	float _circle_radius;        // maximum horizontal speed in cm/s during missions
	float _circle_rot_rate;          // rotation speed in deg/sec
	float _circle_angle;         // current angular position around circle in radians (0=directly north of the center of the circle)
	float _circle_angle_total;   // total angle travelled in radians
	float _circle_angular_vel;   // angular velocity in radians/sec
	float _circle_angular_vel_max;   // maximum velocity in radians/sec

	float _fol_vel_xy;

	/*far close follow circle parameter */
	float _follow_fc_circle_angle;
	float _follow_fc_circle_angle_vel_max;
	bool _follow_fc_circle_init;
	float _follow_fc_circle_radius;
	float _follow_fc_angle_vel_old;
	float _follow_fc_angle_vel_new;
	float _follow_fc_start_angle;
	float _follow_fc_angle_vel_accelearate;


	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _sp_move_rate;

	math::Vector<3> _circle_center;
	math::Vector<3> _circle_edge;

	math::Vector<3> _follow_fc_circle_center;

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude
	 */
	void		reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	bool		cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r,
					const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res);

	void 		follow_fc_init();

	void 		follow_fc_update(float dt);

	void 		control_follow_fc(float dt);
	/**
	 * Move to sp position setpotion
	**/
	void		move_to_sp(const math::Vector<3>& sp, float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	/**
	 * circle calculate velocities
	*/
	void            circle_calc_velocities();

	/**
	 * circle init start angle
	 */
	void		circle_init_start_angle(bool use_heading);

	/**
	 * circle init
	 */
	void		circle_fixed_init();

	/**
	 * circle update
	 */
	void		circle_update(float dt);

	/**
	 * control simple circle
	 */
	void		control_circle_fixed(float dt);

	/**
	 * Set position setpoint for follow
	 */
	void		control_follow_loiter(float dt);

    /**
    * control follow camera
    */
    void        control_follow_camera(float dt);

	/**
	 * circle init with waypoint
	 */
	void		circle_follow_init(const math::Vector<3>& center);

	/**
	 * center update
	 */
	void		center_update(const math::Vector<3>& sp, float dt);

	/**
	 * get closet point
	 */
	void		get_closest_point_on_circle();

	/**
	 * control follow circle
	 */
	void		control_follow_circle(float dt);

	/**
	 * move to closest point on circle
	 */
	void		move_to_edge(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterPositionControl	*g_control;
}

MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(NULL, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_fd(-1),

/* subscriptions */
	_att_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),
	_geofence_result_sub(-1),
	_waypoint_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_local_pos_sp_pub(-1),
	_global_vel_sp_pub(-1),
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_mode_auto(false),

	_circle_fixed_init(false),
	_circle_follow_init(false),
	_move_to_edge(false),
	_circle_radius(0.0f),
	_circle_rot_rate(0.0f),
	_circle_angle(0.0f),
	_circle_angle_total(0.0f),
	_circle_angular_vel(0.0f),
	_circle_angular_vel_max(0.0f),
	_fol_vel_xy(0.0f),
	_follow_fc_circle_angle(0.0f),
	_follow_fc_circle_angle_vel_max(0.0f),
	_follow_fc_circle_init(false),
	_follow_fc_circle_radius(0.0f),
	_follow_fc_angle_vel_old(0.0f),
	_follow_fc_angle_vel_new(0.0f),
	_follow_fc_start_angle(0.0f),
	_follow_fc_angle_vel_accelearate(0.0f)
{
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_att, 0, sizeof(_att));
	memset(&_att_sp, 0, sizeof(_att_sp));
	memset(&_manual, 0, sizeof(_manual));
	memset(&_control_mode, 0, sizeof(_control_mode));
	memset(&_arming, 0, sizeof(_arming));
	memset(&_local_pos, 0, sizeof(_local_pos));
	memset(&_pos_sp_triplet, 0, sizeof(_pos_sp_triplet));
	memset(&_local_pos_sp, 0, sizeof(_local_pos_sp));
	memset(&_global_vel_sp, 0, sizeof(_global_vel_sp));

	memset(&_ref_pos, 0, sizeof(_ref_pos));
	memset(&_geofence_result, 0, sizeof(_geofence_result));
	memset(&_waypoint_sp, 0, sizeof(_waypoint_sp));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_sp_move_rate.zero();

	_circle_center.zero();
	_circle_edge.zero();

	_follow_fc_circle_center.zero();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.z_p			= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max	= param_find("MPC_Z_VEL_MAX");
	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.takeoff_speed = param_find("MPC_TAKEOFF_SP");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");

	_params_handles.follow_dist = param_find("MPC_FOLLOW_DIST");
	_params_handles.follow_yaw = param_find("MPC_FOLLOW_YAW");
	_params_handles.circle_radius = param_find("MPC_CIRCLE_R");
	_params_handles.circle_vel = param_find("MPC_CIRCLE_VEL");
	_params_handles.circle_rot = param_find("MPC_CIRCLE_ROT");
	_params_handles.fol_vel_min = param_find("MPC_FOL_VEL_MIN");
	_params_handles.fol_vel_p = param_find("MPC_FOL_VEL_P");
	_params_handles.fol_vel_dv = param_find("MPC_FOL_VEL_DV");
	_params_handles.fol_acc_max = param_find("MPC_FOL_ACC_MAX");

	_params_handles.follow_fc_horizon_distance = param_find("MPC_FCF_HORI");
	_params_handles.follow_fc_circle_angle_limit = param_find("MPC_FCF_RL");
	_params_handles.follow_fc_circle_accelerate = param_find("MPC_FCF_ACC");
	_params_handles.follow_fc_direction = param_find("MPC_FCF_DIR");


	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	pos_control::g_control = nullptr;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.takeoff_speed, &_params.takeoff_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		param_get(_params_handles.follow_dist, &_params.follow_dist);
		param_get(_params_handles.follow_yaw, &_params.follow_yaw);
		param_get(_params_handles.circle_radius, &_params.circle_radius);
		param_get(_params_handles.circle_vel, &_params.circle_vel);
		param_get(_params_handles.circle_rot, &_params.circle_rot);
		param_get(_params_handles.fol_vel_min, &_params.fol_vel_min);
		param_get(_params_handles.fol_vel_p, &_params.fol_vel_p);
		param_get(_params_handles.fol_vel_dv, &_params.fol_vel_dv);
		param_get(_params_handles.fol_acc_max, &_params.fol_acc_max);
		param_get(_params_handles.follow_fc_horizon_distance, &_params.follow_fc_horizon_distance);
		param_get(_params_handles.follow_fc_circle_angle_limit, &_params.follow_fc_circle_angle_limit);
		param_get(_params_handles.follow_fc_circle_accelerate, &_params.follow_fc_circle_accelerate);
		_params.follow_fc_circle_angle_limit = math::radians(_params.follow_fc_circle_angle_limit);
		param_get(_params_handles.follow_fc_direction, &_params.follow_fc_direction);
		_params.follow_fc_direction = math::radians(_params.follow_fc_direction);


		float v;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max, &v);
		_params.vel_max(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;

		_params.sp_offs_max = _params.vel_max.edivide(_params.pos_p) * 2.0f;

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		param_get(_params_handles.mc_att_yaw_p,&v);
		_params.mc_att_yaw_p = v;
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}

	orb_check(_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}

	orb_check(_arming_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	orb_check(_geofence_result_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(geofence_result), _geofence_result_sub, &_geofence_result);
	}

	orb_check(_waypoint_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(waypoint), _waypoint_sub, &_waypoint_sp);
	}
}

float
MulticopterPositionControl::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

void
MulticopterPositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;
		/* shift position setpoint to make attitude setpoint continuous */
		_pos_sp(0) = _pos(0) + (_vel(0) - PX4_R(_att_sp.R_body, 0, 2) * _att_sp.thrust / _params.vel_p(0)
				- _params.vel_ff(0) * _sp_move_rate(0)) / _params.pos_p(0);
		_pos_sp(1) = _pos(1) + (_vel(1) - PX4_R(_att_sp.R_body, 1, 2) * _att_sp.thrust / _params.vel_p(1)
				- _params.vel_ff(1) * _sp_move_rate(1)) / _params.pos_p(1);
		mavlink_log_info(_mavlink_fd, "[mpc] reset pos sp: %d, %d", (int)_pos_sp(0), (int)_pos_sp(1));
	}
}

void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;
		_pos_sp(2) = _pos(2) + (_vel(2) - _params.vel_ff(2) * _sp_move_rate(2)) / _params.pos_p(2);
		mavlink_log_info(_mavlink_fd, "[mpc] reset alt sp: %d", -(int)_pos_sp(2));
	}
}

void
MulticopterPositionControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

void
MulticopterPositionControl::control_manual(float dt)
{
	_sp_move_rate.zero();

	if (_control_mode.flag_control_altitude_enabled) {
		/* move altitude setpoint with throttle stick */
		_sp_move_rate(2) = -scale_control(_manual.z - 0.5f, 0.5f, alt_ctl_dz);

		if((_sp_move_rate(2) < 0 && _geofence_result.geofence_ver_violated))
			/* cant move altitude setpoint */
			_sp_move_rate(2) = 0.0f;
	}

	if (_control_mode.flag_control_position_enabled) {
		/* move position setpoint with roll/pitch stick */

#if 0
		float rollx, pitchx, rolly, pitchy;
		float surper_simple_yaw = 0.0f;

		float home_bearing;

		home_bearing = PI/2 + atan2f((_local_pos.x - _home_position.x), -(_local_pos.y - _home_position.y));

		if(home_bearing < 0)
			home_bearing += 2*PI;

		if(home_bearing >= 2*PI)
			home_bearing -= 2*PI;

		surper_simple_yaw = home_bearing + PI;

		if((_manual.x > 0) && _geofence_result.geofence_hor_violated) {
			if(fabsf(_manual.x) <= fabsf(_manual.y)) {
				float _manual_x_temp;

				_manual_x_temp = -fabsf(_manual.y);
				rollx = cosf(surper_simple_yaw)*_manual_x_temp - sinf(surper_simple_yaw)*_manual.y;
				pitchx = sinf(surper_simple_yaw)*_manual_x_temp + cosf(surper_simple_yaw)*_manual.y;
			}
			else {
				rollx = -sinf(surper_simple_yaw)*_manual.y;
				pitchx = cosf(surper_simple_yaw)*_manual.y;
			}
		}
		else
		{
			rollx = cosf(surper_simple_yaw)*_manual.x - sinf(surper_simple_yaw)*_manual.y;
			pitchx = sinf(surper_simple_yaw)*_manual.x + cosf(surper_simple_yaw)*_manual.y;
		}

		rolly = cosf(_att.yaw)*rollx + sinf(_att.yaw)*pitchx;
		pitchy = -sinf(_att.yaw)*rollx + cosf(_att.yaw)*pitchx;

		_sp_move_rate(0) = rolly;
		_sp_move_rate(1) = pitchy;

		//_sp_move_rate(0) = _manual.x;
		//_sp_move_rate(1) = _manual.y;
	}
#endif
		_sp_move_rate(0) = _manual.x;
		_sp_move_rate(1) = _manual.y;
	}

	/* limit setpoint move rate */
	float sp_move_norm = _sp_move_rate.length();

	if (sp_move_norm > 1.0f) {
		_sp_move_rate /= sp_move_norm;
	}

	/* _sp_move_rate scaled to 0..1, scale it to max speed and rotate around yaw */
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
	_sp_move_rate = R_yaw_sp * _sp_move_rate.emult(_params.vel_max);


	if (_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (_control_mode.flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}

	static float slow_down = 1.0f;
	/* check position setpoing when geofence horizon violated */
	if (_control_mode.flag_control_position_enabled && _geofence_result.geofence_hor_violated) {
		float _hor_tmp = 0.0f;
		float _hor_next = 0.0f;
		float _hor_now = 0.0f;

		math::Vector<3> _pos_tmp;
		_pos_tmp.zero();

		/* calculate next position setpoints */
		_pos_tmp = _pos_sp + _sp_move_rate * dt;
		_hor_now = sqrt(_pos_sp(0) * _pos_sp(0) + _pos_sp(1) * _pos_sp(1));
		_hor_next = sqrt(_pos_tmp(0) * _pos_tmp(0) + _pos_tmp(1) * _pos_tmp(1));
		_hor_tmp = _hor_next - _hor_now;

		slow_down -= dt;

		if(slow_down < 0.0f) {
			slow_down = 0.0f;
		}

		/* check whether position is more and more far away from home*/
		if (_hor_tmp > 0) {
			/* far away from home, slow the vehicle down */
			_sp_move_rate(0) *= slow_down;
			_sp_move_rate(1) *= slow_down;
		}

	} else {
		slow_down = 1.0f;
	}

	/* feed forward setpoint move rate with weight vel_ff */
	_vel_ff = _sp_move_rate.emult(_params.vel_ff);

	/* move position setpoint */
	_pos_sp += _sp_move_rate * dt;

	/* check if position setpoint is too far from actual position */
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

void
MulticopterPositionControl::control_offboard(float dt)
{
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}

	if (_pos_sp_triplet.current.valid) {
		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;
		} else if (_control_mode.flag_control_velocity_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* control velocity */
			/* reset position setpoint to current position if needed */
			reset_pos_sp();

			/* set position setpoint move rate */
			_sp_move_rate(0) = _pos_sp_triplet.current.vx;
			_sp_move_rate(1) = _pos_sp_triplet.current.vy;
		}

		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

		if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.position_valid) {
			/* Control altitude */
			_pos_sp(2) = _pos_sp_triplet.current.z;
		} else if (_control_mode.flag_control_climb_rate_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();

			/* set altitude setpoint move rate */
			_sp_move_rate(2) = _pos_sp_triplet.current.vz;
		}

		/* feed forward setpoint move rate with weight vel_ff */
		_vel_ff = _sp_move_rate.emult(_params.vel_ff);

		/* move position setpoint */
		_pos_sp += _sp_move_rate * dt;

	} else {
		reset_pos_sp();
		reset_alt_sp();
	}
}

bool
MulticopterPositionControl::cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r,
		const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	/* we have triangle CDX with known CD and CX = R, find DX */
	if (sphere_r > cd_len) {
		/* have two roots, select one in A->B direction from D */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
		res = d + ab_norm * dx_len;
		return true;

	} else {
		/* have no roots, return D */
		res = d;
		return false;
	}
}

void
MulticopterPositionControl::control_follow_fc(float dt)
{
	if (!_mode_auto) {
	    /* reset position setpoint on AUTO mode activation */
	    _mode_auto = true;
	    reset_pos_sp();
	    reset_alt_sp();
    }

	follow_fc_init();

	follow_fc_update(dt);
}

void
MulticopterPositionControl::follow_fc_init()
{
	if (_follow_fc_circle_init) {
		return;
	}
	_follow_fc_circle_init = true;

	/* set circle center to circle_radius ahead of pos_sp point */
	mavlink_log_info(_mavlink_fd, "limit is %.2lf", (double)_params.follow_fc_circle_angle_limit);

	float limit_angel = _wrap_pi(_params.follow_fc_circle_angle_limit / 2);
	mavlink_log_info(_mavlink_fd, "limit angel is %.2lf", (double)limit_angel);
	mavlink_log_info(_mavlink_fd, "limit horizon is  %.2lf", (double)_params.follow_fc_horizon_distance);
	_follow_fc_circle_radius = _params.follow_fc_horizon_distance  / fabsf(sinf(limit_angel));
	float vertical_distance;
	vertical_distance = _params.follow_fc_horizon_distance  / fabsf(tanf(limit_angel));
	_follow_fc_circle_center(0) = _pos_sp(0) - _params.follow_fc_horizon_distance;
	_follow_fc_circle_center(1) = _pos_sp(1);
	_follow_fc_circle_center(2) = _pos_sp(2) - vertical_distance;
	mavlink_log_info(_mavlink_fd, "raidus is %.2lf, center(0) is %.2lf, center(2) is %.2f", (double)_follow_fc_circle_radius, (double)_follow_fc_circle_center(0), (double)_follow_fc_circle_center(2));
	mavlink_log_info(_mavlink_fd, "_pos_sp(0) is %.2lf, _pos_sp(2) is %.2lf", (int)_pos_sp(0), (int)_pos_sp(1));
	// calculate velocities

	_follow_fc_circle_angle_vel_max = _wrap_pi(_params.vel_max(0) / _follow_fc_circle_radius);
	mavlink_log_info(_mavlink_fd, "angel_vel_max %.2lf", (double)_follow_fc_circle_angle_vel_max);
	_follow_fc_start_angle = (M_PI_F / 2 - limit_angel);
	_follow_fc_circle_angle = _follow_fc_start_angle;
	_follow_fc_angle_vel_accelearate = _wrap_pi(_params.follow_fc_circle_accelerate / _follow_fc_circle_radius);
	_follow_fc_angle_vel_old = 0;
}
void
MulticopterPositionControl::follow_fc_update(float dt)
{
	float follow_fc_angle_vel_change;
	// update the target angle and total angle traveled
	follow_fc_angle_vel_change = _follow_fc_angle_vel_accelearate * dt;
	if(_follow_fc_circle_angle <= M_PI_F / 2)
	{
		_follow_fc_angle_vel_old += follow_fc_angle_vel_change;
	}
	else
	{
		_follow_fc_angle_vel_old -= follow_fc_angle_vel_change;
	}
	_follow_fc_angle_vel_new = math::constrain(_follow_fc_angle_vel_old, -_follow_fc_circle_angle_vel_max, _follow_fc_circle_angle_vel_max);
	float angle_change = _follow_fc_angle_vel_new * dt;
	_follow_fc_circle_angle += angle_change;
	// if the circle_radius is zero we are doing panorama so no need to update loiter target
	if (fabsf(_follow_fc_circle_radius) > FLT_EPSILON) {
		// calculate target position
		_pos_sp(0) = _follow_fc_circle_center(0) + _follow_fc_circle_radius * cosf(_follow_fc_circle_angle) * fabsf(cosf(_wrap_pi(_params.follow_fc_direction)));
		_pos_sp(1) = _follow_fc_circle_center(1) + _follow_fc_circle_radius * cosf(_follow_fc_circle_angle) * fabsf(sinf(_wrap_pi(_params.follow_fc_direction)));
		_pos_sp(2) = _follow_fc_circle_center(2) + _follow_fc_circle_radius * sinf(_follow_fc_circle_angle);
		//mavlink_log_info(_mavlink_fd, "_pos_sp(2) is %.2lf", (double)_pos_sp(2));
	}
	double lat_now, lon_now;
	if (_ref_timestamp != 0) {
		map_projection_reproject(&_ref_pos, _pos(0), _pos(1), &lat_now, &lon_now);
		_att_sp.yaw_body = get_bearing_to_next_waypoint(
			           lat_now, lon_now, _waypoint_sp.lat, _waypoint_sp.lon);
	}
}

void
MulticopterPositionControl::move_to_sp(const math::Vector<3>& sp, float dt)
{
	/* scaled space: 1 == position error resulting max allowed speed, L1 = 1 in this space */
	math::Vector<3> scale = _params.pos_p.edivide(_params.vel_max);	// TODO add mult param here

	/* move setpoint not faster than max allowed speed */
	math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

	/* convert current setpoint to scaled space */
	math::Vector<3> pos_sp_s = sp.emult(scale);

	/* difference between current and desired position setpoints, 1 = max speed */
	math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
	float d_pos_m_len = d_pos_m.length();

	if (d_pos_m_len > dt) {
		pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
	}

	/* scale result back to normal space */
	_pos_sp = pos_sp_s.edivide(scale);
}

void MulticopterPositionControl::circle_calc_velocities()
{
	_circle_rot_rate = math::constrain(_params.circle_rot, -20.0f, 20.0f);

        // if we are doing a panorama set the circle_angle to the current heading
        if (_circle_radius <= 0) {
                _circle_angular_vel_max =math::radians(_circle_rot_rate);
                _circle_angular_vel = _circle_angular_vel_max;
        } else {
                // set starting angle to current heading - 180 degrees
                _circle_angle = _wrap_pi(_att.yaw - M_PI_F);

                /* velocity max */
                float velocity_max = math::min(_params.circle_vel, _params.vel_max(0));

                // angular_velocity in radians per second
                _circle_angular_vel_max = velocity_max/_circle_radius;
                _circle_angular_vel_max = math::constrain(math::radians(_circle_rot_rate),-_circle_angular_vel_max,_circle_angular_vel_max);

		_circle_angular_vel = _circle_angular_vel_max;
        }
}

void MulticopterPositionControl::circle_init_start_angle(bool use_heading)
{
        // initialise angle total
        _circle_angle_total = 0;

        // if the radius is zero we are doing panorama so init angle to the current heading
        if (_circle_radius <= 0.0f) {
                _circle_angle = _att.yaw;
                return;
        }

        // if use_heading is true
        if (use_heading) {
                _circle_angle = _wrap_pi(_att.yaw-M_PI_F);

        } else {
                // if we are exactly at the center of the circle, init angle to directly behind vehicle (so vehicle will backup but not change heading)
                if ((fabsf(_circle_center(0) - _pos_sp(0)) < FLT_EPSILON) && (fabsf(_circle_center(1) - _pos_sp(1)) < FLT_EPSILON)) {
                        _circle_angle = _wrap_pi(_att.yaw-M_PI_F);
                } else {
                        // get bearing from circle center to vehicle in radians
                        float bearing_rad = atan2f(_pos_sp(1)-_circle_center(1), _pos_sp(0) - _circle_center(0));
                        _circle_angle = _wrap_pi(bearing_rad);
                }
        }
}

void MulticopterPositionControl::circle_fixed_init()
{
	if (_circle_fixed_init) {
		return;
	}

	/* set circle center to circle_radius ahead of pos_sp point */
	_circle_center(0) = _pos_sp(0) + _circle_radius * cosf(_att.yaw);
	_circle_center(1) = _pos_sp(1) + _circle_radius * sinf(_att.yaw);
	_circle_center(2) = _pos_sp(2);

	// calculate velocities
	circle_calc_velocities();

	// set starting angle from vehicle heading
	circle_init_start_angle(true);

	_circle_fixed_init = true;
}

void MulticopterPositionControl::circle_update(float dt)
{
        // update the target angle and total angle traveled
        float angle_change = _circle_angular_vel * dt;
        _circle_angle += angle_change;
        _circle_angle = _wrap_pi(_circle_angle);
        _circle_angle_total += angle_change;

        // if the circle_radius is zero we are doing panorama so no need to update loiter target
        if (fabsf(_circle_radius) > FLT_EPSILON) {
                // calculate target position
                _pos_sp(0) = _circle_center(0) + _circle_radius * cosf(-_circle_angle);
                _pos_sp(1) = _circle_center(1) - _circle_radius * sinf(-_circle_angle);
                _pos_sp(2) = _circle_center(2);
                // heading is 180 deg from vehicles target position around circle
		/* vehicle needs 1s to speed up, but _circle_angle almost increase
		   by _circle_angular_vel, so we ought to subtract the offset */
		_att_sp.yaw_body = _wrap_pi(_circle_angle - M_PI_F - _circle_angular_vel);

        } else {
                /*do nothing*/
                _att_sp.yaw_body = _circle_angle;
        }
}

void MulticopterPositionControl::control_circle_fixed(float dt)
{

        if (!_mode_auto) {
                _mode_auto = true;
                /* reset position setpoint on AUTO mode activation */
                reset_pos_sp();
                reset_alt_sp();
        }

	circle_fixed_init();

	circle_update(dt);
}



void MulticopterPositionControl::control_follow_loiter(float dt)
{
	if (!_mode_auto) {
		_mode_auto = true;
		/* reset position setpoint on AUTO mode activation */
		reset_pos_sp();
		reset_alt_sp();
	}

	//Poll position setpoint
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!isfinite(_pos_sp_triplet.current.lat) ||
			!isfinite(_pos_sp_triplet.current.lon) ||
			!isfinite(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	if (_pos_sp_triplet.current.valid) {
		/* in case of interrupted mission don't go to waypoint but stay at current position */
		_reset_pos_sp = true;
		_reset_alt_sp = true;

		/* project setpoint to local frame */
		math::Vector<3> curr_sp;
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		float vel_xy = sqrtf(_pos_sp_triplet.current.vx * _pos_sp_triplet.current.vx +
				_pos_sp_triplet.current.vy * _pos_sp_triplet.current.vy);


		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			float dx = curr_sp(0) - _pos(0);
			float dy = curr_sp(1) - _pos(1);
			/* calculate distance */
			float dist = sqrtf(dx * dx + dy * dy);

			if (dist > _params.follow_dist) {
				/* Rapid Deceleration */
				if ((vel_xy - _fol_vel_xy) / dt < -_params.fol_acc_max) {
					/* smooth velocity */
					_fol_vel_xy = _fol_vel_xy * (1 - _params.fol_vel_p) + vel_xy * _params.fol_vel_p;
				} else {
					_fol_vel_xy = vel_xy;
				}

				/* velocity compensation according to distance, not recommended */
				float dv = (dist - _params.follow_dist) * _params.fol_vel_dv;

				math::Vector<3> follow_vel;
				follow_vel(0) = math::constrain(_fol_vel_xy + dv, _params.fol_vel_min, _params.vel_max(0));
				follow_vel(1) = math::constrain(_fol_vel_xy + dv, _params.fol_vel_min, _params.vel_max(1));
				follow_vel(2) = _params.vel_max(2);

				/* scaled space: 1 == position error resulting max allowed speed, L1 = 1 in this space */
				math::Vector<3> scale = _params.pos_p.edivide(follow_vel);	// TODO add mult param here

				/* move setpoint not faster than max allowed speed */
				math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

				/* convert current setpoint to scaled space */
				math::Vector<3> pos_sp_s = curr_sp.emult(scale);

				/* difference between current and desired position setpoints, 1 = max speed */
				math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
				float d_pos_m_len = d_pos_m.length();

				if (d_pos_m_len > dt) {
					pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
				}

				/* scale result back to normal space */
				_pos_sp = pos_sp_s.edivide(scale);
			}

			if (dist > _params.follow_yaw) {
				/* update yaw setpoint if needed */
				if (isfinite(_pos_sp_triplet.current.yaw)) {
					_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
				}
			}

		} else {
			move_to_sp(curr_sp, dt);
		}
	}
}

void MulticopterPositionControl::control_follow_camera(float dt)
{
    if (!_mode_auto) {
        _mode_auto = true;
        /* reset position setpoint on AUTO mode activation */
        reset_pos_sp();
        reset_alt_sp();
    }

    //Poll position setpoint
    bool updated;
    orb_check(_pos_sp_triplet_sub, &updated);
    if (updated) {
        orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

        //Make sure that the position setpoint is valid
        if (!isfinite(_pos_sp_triplet.current.lat) ||
                !isfinite(_pos_sp_triplet.current.lon) ||
                !isfinite(_pos_sp_triplet.current.alt)) {
            _pos_sp_triplet.current.valid = false;
        }
    }

    if (_pos_sp_triplet.current.valid) {
        /* in case of interrupted mission don't go to waypoint but stay at current position */
        _reset_pos_sp = true;
        _reset_alt_sp = true;

        /* project setpoint to local frame */
        math::Vector<3> curr_sp;
        map_projection_project(&_ref_pos,
                _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
                &curr_sp.data[0], &curr_sp.data[1]);
        curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

        if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            move_to_sp(curr_sp, dt);
            return;
        }

	float dx = curr_sp(0) - _pos(0);
	float dy = curr_sp(1) - _pos(1);
	/* calculate distance */
	float dist = sqrtf(dx * dx + dy * dy);

	if (dist > _params.follow_yaw) {
		/* update yaw setpoint if needed */
		if (isfinite(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}
	}
    }
}

void MulticopterPositionControl::circle_follow_init(const math::Vector<3>& center)
{
	if (_circle_follow_init) {
		return;
	}

	_circle_center = center;

	// calculate velocities
	circle_calc_velocities();

	// set starting angle from vehicle heading
	circle_init_start_angle(false);
	get_closest_point_on_circle();

	_circle_follow_init = true;
	_move_to_edge = true;
}

void MulticopterPositionControl::move_to_edge(float dt)
{
	if ((_pos_sp - _circle_edge).length() < MIN_DIST) {
		_move_to_edge = false;
		mavlink_log_info(_mavlink_fd, "[mpc] moving to edge done");

	} else {
		move_to_sp(_circle_edge, dt);
	}

	/* towards to center */
	_att_sp.yaw_body = _wrap_pi(_circle_angle-M_PI_F);

}

void MulticopterPositionControl::get_closest_point_on_circle()
{
    // return center if radius is zero
    if (_circle_radius <= 0.0f) {
        _circle_edge = _circle_center;
        return;
    }

    // calc vector from current location to circle center
    math::Vector<2> vec;   // vector from circle center to current location
    vec(0) = (_pos_sp(0) - _circle_center(0));
    vec(1) = (_pos_sp(1) - _circle_center(1));

    float dist = sqrtf(vec(0) * vec(0) + vec(1) * vec(1));

    // if current location is exactly at the center of the circle return edge directly behind vehicle
    if (fabsf(dist) < FLT_EPSILON) {
        _circle_edge(0) = _circle_center(0) - _circle_radius * cosf(_att.yaw);// _ahrs.cos_yaw();
        _circle_edge(1) = _circle_center(1) - _circle_radius * sinf(_att.yaw);//_ahrs.sin_yaw();
        _circle_edge(2) = _circle_center(2);

        return;
    }

    // calculate closest point on edge of circle
    _circle_edge(0) = _circle_center(0) + vec(0) / dist * _circle_radius;
    _circle_edge(1) = _circle_center(1) + vec(1) / dist * _circle_radius;
    _circle_edge(2) = _circle_center(2);
}

void MulticopterPositionControl::center_update(const math::Vector<3>& sp, float dt)
{
	/* scaled space: 1 == position error resulting max allowed speed, L1 = 1 in this space */
	math::Vector<3> scale = _params.pos_p.edivide(_params.vel_max);	// TODO add mult param here

	/* move setpoint not faster than max allowed speed */
	math::Vector<3> pos_sp_old_s = _circle_center.emult(scale);

	/* convert current setpoint to scaled space */
	math::Vector<3> pos_sp_s = sp.emult(scale);

	/* difference between current and desired position setpoints, 1 = max speed */
	math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
	float d_pos_m_len = d_pos_m.length();

	if (d_pos_m_len > dt) {
		pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
	}

	/* scale result back to normal space */
	_circle_center = pos_sp_s.edivide(scale);
}

void MulticopterPositionControl::control_follow_circle(float dt)
{
        if (!_mode_auto) {
                _mode_auto = true;
                /* reset position setpoint on AUTO mode activation */
                reset_pos_sp();
                reset_alt_sp();
        }

	//Poll position setpoint
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!isfinite(_pos_sp_triplet.current.lat) ||
			!isfinite(_pos_sp_triplet.current.lon) ||
			!isfinite(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	if (_pos_sp_triplet.current.valid) {
		/* project setpoint to local frame */
		math::Vector<3> curr_sp;
		map_projection_project(&_ref_pos,
		_pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
		&curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			move_to_sp(curr_sp, dt);
			return;
		}

		circle_follow_init(curr_sp);

		if (!_move_to_edge) {
			/* need update circle center */
			center_update(curr_sp, dt);
			circle_update(dt);
		} else {
			move_to_edge(dt);
		}
	}
}

void MulticopterPositionControl::control_auto(float dt)
{
	if (!_mode_auto) {
		_mode_auto = true;
		/* reset position setpoint on AUTO mode activation */
		reset_pos_sp();
		reset_alt_sp();
	}

	//Poll position setpoint
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!isfinite(_pos_sp_triplet.current.lat) ||
			!isfinite(_pos_sp_triplet.current.lon) ||
			!isfinite(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	if (_pos_sp_triplet.current.valid) {
		/* in case of interrupted mission don't go to waypoint but stay at current position */
		_reset_pos_sp = true;
		_reset_alt_sp = true;

		/* project setpoint to local frame */
		math::Vector<3> curr_sp;
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		/* scaled space: 1 == position error resulting max allowed speed, L1 = 1 in this space */
		math::Vector<3> scale = _params.pos_p.edivide(_params.vel_max);	// TODO add mult param here


        if (!_control_mode.flag_control_manual_enabled &&
            _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
            _params.vel_max(2) = _params.takeoff_speed;
            scale = _params.pos_p.edivide(_params.vel_max);	// TODO add mult param here
        } else {
            float v;
            param_get(_params_handles.z_vel_max, &v);
            _params.vel_max(2) = v;
            /* scaled space: 1 == position error resulting max allowed speed, L1 = 1 in this space */
            scale = _params.pos_p.edivide(_params.vel_max);	// TODO add mult param here
        }

		/* convert current setpoint to scaled space */
		math::Vector<3> curr_sp_s = curr_sp.emult(scale);

		/* by default use current setpoint as is */
		math::Vector<3> pos_sp_s = curr_sp_s;

		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION && _pos_sp_triplet.previous.valid) {
			/* follow "previous - current" line */
			math::Vector<3> prev_sp;
			map_projection_project(&_ref_pos,
						   _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
						   &prev_sp.data[0], &prev_sp.data[1]);
			prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);

			if ((curr_sp - prev_sp).length() > MIN_DIST) {

				/* find X - cross point of L1 sphere and trajectory */
				math::Vector<3> pos_s = _pos.emult(scale);
				math::Vector<3> prev_sp_s = prev_sp.emult(scale);
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.length();
				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than L1 radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					if (_pos_sp_triplet.next.valid) {
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos,
									   _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
									   &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale);

							/* calculate angle prev - curr - next */
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s;
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();

							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;

							/* cos(b), b = angle pos - curr_sp - prev_sp */
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();
								/* if curr - next distance is larger than L1 radius, limit it */
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}

								/* feed forward position setpoint offset */
								math::Vector<3> pos_ff = prev_curr_s_norm *
										cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
										(1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff;
							}
						}
					}

				} else {
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);
					if (near) {
						/* L1 sphere crosses trajectory */

					} else {
						/* copter is too far from trajectory */
						/* if copter is behind prev waypoint, go directly to prev waypoint */
						if ((pos_sp_s - prev_sp_s) * prev_curr_s < 0.0f) {
							pos_sp_s = prev_sp_s;
						}

						/* if copter is in front of curr waypoint, go directly to curr waypoint */
						if ((pos_sp_s - curr_sp_s) * prev_curr_s > 0.0f) {
							pos_sp_s = curr_sp_s;
						}

						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}

		/* move setpoint not faster than max allowed speed */
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

		/* difference between current and desired position setpoints, 1 = max speed */
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();
		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}
		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND)
		{
			pos_sp_s(2) = pos_sp_old_s(2) + _params.land_speed * dt * scale(2);
		}

		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale);

		/* update yaw setpoint if needed */
		if (isfinite(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

	} else {
		/* no waypoint, do nothing, setpoint was already reset */
	}
}

void
MulticopterPositionControl::task_main()
{

	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	_geofence_result_sub = orb_subscribe(ORB_ID(geofence_result));
	_waypoint_sub = orb_subscribe(ORB_ID(waypoint));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool reset_yaw_sp = true;
	bool was_armed = false;

	hrt_abstime t_prev = 0;

	math::Vector<3> thrust_int;
	thrust_int.zero();
	math::Matrix<3, 3> R;
	R.identity();

	/* wakeup source */
	struct pollfd fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();
		parameters_update(false);

		_circle_radius = math::constrain(_params.circle_radius, 5.0f, 30.0f);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			reset_int_z = true;
			reset_int_xy = true;
			reset_yaw_sp = true;
		}

		//Update previous arming state
		was_armed = _control_mode.flag_armed;

		update_ref();

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled) {

			_pos(0) = _local_pos.x;
			_pos(1) = _local_pos.y;
			_pos(2) = _local_pos.z;

			_vel(0) = _local_pos.vx;
			_vel(1) = _local_pos.vy;
			_vel(2) = _local_pos.vz;

			_vel_ff.zero();
			_sp_move_rate.zero();

			if (_control_mode.flag_control_custom_mode != vehicle_control_mode_s::CUSTOM_MODE_CIRCLE) {
				_circle_fixed_init = false;
			}

			if (_control_mode.flag_control_custom_mode != vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_CIRCLE) {
				_circle_follow_init = false;
			}

			if (_control_mode.flag_control_custom_mode != vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_FC_ARC) {
				_follow_fc_circle_init = false;
			}

			if (_control_mode.flag_control_custom_mode != vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_LOITER) {
				_fol_vel_xy = 0.0f;
			}

			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				control_manual(dt);
				_mode_auto = false;

			} else if (_control_mode.flag_control_offboard_enabled) {
				/* offboard control */
				control_offboard(dt);
				_mode_auto = false;
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_CIRCLE) {
				control_circle_fixed(dt);
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_LOITER) {
				control_follow_loiter(dt);
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_CAMERA) {
                control_follow_camera(dt);
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_CIRCLE) {
				control_follow_circle(dt);
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_FC) {
				control_auto(dt);
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_FOLLOW_FC_ARC) {
				control_follow_fc(dt);
			} else if (_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_IDLE) {
				/* IDLE */
				_mode_auto = false;
			} else {
				/* AUTO */
				control_auto(dt);
			}

			if ((_control_mode.flag_control_custom_mode == vehicle_control_mode_s::CUSTOM_MODE_IDLE) || (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE)) {
				/* idle state, don't run controller and set zero thrust */
				R.identity();
				memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
				_att_sp.R_valid = true;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _att.yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

				} else {
					_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
				}

			} else {
				/* run position & altitude controllers, calculate velocity setpoint */
				math::Vector<3> pos_err = _pos_sp - _pos;

				_vel_sp = pos_err.emult(_params.pos_p) + _vel_ff;

				/* make sure velocity setpoint is saturated in xy*/
				float vel_norm_xy = sqrtf(_vel_sp(0)*_vel_sp(0) + 
					_vel_sp(1)*_vel_sp(1));
				if (vel_norm_xy > _params.vel_max(0)) { 
					/* note assumes vel_max(0) == vel_max(1) */
					_vel_sp(0) = _vel_sp(0)*_params.vel_max(0)/vel_norm_xy;
					_vel_sp(1) = _vel_sp(1)*_params.vel_max(1)/vel_norm_xy;
				}

				/* make sure velocity setpoint is saturated in z*/
				float vel_norm_z = sqrtf(_vel_sp(2)*_vel_sp(2));
				if (vel_norm_z > _params.vel_max(2)) {
					_vel_sp(2) = _vel_sp(2)*_params.vel_max(2)/vel_norm_z;
				}

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
					_vel_sp(2) = 0.0f;
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
				}

				/* use constant descend rate when landing, ignore altitude setpoint */
				// if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
				// 	_vel_sp(2) = _params.land_speed;
				// }

				if (!_control_mode.flag_control_manual_enabled && _control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_auto_enabled) {
					_vel_sp(2) = _params.land_speed;
				}

				_global_vel_sp.vx = _vel_sp(0);
				_global_vel_sp.vy = _vel_sp(1);
				_global_vel_sp.vz = _vel_sp(2);

				/* publish velocity setpoint */
				if (_global_vel_sp_pub > 0) {
					orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

				} else {
					_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
				}

				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled) {
					/* reset integrals if needed */
					if (_control_mode.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = false;
							float i = _params.thr_min;

							if (reset_int_z_manual) {
								i = _manual.z;

								if (i < _params.thr_min) {
									i = _params.thr_min;

								} else if (i > _params.thr_max) {
									i = _params.thr_max;
								}
							}

							thrust_int(2) = -i;
						}

					} else {
						reset_int_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					math::Vector<3> vel_err = _vel_sp - _vel;

					/* derivative of velocity error, not includes setpoint acceleration */
					math::Vector<3> vel_err_d = (_sp_move_rate - _vel).emult(_params.pos_p) - (_vel - _vel_prev) / dt;

					/* thrust vector in NED frame */
					math::Vector<3> thrust_sp = vel_err.emult(_params.vel_p) + vel_err_d.emult(_params.vel_d) + thrust_int;

					if (!_control_mode.flag_control_velocity_enabled) {
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
					}

					if (!_control_mode.flag_control_climb_rate_enabled) {
						thrust_sp(2) = 0.0f;
					}

					/* limit thrust vector and check for saturation */
					bool saturation_xy = false;
					bool saturation_z = false;

					/* limit min lift */
					float thr_min = _params.thr_min;

					if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						thr_min = 0.0f;
					}

					float tilt_max = _params.tilt_max_air;

					/* adjust limits for landing mode */
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					  	_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _params.tilt_max_land;

						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}
					}

					/* limit min lift */
					if (-thrust_sp(2) < thr_min) {
						thrust_sp(2) = -thr_min;
						saturation_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						/* limit max tilt */
						if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}

					} else {
						/* thrust compensation for altitude only control mode */
						float att_comp;

						if (PX4_R(_att.R, 2, 2) > TILT_COS_MAX) {
							att_comp = 1.0f / PX4_R(_att.R, 2, 2);

						} else if (PX4_R(_att.R, 2, 2) > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * PX4_R(_att.R, 2, 2) + 1.0f;
							saturation_z = true;

						} else {
							att_comp = 1.0f;
							saturation_z = true;
						}

						thrust_sp(2) *= att_comp;
					}

					/* limit max thrust */
					float thrust_abs = thrust_sp.length();

					if (thrust_abs > _params.thr_max) {
						if (thrust_sp(2) < 0.0f) {
							if (-thrust_sp(2) > _params.thr_max) {
								/* thrust Z component is too large, limit it */
								thrust_sp(0) = 0.0f;
								thrust_sp(1) = 0.0f;
								thrust_sp(2) = -_params.thr_max;
								saturation_xy = true;
								saturation_z = true;

							} else {
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(_params.thr_max * _params.thr_max - thrust_sp(2) * thrust_sp(2));
								float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}

						} else {
							/* Z component is negative, going down, simply limit thrust vector */
							float k = _params.thr_max / thrust_abs;
							thrust_sp *= k;
							saturation_xy = true;
							saturation_z = true;
						}

						thrust_abs = _params.thr_max;
					}

					/* update integrals */
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
					}

					/* calculate attitude setpoint from thrust vector */
					if (_control_mode.flag_control_velocity_enabled) {
						/* desired body_z axis = -normalize(thrust_vector) */
						math::Vector<3> body_x;
						math::Vector<3> body_y;
						math::Vector<3> body_z;

						if (thrust_abs > SIGMA) {
							body_z = -thrust_sp / thrust_abs;

						} else {
							/* no thrust, set Z axis to safe value */
							body_z.zero();
							body_z(2) = 1.0f;
						}

						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);

						if (fabsf(body_z(2)) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							body_x = y_C % body_z;

							/* keep nose to front while inverted upside down */
							if (body_z(2) < 0.0f) {
								body_x = -body_x;
							}

							body_x.normalize();

						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							body_x.zero();
							body_x(2) = 1.0f;
						}

						/* desired body_y axis */
						body_y = body_z % body_x;

						/* fill rotation matrix */
						for (int i = 0; i < 3; i++) {
							R(i, 0) = body_x(i);
							R(i, 1) = body_y(i);
							R(i, 2) = body_z(i);
						}

						/* copy rotation matrix to attitude setpoint topic */
						memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
						_att_sp.R_valid = true;

						/* copy quaternion setpoint to attitude setpoint topic */
						math::Quaternion q_sp;
						q_sp.from_dcm(R);
						memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));

						/* calculate euler angles, for logging only, must not be used for control */
						math::Vector<3> euler = R.to_euler();
						_att_sp.roll_body = euler(0);
						_att_sp.pitch_body = euler(1);
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

					} else if (!_control_mode.flag_control_manual_enabled) {
						/* autonomous altitude control without position control (failsafe landing),
						 * force level attitude, don't change yaw */
						R.from_euler(0.0f, 0.0f, _att_sp.yaw_body);

						/* copy rotation matrix to attitude setpoint topic */
						memcpy(&_att_sp.R_body[0], R.data, sizeof(_att_sp.R_body));
						_att_sp.R_valid = true;

						/* copy quaternion setpoint to attitude setpoint topic */
						math::Quaternion q_sp;
						q_sp.from_dcm(R);
						memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));

						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
					}

					_att_sp.thrust = thrust_abs;

					/* save thrust setpoint for logging */
					_local_pos_sp.acc_x = thrust_sp(0);
					_local_pos_sp.acc_y = thrust_sp(1);
					_local_pos_sp.acc_z = thrust_sp(2);

					_att_sp.timestamp = hrt_absolute_time();


				} else {
					reset_int_z = true;
				}
			}

			/* fill local position, velocity and thrust setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;
			_local_pos_sp.vx = _vel_sp(0);
			_local_pos_sp.vy = _vel_sp(1);
			_local_pos_sp.vz = _vel_sp(2);

			/* publish local position setpoint */
			if (_local_pos_sp_pub > 0) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);
			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

		} else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_pos_sp = true;
			_mode_auto = false;
			reset_int_z = true;
			reset_int_xy = true;
		}

		/* generate attitude setpoint from manual controls */
		if(_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {

			/* reset yaw setpoint to current position if needed */
			if (reset_yaw_sp) {
				reset_yaw_sp = false;
				_att_sp.yaw_body = _att.yaw;
			}

			/* do not move yaw while arming */
			else if (_manual.z > 0.1f)
			{
				const float yaw_offset_max = _params.man_yaw_max / _params.mc_att_yaw_p;

				_att_sp.yaw_sp_move_rate = _manual.r * _params.man_yaw_max;
				float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
				float yaw_offs = _wrap_pi(yaw_target - _att.yaw);

				// If the yaw offset became too big for the system to track stop
				// shifting it
				if (fabsf(yaw_offs) < yaw_offset_max) {
					_att_sp.yaw_body = yaw_target;
				}
			}

			/* control roll and pitch directly if we no aiding velocity controller is active */
			if (!_control_mode.flag_control_velocity_enabled) {
				_att_sp.roll_body = _manual.y * _params.man_roll_max;
				_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;
			}

			/* control throttle directly if no climb rate controller is active */
			if (!_control_mode.flag_control_climb_rate_enabled) {
				_att_sp.thrust = math::min(_manual.z, _manual_thr_max.get());

				/* enforce minimum throttle if not landed */
				if (!_vehicle_status.condition_landed) {
					_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
				}
			}

			/* construct attitude setpoint rotation matrix */
			math::Matrix<3,3> R_sp;
			R_sp.from_euler(_att_sp.roll_body,_att_sp.pitch_body,_att_sp.yaw_body);
			memcpy(&_att_sp.R_body[0], R_sp.data, sizeof(_att_sp.R_body));

			/* copy quaternion setpoint to attitude setpoint topic */
			math::Quaternion q_sp;
			q_sp.from_dcm(R_sp);
			memcpy(&_att_sp.q_d[0], &q_sp.data[0], sizeof(_att_sp.q_d));
			_att_sp.timestamp = hrt_absolute_time();
		}
		else {
			reset_yaw_sp = true;
		}

		/* update previous velocity for velocity controller D part */
		_vel_prev = _vel;

		/* publish attitude setpoint
		 * Do not publish if offboard is enabled but position/velocity control is disabled,
		 * in this case the attitude setpoint is published by the mavlink app
		 */
		if (!(_control_mode.flag_control_offboard_enabled &&
					!(_control_mode.flag_control_position_enabled ||
						_control_mode.flag_control_velocity_enabled))) {
			if (_att_sp_pub > 0 && _vehicle_status.is_rotary_wing) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);
			} else if (_att_sp_pub <= 0 && _vehicle_status.is_rotary_wing){
				_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled && !_control_mode.flag_control_climb_rate_enabled;
	}

	warnx("stopped");
	mavlink_log_info(_mavlink_fd, "[mpc] stopped");

	_control_task = -1;
	_exit(0);
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("mc_pos_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       1500,
				       (main_t)&MulticopterPositionControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: mc_pos_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr) {
			errx(1, "already running");
		}

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			errx(1, "not running");
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

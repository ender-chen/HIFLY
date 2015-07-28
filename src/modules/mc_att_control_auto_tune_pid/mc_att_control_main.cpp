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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_virtual_fw.h>
#include <uORB/topics/actuator_controls_virtual_mc.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <mavlink/mavlink_log.h>

#include <fcntl.h>
/**
 * Multicopter attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[]);

#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f

#define AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS  500000    // restart tuning if pilot has left sticks in middle for 2 seconds
#define AUTOTUNE_TESTING_STEP_TIMEOUT_MS    500000    // timeout for tuning mode's testing step
#define AUTOTUNE_LEVEL_ANGLE_CD             3    // angle which qualifies as level
#define AUTOTUNE_REQUIRED_LEVEL_TIME_MS     250000    // time we require the copter to be level
#define AUTOTUNE_AGGRESSIVENESS            0.1f    // tuning for 10% overshoot
 #define AUTOTUNE_RD_STEP                0.0005f    // minimum increment when increasing/decreasing Rate D term
#define AUTOTUNE_RP_STEP                 0.005f    // minimum increment when increasing/decreasing Rate P term
 #define AUTOTUNE_SP_STEP                   0.5f    // minimum increment when increasing/decreasing Stab P term
#define AUTOTUNE_SP_BACKOFF               0.75f    // Stab P gains are reduced to 75% of their maximum value discovered during tuning
#define AUTOTUNE_PI_RATIO_FOR_TESTING      0.1f    // I is set 10x smaller than P during testing
#define AUTOTUNE_RP_RATIO_FINAL            1.0f    // I is set 1x P after te sting
#define AUTOTUNE_RD_MIN                  0.002f    // minimum Rate D value
#define AUTOTUNE_RD_MAX                  0.020f    // maximum Rate D value
#define AUTOTUNE_RP_MIN                   0.01f    // minimum Rate P value
#define AUTOTUNE_RP_MAX                   0.35f    // maximum Rate P value
#define AUTOTUNE_SP_MAX                   20.0f    // maximum Stab P value
#define AUTOTUNE_SUCCESS_COUNT                4    // how many successful iterations we need to freeze at current gains

static int mavlink_fd = 0;
bool autotune_complete = 0;
static	float    tune_roll_rp, tune_roll_rd, tune_roll_sp;                   // currently being tuned parameter values
static	float    tune_pitch_rp, tune_pitch_rd, tune_pitch_sp;                // currently being tuned parameter values
float tar_yaw;
class MulticopterAttitudeControl
{
public:
	/**
	 * Constructor
	 */
	MulticopterAttitudeControl();

	/**
	 * Destructor, also kills the main task
	 */
	~MulticopterAttitudeControl();

	/**
	 * Start the multicopter attitude control task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool	_task_should_exit;		/**< if true, task_main() should exit */
	int		_control_task;			/**< task handle */

	int		_v_att_sub;				/**< vehicle attitude subscription */
	int		_v_att_sp_sub;			/**< vehicle attitude setpoint subscription */
	int		_v_rates_sp_sub;		/**< vehicle rates setpoint subscription */
	int		_v_control_mode_sub;	/**< vehicle control mode subscription */
	int		_params_sub;			/**< parameter updates subscription */
	int		_manual_control_sp_sub;	/**< manual control setpoint subscription */
	int		_armed_sub;				/**< arming status subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int 	_motor_limits_sub;		/**< motor limits subscription */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_v_rates_sp_pub;		/**< rate setpoint publication */
	orb_advert_t	_actuators_0_pub;		/**< attitude actuator controls publication */
	orb_advert_t	_controller_status_pub;	/**< controller status publication */

	orb_id_t _rates_sp_id;	/**< pointer to correct rates setpoint uORB metadata structure */
	orb_id_t _actuators_id;	/**< pointer to correct actuator controls0 uORB metadata structure */

	bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct vehicle_attitude_s			_v_att;				/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s	_v_att_sp;			/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s		_v_rates_sp;		/**< vehicle rates setpoint */
	struct manual_control_setpoint_s	_manual_control_sp;	/**< manual control setpoint */
	struct vehicle_control_mode_s		_v_control_mode;	/**< vehicle control mode */
	struct actuator_controls_s			_actuators;			/**< actuator controls */
	struct actuator_armed_s				_armed;				/**< actuator arming status */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct multirotor_motor_limits_s	_motor_limits;		/**< motor limits */
	struct mc_att_ctrl_status_s 		_controller_status; /**< controller status */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_controller_latency_perf;

	math::Vector<3>		_rates_prev;	/**< angular rates on previous step */
	math::Vector<3>		_rates_sp_prev; /**< previous rates setpoint */
	math::Vector<3>		_rates_sp;		/**< angular rates setpoint */
	math::Vector<3>		_rates_int;		/**< angular rates integral error */
	float				_thrust_sp;		/**< thrust setpoint */
	math::Vector<3>		_att_control;	/**< attitude control vector */

	math::Matrix<3, 3>  _I;				/**< identity matrix */

	bool	_reset_yaw_sp;			/**< reset yaw setpoint flag */

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;

		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */

	}		_params;

	struct {
		math::Vector<3> att_p;					/**< P gain for angular error */
		math::Vector<3> rate_p;				/**< P gain for angular rate error */
		math::Vector<3> rate_i;				/**< I gain for angular rate error */
		math::Vector<3> rate_d;				/**< D gain for angular rate error */
		math::Vector<3>	rate_ff;			/**< Feedforward gain for desired rates */
		float yaw_ff;						/**< yaw control feed-forward */

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		math::Vector<3> mc_rate_max;		/**< attitude rate limits in stabilized modes */

		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		math::Vector<3> acro_rate_max;		/**< max attitude rates in acro mode */

	}		_params_orig;

	// autotune modes (high level states)
	enum AutoTuneTuneMode {
		AUTOTUNE_MODE_UNINITIALISED = 0,          // autotune has never been run
		AUTOTUNE_MODE_TUNING = 1,               // autotune is testing gains
		AUTOTUNE_MODE_SUCCESS = 2,              // tuning has completed, user is flight testing the new gains
		AUTOTUNE_MODE_FAILED = 3,               // tuning has failed, user is flying on original gains
	};

	// steps performed while in the tuning mode
	enum AutoTuneStepType {
		AUTOTUNE_STEP_WAITING_FOR_LEVEL = 0,    // autotune is waiting for vehicle to return to level before beginning the next twitch
		AUTOTUNE_STEP_TWITCHING = 1,            // autotune has begun a twitch and is watching the resulting vehicle movement
		AUTOTUNE_STEP_UPDATE_GAINS = 2          // autotune has completed a twitch and is updating the gains based on the results
	};

	// things that can be tuned
	enum AutoTuneAxisType {
		AUTOTUNE_AXIS_ROLL = 0,                 // roll axis is being tuned (either angle or rate)
		AUTOTUNE_AXIS_PITCH = 1                 // pitch axis is being tuned (either angle or rate)
	};

	// mini steps performed while in Tuning mode, Testing step
	enum AutoTuneTuneType {
		AUTOTUNE_TYPE_RD_UP = 0,                // rate D is being tuned up
		AUTOTUNE_TYPE_RD_DOWN = 1,              // rate D is being tuned down
		AUTOTUNE_TYPE_RP_UP = 2,                // rate P is being tuned up
		AUTOTUNE_TYPE_SP_UP = 3                 // angle P is being tuned up
	};

	// autotune_state_struct - hold state flags
	struct autotune_state_struct {
		AutoTuneTuneMode    mode                : 2;    // see AutoTuneTuneMode for what modes are allowed
		uint8_t             pilot_override      : 1;    // 1 = pilot is overriding controls so we suspend tuning temporarily
		AutoTuneAxisType    axis                : 1;    // see AutoTuneAxisType for which things can be tuned
		uint8_t             positive_direction  : 1;    // 0 = tuning in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
		AutoTuneStepType    step                : 2;    // see AutoTuneStepType for what steps are performed
		AutoTuneTuneType    tune_type           : 2;    // see AutoTuneTuneType
	} autotune_state;

	uint32_t autotune_override_time;                                     // the last time the pilot overrode the controls
	float    autotune_test_min;                                          // the minimum angular rate achieved during TESTING_RATE step
	float    autotune_test_max;                                          // the maximum angular rate achieved during TESTING_RATE step
	uint32_t autotune_step_start_time;                                   // start time of current tuning step (used for timeout checks)
	int8_t   autotune_counter;                                           // counter for tuning gains

	void autotune_init();

	void autotune_stop();

	bool autotune_start(bool ignore_checks);

	void autotune_backup_gains_and_initialise();

	void autotune_load_orig_gains();

	void autotune_load_tuned_gains();

	void autotune_load_intra_test_gains();

	void autotune_load_twitch_gains();

	void autotune_save_tuning_gains();
	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	/**
	 * Check for parameter update and handle it.
	 */
	void		parameter_update_poll();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for attitude setpoint updates.
	 */
	void		vehicle_attitude_setpoint_poll();

	/**
	 * Check for rates setpoint updates.
	 */
	void		vehicle_rates_setpoint_poll();

	/**
	 * Check for arming status updates.
	 */
	void		arming_status_poll();

	/**
	 * Attitude controller.
	 */
	void		control_attitude(float dt);

	/**
	 * Attitude rates controller.
	 */
	void		control_attitude_rates(float dt);

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for vehicle motor limits status.
	 */
	void		vehicle_motor_limits_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude control task.
	 */
	void		task_main();
};

namespace mc_att_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MulticopterAttitudeControl	*g_control;
}

MulticopterAttitudeControl::MulticopterAttitudeControl() :

	_task_should_exit(false),
//	_first_autotune(false),
	_control_task(-1),

/* subscriptions */
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),

/* publications */
	_att_sp_pub(-1),
	_v_rates_sp_pub(-1),
	_actuators_0_pub(-1),
	_controller_status_pub(-1),
	_rates_sp_id(0),
	_actuators_id(0),

	_actuators_0_circuit_breaker_enabled(false),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

{
	memset(&_v_att, 0, sizeof(_v_att));
	memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_actuators, 0, sizeof(_actuators));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status,0,sizeof(_controller_status));
	_vehicle_status.is_rotary_wing = true;

	_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.man_roll_max = 0.0f;
	_params.man_pitch_max = 0.0f;
	_params.man_yaw_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.acro_rate_max.zero();

	_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();

	_I.identity();

	_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.man_roll_max	= 	param_find("MC_MAN_R_MAX");
	_params_handles.man_pitch_max	= 	param_find("MC_MAN_P_MAX");
	_params_handles.man_yaw_max		= 	param_find("MC_MAN_Y_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max		= 	param_find("MC_ACRO_Y_MAX");

	/* fetch initial parameter values */
	parameters_update();
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
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

	mc_att_control::g_control = nullptr;
}

// autotune_init - should be called when autotune mode is selected
void MulticopterAttitudeControl::autotune_init()
{
	bool success = true;

	switch (autotune_state.mode) {
		case AUTOTUNE_MODE_FAILED:
			// autotune has been run but failed so reset state to uninitialised
			autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
			// no break to allow fall through to restart the tuning
		case AUTOTUNE_MODE_UNINITIALISED:
			// autotune has never been run
			//success = autotune_start(false);
			if (success) {
				// so store current gains as original gains
				autotune_backup_gains_and_initialise();
				// advance mode to tuning
				autotune_state.mode = AUTOTUNE_MODE_TUNING;
			}
			break;

		case AUTOTUNE_MODE_TUNING:
			// we are restarting tuning after the user must have switched ch7/ch8 off so we restart tuning where we left off
			//success = autotune_start(false);
			if (success) {
				// reset gains to tuning-start gains (i.e. low I term)
				autotune_load_intra_test_gains();
			}
			break;
		case AUTOTUNE_MODE_SUCCESS:
			// we have completed a tune and the pilot wishes to test the new gains in the current flight mode
			// so simply apply tuning gains (i.e. do not change flight mode)
			autotune_load_tuned_gains();
			break;
	}
}

void MulticopterAttitudeControl::autotune_stop()
{
	// set gains to their original values
	autotune_load_orig_gains();

}

bool MulticopterAttitudeControl::autotune_start(bool ignore_checks)
{
	// only allow flip from Stabilize or AltHold flight modes
	 if (!_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled && !_v_control_mode.flag_control_altitude_enabled && !_v_control_mode.flag_control_climb_rate_enabled && _v_control_mode.flag_control_position_enabled) {
		 return false;
	 }

	 // ensure throttle is above zero
	 if (_manual_control_sp.z <= 0) {
		 return false;
	 }

	 // ensure we are flying
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		return false;
	}

	return true;
}

void MulticopterAttitudeControl::autotune_backup_gains_and_initialise()
{
	// initialise state because this is our first time
	autotune_state.axis = AUTOTUNE_AXIS_ROLL;
	autotune_state.positive_direction = false;
	autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
	autotune_step_start_time = hrt_absolute_time();
	autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

	float v;
	// backup original pids
	/* roll gains */
	param_get(_params_handles.roll_p, &v);	
	_params_orig.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params_orig.rate_p(0) = v;
	param_get(_params_handles.roll_rate_i, &v);
	_params_orig.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params_orig.rate_d(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);	
	_params_orig.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params_orig.rate_p(1) = v;
	param_get(_params_handles.pitch_rate_i, &v);
	_params_orig.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params_orig.rate_d(1) = v;

	// initialise tuned pid values
	tune_roll_rp = _params_orig.rate_p(0);
	tune_roll_rd = _params_orig.rate_d(0);
	tune_roll_sp = _params_orig.att_p(0);
	tune_pitch_rp = _params_orig.rate_p(1);
	tune_pitch_rd = _params_orig.rate_d(1);
	tune_pitch_sp = _params_orig.att_p(1);
			 mavlink_and_console_log_critical(mavlink_fd,"and_initialise():une_roll_sp: %8.4f", (double)tune_roll_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "and_initialise();tune_roll_rp: %8.4f",(double) tune_roll_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "and_initialise():tune_roll_rd: %8.4f", (double)tune_roll_rd);
}

void MulticopterAttitudeControl::autotune_load_orig_gains()
{
	float v;
	// sanity check the original gains
	if (_params_orig.rate_p(0) > 0 && _params_orig.rate_p(1) > 0) {
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_orig_gains():tune_roll_sp: %8.4f", (double)tune_roll_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_orig_gains():tune_roll_rp: %8.4f", (double)tune_roll_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_orig_gains();tune_roll_rd: %8.4f", (double)tune_roll_rd);
		//roll 
		v = _params_orig.att_p(0);
		param_set(_params_handles.roll_p, &v);
		v = _params_orig.rate_p(0);
		param_set(_params_handles.roll_rate_p, &v);
		v = _params_orig.rate_d(0);
		param_set(_params_handles.roll_rate_d, &v);
		v = _params_orig.rate_i(0);
		param_set(_params_handles.roll_rate_i, &v);

		//pitch
		v = _params_orig.att_p(1);
		param_set(_params_handles.pitch_p, &v);
		v = _params_orig.rate_p(1);
		param_set(_params_handles.pitch_rate_p, &v);
		v = _params_orig.rate_d(1);
		param_set(_params_handles.pitch_rate_d, &v);
		v = _params_orig.rate_i(1);
		param_set(_params_handles.pitch_rate_i, &v);
	}
}

void MulticopterAttitudeControl::autotune_load_tuned_gains()
{
	float v;
	// sanity check the gains
	if (tune_roll_rp > 0 && tune_pitch_rp > 0) {
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_tuned_gains():tune_roll_sp: %8.4f", (double)tune_roll_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_tuned_gains():tune_roll_rp: %8.4f", (double)tune_roll_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_tuned_gains():tune_roll_rd: %8.4f", (double)tune_roll_rd);
		//roll 
		v = tune_roll_sp;
		param_set(_params_handles.roll_p, &v);
		v = tune_roll_rp;
		param_set(_params_handles.roll_rate_p, &v);
		v = tune_roll_rd;
		param_set(_params_handles.roll_rate_d, &v);
		v = tune_roll_rp*AUTOTUNE_RP_RATIO_FINAL;
		param_set(_params_handles.roll_rate_i, &v);

		//pitch
		v = tune_pitch_sp;
		param_set(_params_handles.pitch_p, &v);
		v = tune_pitch_rp;
		param_set(_params_handles.pitch_rate_p, &v);
		v = tune_pitch_rd;
		param_set(_params_handles.pitch_rate_d, &v);
		v = tune_pitch_rp*AUTOTUNE_RP_RATIO_FINAL;
		param_set(_params_handles.pitch_rate_i, &v);
	}else{
	}
}

void MulticopterAttitudeControl::autotune_load_intra_test_gains()
{
	float v;
	if (_params_orig.rate_p(0) > 0 && _params_orig.rate_p(1) > 0) {
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_intra_test_gains():tune_roll_sp: %8.4f", (double)tune_roll_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_intra_test_gains():tune_roll_rp: %8.4f", (double)tune_roll_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_intra_test_gains();tune_roll_rd: %8.4f", (double)tune_roll_rd);
		//roll 
		v = _params_orig.att_p(0);
		param_set(_params_handles.roll_p, &v);
		v = _params_orig.rate_p(0);
		param_set(_params_handles.roll_rate_p, &v);
		v = _params_orig.rate_d(0);
		param_set(_params_handles.roll_rate_d, &v);
		v = _params_orig.rate_p(0)*AUTOTUNE_PI_RATIO_FOR_TESTING;
		param_set(_params_handles.roll_rate_i, &v);

		//pitch
		v = _params_orig.att_p(1);
		param_set(_params_handles.pitch_p, &v);
		v = _params_orig.rate_p(1);
		param_set(_params_handles.pitch_rate_p, &v);
		v = _params_orig.rate_d(1);
		param_set(_params_handles.pitch_rate_d, &v);
		v = _params_orig.rate_p(1)*AUTOTUNE_PI_RATIO_FOR_TESTING;
		param_set(_params_handles.pitch_rate_i, &v);

	}else{
	}
}

void MulticopterAttitudeControl::autotune_load_twitch_gains()
{
	float v;
	if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
		if (tune_roll_rp > 0) {
			//roll 
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_twitch_gains():tune_roll_sp: %8.4f", (double)tune_roll_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_twitch_gains():tune_roll_rp: %8.4f", (double)tune_roll_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_twitch_gains():tune_roll_rd: %8.4f", (double)tune_roll_rd);
			v = tune_roll_sp;
			param_set(_params_handles.roll_p, &v);
			v = tune_roll_rp;
			param_set(_params_handles.roll_rate_p, &v);
			v = tune_roll_rd;
			param_set(_params_handles.roll_rate_d, &v);
			v = tune_roll_rp*AUTOTUNE_RP_RATIO_FINAL;
			param_set(_params_handles.roll_rate_i, &v);
		}else{
		}
	}else{
		if (tune_pitch_rp > 0) {
			//pitch
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_twitch_gains():tune_pitch_sp: %8.4f", (double)tune_pitch_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_twitch_gains():tune_pitch_rp: %8.4f", (double)tune_pitch_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_load_twitch_gains():tune_pitch_rd: %8.4f", (double)tune_pitch_rd);
			v = tune_pitch_sp;
			param_set(_params_handles.pitch_p, &v);
			v = tune_pitch_rp;
			param_set(_params_handles.pitch_rate_p, &v);
			v = tune_pitch_rd;
			param_set(_params_handles.pitch_rate_d, &v);
			v = tune_pitch_rp*AUTOTUNE_RP_RATIO_FINAL;
			param_set(_params_handles.pitch_rate_i, &v);
		}else{
		}
	}
}

void MulticopterAttitudeControl::autotune_save_tuning_gains()
{
	float v;
	// if we successfully completed tuning
	if (autotune_state.mode == AUTOTUNE_MODE_SUCCESS) {
		// sanity check the rate P values
		if (tune_roll_rp > 0 && tune_pitch_rp > 0) {
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_save_tuning_gains():tune_roll_sp: %8.4f", (double)tune_roll_sp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_save_tuning_gains():tune_roll_rp: %8.4f", (double)tune_roll_rp);
			 mavlink_and_console_log_critical(mavlink_fd, "autotune_save_tuning_gains():tune_roll_rd: %8.4f", (double)tune_roll_rd);
			//roll 
			v = tune_roll_sp;
			param_set(_params_handles.roll_p, &v);
			v = tune_roll_rp;
			param_set(_params_handles.roll_rate_p, &v);
			v = tune_roll_rd;
			param_set(_params_handles.roll_rate_d, &v);
			v = tune_roll_rp*AUTOTUNE_RP_RATIO_FINAL;
			param_set(_params_handles.roll_rate_i, &v);

			//pitch
			v = tune_pitch_sp;
			param_set(_params_handles.pitch_p, &v);
			v = tune_pitch_rp;
			param_set(_params_handles.pitch_rate_p, &v);
			v = tune_pitch_rd;
			param_set(_params_handles.pitch_rate_d, &v);
			v = tune_pitch_rp*AUTOTUNE_RP_RATIO_FINAL;
			param_set(_params_handles.pitch_rate_i, &v);

			param_get(_params_handles.roll_rate_p, &v);
			_params_orig.rate_p(0) = v; 
			param_get(_params_handles.roll_rate_d, &v);
			_params_orig.rate_d(0) = v;
			param_get(_params_handles.roll_rate_i, &v); 
			_params_orig.rate_i(0) = v;
			param_get(_params_handles.roll_p, &v);
			_params_orig.att_p(0) = v;
			param_get(_params_handles.pitch_rate_p, &v);
			_params_orig.rate_p(1) = v;
			param_get(_params_handles.pitch_rate_d, &v);
			_params_orig.rate_d(1) = v;
			param_get(_params_handles.pitch_rate_i, &v); 
			_params_orig.rate_i(1) = v;
			param_get(_params_handles.pitch_p, &v);
			_params_orig.att_p(1) = v;
		}else{
		}
	}
}

int
MulticopterAttitudeControl::parameters_update()
{
	float v;

	/* roll gains */
	param_get(_params_handles.roll_p, &v);
	_params.att_p(0) = v;
	param_get(_params_handles.roll_rate_p, &v);
	_params.rate_p(0) = v;
	param_get(_params_handles.roll_rate_i, &v);
	_params.rate_i(0) = v;
	param_get(_params_handles.roll_rate_d, &v);
	_params.rate_d(0) = v;
	param_get(_params_handles.roll_rate_ff, &v);
	_params.rate_ff(0) = v;

	/* pitch gains */
	param_get(_params_handles.pitch_p, &v);
	_params.att_p(1) = v;
	param_get(_params_handles.pitch_rate_p, &v);
	_params.rate_p(1) = v;
	param_get(_params_handles.pitch_rate_i, &v);
	_params.rate_i(1) = v;
	param_get(_params_handles.pitch_rate_d, &v);
	_params.rate_d(1) = v;
	param_get(_params_handles.pitch_rate_ff, &v);
	_params.rate_ff(1) = v;

	/* yaw gains */
	param_get(_params_handles.yaw_p, &v);
	_params.att_p(2) = v;
	param_get(_params_handles.yaw_rate_p, &v);
	_params.rate_p(2) = v;
	param_get(_params_handles.yaw_rate_i, &v);
	_params.rate_i(2) = v;
	param_get(_params_handles.yaw_rate_d, &v);
	_params.rate_d(2) = v;
	param_get(_params_handles.yaw_rate_ff, &v);
	_params.rate_ff(2) = v;

	param_get(_params_handles.yaw_ff, &_params.yaw_ff);

	/* angular rate limits */
	param_get(_params_handles.roll_rate_max, &_params.roll_rate_max);
	_params.mc_rate_max(0) = math::radians(_params.roll_rate_max);
	param_get(_params_handles.pitch_rate_max, &_params.pitch_rate_max);
	_params.mc_rate_max(1) = math::radians(_params.pitch_rate_max);
	param_get(_params_handles.yaw_rate_max, &_params.yaw_rate_max);
	_params.mc_rate_max(2) = math::radians(_params.yaw_rate_max);

	/* manual attitude control scale */
	param_get(_params_handles.man_roll_max, &_params.man_roll_max);
	param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
	param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
	_params.man_roll_max = math::radians(_params.man_roll_max);
	_params.man_pitch_max = math::radians(_params.man_pitch_max);
	_params.man_yaw_max = math::radians(_params.man_yaw_max);

	/* manual rate control scale and auto mode roll/pitch rate limits */
	param_get(_params_handles.acro_roll_max, &v);
	_params.acro_rate_max(0) = math::radians(v);
	param_get(_params_handles.acro_pitch_max, &v);
	_params.acro_rate_max(1) = math::radians(v);
	param_get(_params_handles.acro_yaw_max, &v);
	_params.acro_rate_max(2) = math::radians(v);

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled("CBRK_RATE_CTRL", CBRK_RATE_CTRL_KEY);

	return OK;
}

void
MulticopterAttitudeControl::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void
MulticopterAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check if vehicle control mode has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

void
MulticopterAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_attitude_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _v_att_sp_sub, &_v_att_sp);
	}
}

void
MulticopterAttitudeControl::vehicle_rates_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_v_rates_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_rates_setpoint), _v_rates_sp_sub, &_v_rates_sp);
	}
}

void
MulticopterAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void
MulticopterAttitudeControl::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(mc_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_mc);
			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);
			}
		}
	}
}

void
MulticopterAttitudeControl::vehicle_motor_limits_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_motor_limits_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(multirotor_motor_limits), _motor_limits_sub, &_motor_limits);
	}
}

/**
 * Attitude controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void
MulticopterAttitudeControl::control_attitude(float dt)
{
	float rotation_rate;        // rotation rate in radians/second
	int32_t lean_angle;
	float autotune_target_angle_cd = math::radians(20.0f);
	float autotune_target_rate_cds = math::radians(90.0f);

	vehicle_attitude_setpoint_poll();

	_thrust_sp = _v_att_sp.thrust;

	/* construct attitude setpoint rotation matrix */
	math::Matrix<3, 3> R_sp;
	if((fabs(_manual_control_sp.x) > 0.01) || (fabs(_manual_control_sp.y) > 0.01) || _vehicle_status.nav_state != vehicle_status_s::MAIN_STATE_ALTCTL || autotune_complete || !_armed.armed) {
		if(!autotune_state.pilot_override) {
			autotune_state.pilot_override = true;
			// set gains to their original values
			autotune_load_orig_gains();
		}
			//reset pilot override time
			autotune_override_time = hrt_absolute_time();
	}else if(autotune_state.pilot_override)
	{
		// check if we should resume tuning after pilot's override
		if (hrt_absolute_time() - autotune_override_time > AUTOTUNE_PILOT_OVERRIDE_TIMEOUT_MS) {
			autotune_state.pilot_override = false;
			tar_yaw = _v_att.yaw;
			// set gains to their intra-test values (which are very close to the original gains)
			autotune_load_intra_test_gains();
			autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
			autotune_step_start_time = hrt_absolute_time();
		}
	}
		// if pilot override call attitude controller
	if (autotune_state.pilot_override){//|| (!_v_control_mode.flag_control_manual_enabled && !_v_control_mode.flag_control_attitude_enabled && !_v_control_mode.flag_control_altitude_enabled && !_v_control_mode.flag_control_climb_rate_enabled && _v_control_mode.flag_control_position_enabled)) 
		 /* construct attitude setpoint rotation matrix */
		R_sp.set(_v_att_sp.R_body);

		/* rotation matrix for current state */
		math::Matrix<3, 3> R;
		R.set(_v_att.R);

		/* all input data is ready, run controller itself */
		/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
		math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
		math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

		/* axis and sin(angle) of desired rotation */
		math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

		/* calculate angle error */
		float e_R_z_sin = e_R.length();
		float e_R_z_cos = R_z * R_sp_z;

		/* calculate weight for yaw control */
		float yaw_w = R_sp(2, 2) * R_sp(2, 2);

		/* calculate rotation matrix after roll/pitch only rotation */
		math::Matrix<3, 3> R_rp;

		if (e_R_z_sin > 0.0f) {
			/* get axis-angle representation */
			float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
			math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

			e_R = e_R_z_axis * e_R_z_angle;

			/* cross product matrix for e_R_axis */
			math::Matrix<3, 3> e_R_cp;
			e_R_cp.zero();
			e_R_cp(0, 1) = -e_R_z_axis(2);
			e_R_cp(0, 2) = e_R_z_axis(1);
			e_R_cp(1, 0) = e_R_z_axis(2);
			e_R_cp(1, 2) = -e_R_z_axis(0);
			e_R_cp(2, 0) = -e_R_z_axis(1);
			e_R_cp(2, 1) = e_R_z_axis(0);
																																			/* rotation matrix for roll/pitch only rotation */
			R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));
		} else {
			/* zero roll/pitch rotation */
			R_rp = R;
		}

		/* R_rp and R_sp has the same Z axis, calculate yaw error */
		math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
		math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
		e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

		if (e_R_z_cos < 0.0f) {
			/* for large thrust vector rotations use another rotation method:
			 *		 * calculate angle and axis for R -> R_sp rotation directly */
			 math::Quaternion q;
			 q.from_dcm(R.transposed() * R_sp);
			 math::Vector<3> e_R_d = q.imag();
			 e_R_d.normalize();
			 e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

			/* use fusion of Z axis based rotation and direct rotation */
			 float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
			 e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
		}

		/* calculate angular rates setpoint */
		_rates_sp = _params.att_p.emult(e_R);

		/* limit rates */
		for (int i = 0; i < 3; i++) {
			_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
		}

		/* feed forward yaw setpoint rate */
		_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;
	 }else{
		 // somehow get attitude requests from autotuning

		//R_sp.set(_v_att_sp.R_body);
		// check tuning step
		switch (autotune_state.step) {
			case AUTOTUNE_STEP_WAITING_FOR_LEVEL:
				//hold level attitude
				R_sp.from_euler(0.0f, 0.0f, tar_yaw);

//				if ((fabs(math::degrees(_v_att.roll)) > AUTOTUNE_LEVEL_ANGLE_CD) || (fabs(math::degrees(_v_att.pitch)) > AUTOTUNE_LEVEL_ANGLE_CD)) {
//					autotune_step_start_time = hrt_absolute_time();
//				}
//
				if (hrt_absolute_time() - autotune_step_start_time >= AUTOTUNE_REQUIRED_LEVEL_TIME_MS) {
					// init variables for next step
					autotune_state.step = AUTOTUNE_STEP_TWITCHING;
					autotune_step_start_time = hrt_absolute_time();
					autotune_test_max = 0;
					autotune_test_min = 0;
					rotation_rate = 0;
					// set gains to their to-be-tested values
					autotune_load_twitch_gains();
				}
				break;

			case AUTOTUNE_STEP_TWITCHING:

				if(autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP) {
					// Testing increasing stabilize P gain so will set lean angle target
					if(autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						//request roll to 20deg
						if(autotune_state.positive_direction) {
							R_sp.from_euler(autotune_target_angle_cd, 0.0f, tar_yaw);
						}else{
							R_sp.from_euler(-autotune_target_angle_cd, 0.0f, tar_yaw);
						}
					}else{
						//request pitch to 20deg
						if(autotune_state.positive_direction) {
							R_sp.from_euler(0.0f, autotune_target_angle_cd, tar_yaw);
						}else{
							R_sp.from_euler(0.0f, -autotune_target_angle_cd, tar_yaw);
						}
					}
			}else{
				// Testing rate P and D gains so will set body-frame rate targets
				if(autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
					// override body-frame roll rate (rate controller will use existing pitch and yaw body-frame rates and convert to motor outputs)
					if (autotune_state.positive_direction) {
						_rates_sp(0) = autotune_target_rate_cds;
						_rates_sp(1) = 0;
						_rates_sp(2) = 0;
					}else{
						_rates_sp(0) = -autotune_target_rate_cds;
						_rates_sp(1) = 0;
						_rates_sp(2) = 0;
					}
				}else{
					// override body-frame pitch rate (rate controller will use existing roll and yaw body-frame rates and convert to motor outputs)
					if (autotune_state.positive_direction) {
						_rates_sp(0) = 0;
						_rates_sp(1) = autotune_target_rate_cds;
						_rates_sp(2) = 0;
					}else{
						_rates_sp(0) = 0;
						_rates_sp(1) = -autotune_target_rate_cds;
						_rates_sp(2) = 0;
					}
				}
			}

			// capture this iterations rotation rate and lean angle
			if(autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
				rotation_rate =fabs( _v_att.rollspeed);
				lean_angle = fabs(_v_att.roll);
			}else{
				rotation_rate = fabs(_v_att.pitchspeed);
				lean_angle = fabs(_v_att.pitch);
			}

			// compare rotation rate or lean angle to previous iterations of this testing step
			if(autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP) {
				//when tuning stabilize P gain, capture the max lean angle
				if (lean_angle > autotune_test_max) {
					autotune_test_max = lean_angle;
					autotune_test_min = lean_angle;
				}

				// capture min lean angle
				if(lean_angle < autotune_test_min && autotune_test_max > autotune_target_angle_cd*(1-AUTOTUNE_AGGRESSIVENESS)) {
					autotune_test_min = lean_angle;
				}
			}else{
				// when tuning rate P and D gain, capture max rotation rate
				if(rotation_rate > autotune_test_max){
					autotune_test_max = rotation_rate;
					autotune_test_min = rotation_rate;
				}

				 // capture min rotation rate after the rotation rate has peaked (aka "bounce back rate")
				 if(rotation_rate < autotune_test_min && autotune_test_max > autotune_target_rate_cds*0.5f) {
					 autotune_test_min = rotation_rate;
				 }
			}

			// check for end of test conditions
			// testing step time out after 0.5sec
			if(hrt_absolute_time() - autotune_step_start_time >= AUTOTUNE_TESTING_STEP_TIMEOUT_MS) {
				autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
			}	
			if(autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP) {
				 // stabilize P testing completes when the lean angle reaches 22deg or the vehicle has rotated 22deg
				 if((lean_angle >= autotune_target_angle_cd*(1+AUTOTUNE_AGGRESSIVENESS)) || (autotune_test_max-autotune_test_min > autotune_target_angle_cd*AUTOTUNE_AGGRESSIVENESS)) {
					 autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
				 }
			}else{
				// rate P and D testing completes when the vehicle reaches 20deg
				if(lean_angle >= autotune_target_angle_cd){
					autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
				}
				// rate P and D testing can also complete when the "bounce back rate" isat least 9deg less than the maximum rotation rate
				if(autotune_state.tune_type == AUTOTUNE_TYPE_RD_UP || autotune_state.tune_type == AUTOTUNE_TYPE_RD_DOWN) {
					if(autotune_test_max-autotune_test_min > autotune_target_rate_cds*AUTOTUNE_AGGRESSIVENESS) {
						autotune_state.step = AUTOTUNE_STEP_UPDATE_GAINS;
					}
				}
			}
			break;

		case AUTOTUNE_STEP_UPDATE_GAINS:
			// set gains to their intra-test values (which are very close to the original gains)
			//
			autotune_load_intra_test_gains();
			// Check results after mini-step to increase rate D gain
			if(autotune_state.tune_type == AUTOTUNE_TYPE_RD_UP) {
				// when tuning the rate D gain
				if(autotune_test_max > autotune_target_rate_cds) {
					// if max rotation rate was higher than target, reduce rate P
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_rp -= AUTOTUNE_RP_STEP;
						// abandon tuning if rate P falls below 0.01
						if(tune_roll_rp < AUTOTUNE_RP_MIN) {
							tune_roll_rp = AUTOTUNE_RP_MIN;
							autotune_counter = AUTOTUNE_SUCCESS_COUNT;
						}
					}else{
						tune_pitch_rp -= AUTOTUNE_RP_STEP;
						// abandon tuning if rate P falls below 0.01
						if( tune_pitch_rp < AUTOTUNE_RP_MIN ) {
							tune_pitch_rp = AUTOTUNE_RP_MIN;
							autotune_counter = AUTOTUNE_SUCCESS_COUNT;
						}
					}
					// if maximum rotation rate was less than 80% of requested rate increase rate P
				}else if(autotune_test_max < autotune_target_rate_cds*(1-AUTOTUNE_AGGRESSIVENESS*2.0f) && ((autotune_state.axis == AUTOTUNE_AXIS_ROLL && tune_roll_rp <= AUTOTUNE_RP_MAX) || (autotune_state.axis == AUTOTUNE_AXIS_PITCH && tune_pitch_rp <= AUTOTUNE_RP_MAX))) {
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_rp += AUTOTUNE_RP_STEP*2.0f;
					}else{
						tune_pitch_rp += AUTOTUNE_RP_STEP*2.0f;
					}
				}else{
					// if "bounce back rate" if greater than 10% of requested rate (i.e.>9deg/sec) this is a good tune
					if (autotune_test_max-autotune_test_min > autotune_target_rate_cds*AUTOTUNE_AGGRESSIVENESS) {
						autotune_counter++;
					}else{
						// bounce back was too small so reduce number of good tunes
						if (autotune_counter > 0 ) {
							autotune_counter--;
						}
						// increase rate D (which should increase "bounce back rate")
						if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
							tune_roll_rd += AUTOTUNE_RD_STEP*2.0f;
							// stop tuning if we hit max D
							if (tune_roll_rd >= AUTOTUNE_RD_MAX) {
								tune_roll_rd = AUTOTUNE_RD_MAX;
								autotune_counter = AUTOTUNE_SUCCESS_COUNT;
							}
						}else{
							tune_pitch_rd += AUTOTUNE_RD_STEP*2.0f;
							// stop tuning if we hit max D
							if (tune_pitch_rd >= AUTOTUNE_RD_MAX) {
								tune_pitch_rd = AUTOTUNE_RD_MAX;
								autotune_counter = AUTOTUNE_SUCCESS_COUNT;
							}
						}
					}
				}
			// Check results after mini-step to decrease rate D gain
			}else if(autotune_state.tune_type == AUTOTUNE_TYPE_RD_DOWN) {
				if (autotune_test_max > autotune_target_rate_cds) {
					// if max rotation rate was higher than target, reduce rate P
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_rp -= AUTOTUNE_RP_STEP;
						// reduce rate D if tuning if rate P falls below 0.01
						if(tune_roll_rp < AUTOTUNE_RP_MIN) {
							tune_roll_rp = AUTOTUNE_RP_MIN;
							tune_roll_rd -= AUTOTUNE_RD_STEP;
							// stop tuning if we hit min D
							if (tune_roll_rd <= AUTOTUNE_RD_MIN) {
								tune_roll_rd = AUTOTUNE_RD_MIN;
								autotune_counter = AUTOTUNE_SUCCESS_COUNT;
							}
						}
					}else{
						tune_pitch_rp -= AUTOTUNE_RP_STEP;
						// reduce rate D if tuning if rate P falls below 0.01
						if( tune_pitch_rp < AUTOTUNE_RP_MIN ) {
							tune_pitch_rp = AUTOTUNE_RP_MIN;
							tune_pitch_rd -= AUTOTUNE_RD_STEP;
							// stop tuning if we hit min D
							if (tune_pitch_rd <= AUTOTUNE_RD_MIN) {
								tune_pitch_rd = AUTOTUNE_RD_MIN;
								autotune_counter = AUTOTUNE_SUCCESS_COUNT;
							}
						}
					}
				// if maximum rotation rate was less than 80% of requested rate increase rate P
				}else if(autotune_test_max < autotune_target_rate_cds*(1-AUTOTUNE_AGGRESSIVENESS*2.0f) && ((autotune_state.axis == AUTOTUNE_AXIS_ROLL && tune_roll_rp <= AUTOTUNE_RP_MAX) ||  (autotune_state.axis == AUTOTUNE_AXIS_PITCH && tune_pitch_rp <= AUTOTUNE_RP_MAX)) ) {
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_rp += AUTOTUNE_RP_STEP;
					}else{
						tune_pitch_rp += AUTOTUNE_RP_STEP;
					}
				}else{
					// if "bounce back rate" if less than 10% of requested rate (i.e. >9deg/sec) this is a good tune
					if (autotune_test_max-autotune_test_min < autotune_target_rate_cds*AUTOTUNE_AGGRESSIVENESS) {
						autotune_counter++;
					}else{
						// bounce back was too large so reduce number of good tunes
						if (autotune_counter > 0 ) {
							autotune_counter--;
						}
						// decrease rate D (which should decrease "bounce back rate")
						if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
							tune_roll_rd -= AUTOTUNE_RD_STEP;
							// stop tuning if we hit min D
							if (tune_roll_rd <= AUTOTUNE_RD_MIN) {
								tune_roll_rd = AUTOTUNE_RD_MIN;
								autotune_counter = AUTOTUNE_SUCCESS_COUNT;
							}
						}else{
							tune_pitch_rd -= AUTOTUNE_RD_STEP;
							// stop tuning if we hit min D
							if (tune_pitch_rd <= AUTOTUNE_RD_MIN) {
								tune_pitch_rd = AUTOTUNE_RD_MIN;
								autotune_counter = AUTOTUNE_SUCCESS_COUNT;
							}
						}
					}
				}
			// Check results after mini-step to increase rate P gain
			}else if (autotune_state.tune_type == AUTOTUNE_TYPE_RP_UP){
				// if max rotation rate greater than target, this is a good tune
				if (autotune_test_max > autotune_target_rate_cds) {
					autotune_counter++;
				}else{
					// rotation rate was too low so reduce number of good tunes
					if (autotune_counter > 0 ) {
						autotune_counter--;
					}
					// increase rate P and I gains
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_rp += AUTOTUNE_RP_STEP;
						// stop tuning if we hit max P
						if (tune_roll_rp >= AUTOTUNE_RP_MAX) {
							tune_roll_rp = AUTOTUNE_RP_MAX;
							autotune_counter = AUTOTUNE_SUCCESS_COUNT;
						}
					}else{
						tune_pitch_rp += AUTOTUNE_RP_STEP;
						// stop tuning if we hit max P
						if (tune_pitch_rp >= AUTOTUNE_RP_MAX) {
							tune_pitch_rp = AUTOTUNE_RP_MAX;
							autotune_counter = AUTOTUNE_SUCCESS_COUNT;
						}
					}
				}
			// Check results after mini-step to increase stabilize P gain
			} else if (autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP) {
				// if max angle reaches 22deg this is a successful tune
				if (autotune_test_max > autotune_target_angle_cd*(1+AUTOTUNE_AGGRESSIVENESS) || (autotune_test_max-autotune_test_min > autotune_target_angle_cd*AUTOTUNE_AGGRESSIVENESS)) {
					autotune_counter++;
				}else{
					// did not reach the target angle so this is a bad tune
					if (autotune_counter > 0 ) {
						autotune_counter--;
					}
					// increase stabilize P and I gains
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_sp += AUTOTUNE_SP_STEP;
						// stop tuning if we hit max P
						if (tune_roll_sp >= AUTOTUNE_SP_MAX) {
							tune_roll_sp = AUTOTUNE_SP_MAX;
							autotune_counter = AUTOTUNE_SUCCESS_COUNT;
						}
					}else{
						tune_pitch_sp += AUTOTUNE_SP_STEP;
						// stop tuning if we hit max P
						if (tune_pitch_sp >= AUTOTUNE_SP_MAX) {
							tune_pitch_sp = AUTOTUNE_SP_MAX;
							autotune_counter = AUTOTUNE_SUCCESS_COUNT;
						}
					}
				}
			}

			// reverse direction
			autotune_state.positive_direction = !autotune_state.positive_direction;
			// we've complete this step, finalise pids and move to next step
			if (autotune_counter >= AUTOTUNE_SUCCESS_COUNT) {
				// reset counter
				autotune_counter = 0;

				// move to the next tuning type
				if (autotune_state.tune_type < AUTOTUNE_TYPE_SP_UP) {
					autotune_state.tune_type++;
				}else{
					// we've reached the end of a D-up-down PI-up-down tune type cycle
					autotune_state.tune_type = AUTOTUNE_TYPE_RD_UP;

					// if we've just completed roll move onto pitch
					if (autotune_state.axis == AUTOTUNE_AXIS_ROLL) {
						tune_roll_sp = tune_roll_sp * AUTOTUNE_SP_BACKOFF;
						autotune_state.axis = AUTOTUNE_AXIS_PITCH;
					}else{
						tune_pitch_sp = tune_pitch_sp * AUTOTUNE_SP_BACKOFF;
						//tune_roll_sp = math::min(tune_roll_sp, tune_pitch_sp);
						//tune_pitch_sp = math::min(tune_roll_sp, tune_pitch_sp);
						// if we've just completed pitch we have successfully completed the autotune
						// change to TESTING mode to allow user to fly with new gains
						autotune_state.mode = AUTOTUNE_MODE_SUCCESS;
						autotune_save_tuning_gains();
						autotune_complete = 1;
					}
				}
			}

			// reset testing step
			autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL;
			autotune_step_start_time = hrt_absolute_time();

			break;
		}

		if(autotune_state.step == AUTOTUNE_STEP_WAITING_FOR_LEVEL || (autotune_state.step == AUTOTUNE_STEP_TWITCHING && autotune_state.tune_type == AUTOTUNE_TYPE_SP_UP)){
			/* rotation matrix for current state */
				math::Matrix<3, 3> R;
				R.set(_v_att.R);

				/* all input data is ready, run controller itself */

				/* try to move thrust vector shortest way, because yaw response is slower than roll/pitch */
				math::Vector<3> R_z(R(0, 2), R(1, 2), R(2, 2));
				math::Vector<3> R_sp_z(R_sp(0, 2), R_sp(1, 2), R_sp(2, 2));

				/* axis and sin(angle) of desired rotation */
				math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);

				/* calculate angle error */
				float e_R_z_sin = e_R.length();
				float e_R_z_cos = R_z * R_sp_z;

				/* calculate weight for yaw control */
				float yaw_w = R_sp(2, 2) * R_sp(2, 2);

				/* calculate rotation matrix after roll/pitch only rotation */
				math::Matrix<3, 3> R_rp;

				if (e_R_z_sin > 0.0f) {
					/* get axis-angle representation */
					float e_R_z_angle = atan2f(e_R_z_sin, e_R_z_cos);
					math::Vector<3> e_R_z_axis = e_R / e_R_z_sin;

					e_R = e_R_z_axis * e_R_z_angle;

					/* cross product matrix for e_R_axis */
					math::Matrix<3, 3> e_R_cp;
					e_R_cp.zero();
					e_R_cp(0, 1) = -e_R_z_axis(2);
					e_R_cp(0, 2) = e_R_z_axis(1);
					e_R_cp(1, 0) = e_R_z_axis(2);
					e_R_cp(1, 2) = -e_R_z_axis(0);
					e_R_cp(2, 0) = -e_R_z_axis(1);
					e_R_cp(2, 1) = e_R_z_axis(0);

					/* rotation matrix for roll/pitch only rotation */
					R_rp = R * (_I + e_R_cp * e_R_z_sin + e_R_cp * e_R_cp * (1.0f - e_R_z_cos));

				} else {
					/* zero roll/pitch rotation */
					R_rp = R;
				}

				/* R_rp and R_sp has the same Z axis, calculate yaw error */
				math::Vector<3> R_sp_x(R_sp(0, 0), R_sp(1, 0), R_sp(2, 0));
				math::Vector<3> R_rp_x(R_rp(0, 0), R_rp(1, 0), R_rp(2, 0));
				e_R(2) = atan2f((R_rp_x % R_sp_x) * R_sp_z, R_rp_x * R_sp_x) * yaw_w;

				if (e_R_z_cos < 0.0f) {
					/* for large thrust vector rotations use another rotation method:
					 * calculate angle and axis for R -> R_sp rotation directly */
					math::Quaternion q;
					q.from_dcm(R.transposed() * R_sp);
					math::Vector<3> e_R_d = q.imag();
					e_R_d.normalize();
					e_R_d *= 2.0f * atan2f(e_R_d.length(), q(0));

					/* use fusion of Z axis based rotation and direct rotation */
					float direct_w = e_R_z_cos * e_R_z_cos * yaw_w;
					e_R = e_R * (1.0f - direct_w) + e_R_d * direct_w;
				}

				/* calculate angular rates setpoint */
				_rates_sp = _params.att_p.emult(e_R);

				/* limit rates */
				for (int i = 0; i < 3; i++) {
					_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
				}

				/* feed forward yaw setpoint rate */
				_rates_sp(2) += _v_att_sp.yaw_sp_move_rate * yaw_w * _params.yaw_ff;

		}
	}

}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
void
MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	/* reset integral if disarmed */
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	/* current body angular rates */
	math::Vector<3> rates;
	rates(0) = _v_att.rollspeed;
	rates(1) = _v_att.pitchspeed;
	rates(2) = _v_att.yawspeed;

	/* angular rates error */
	math::Vector<3> rates_err = _rates_sp - rates;
	_att_control = _params.rate_p.emult(rates_err) + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int + _params.rate_ff.emult(_rates_sp - _rates_sp_prev) / dt;
	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	/* update integral only if not saturated on low limit and if motor commands are not saturated */
	if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit ) {
		for (int i = 0; i < 3; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
				float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

				if (isfinite(rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}
}

void
MulticopterAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	mc_att_control::g_control->task_main();
}

void
MulticopterAttitudeControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* initialize parameters cache */
	parameters_update();
	autotune_state.mode = AUTOTUNE_MODE_UNINITIALISED;
	autotune_init();

	/* wakeup source: vehicle attitude */
	struct pollfd fds[1];

	fds[0].fd = _v_att_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		/* run controller on attitude changes */
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too small (< 2ms) and too large (> 20ms) dt's */
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			/* copy attitude topic */
			orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);

			/* check for updates in other topics */
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll();

			if (_v_control_mode.flag_control_attitude_enabled) {
				control_attitude(dt);

				/* publish attitude rates setpoint */
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub > 0) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

			} else {
				/* attitude controller disabled, poll rates setpoint topic */
				if (_v_control_mode.flag_control_manual_enabled) {
					/* manual rates control - ACRO mode */
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x, _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					/* publish attitude rates setpoint */
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub > 0) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					/* attitude controller disabled, poll rates setpoint topic */
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			}

			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				/* publish actuator controls */
				_actuators.control[0] = (isfinite(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (isfinite(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (isfinite(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (isfinite(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _v_att.timestamp;

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub > 0) {
						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				/* publish controller status */
				if(_controller_status_pub > 0) {
					orb_publish(ORB_ID(mc_att_ctrl_status),_controller_status_pub, &_controller_status);
				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			}
		}

		perf_end(_loop_perf);
	}

	warnx("exit");

	_control_task = -1;
	_exit(0);
}

int
MulticopterAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("mc_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       1500,
				       (main_t)&MulticopterAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_att_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: mc_att_control {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (mc_att_control::g_control != nullptr)
			errx(1, "already running");

		mc_att_control::g_control = new MulticopterAttitudeControl;

		if (mc_att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != mc_att_control::g_control->start()) {
			delete mc_att_control::g_control;
			mc_att_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (mc_att_control::g_control == nullptr)
			errx(1, "not running");

		delete mc_att_control::g_control;
		mc_att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (mc_att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

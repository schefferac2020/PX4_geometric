/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "drews_module.hpp"

#include <iostream>

// #include <px4_platform_common/getopt.h>
// #include <px4_platform_common/log.h>
// #include <px4_platform_common/posix.h>

// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/sensor_combined.h>

// int DrewsModule::custom_command(int argc, char *argv[])
// {
// 	/*
// 	if (!is_running()) {
// 		print_usage("not running");
// 		return 1;
// 	}

// 	// additional custom commands can be handled like this:
// 	if (!strcmp(argv[0], "do-something")) {
// 		get_instance()->do_something();
// 		return 0;
// 	}
// 	 */

// 	return print_usage("unknown command");
// }

// bool DrewsModule::init()
// {
// 	return true;
// }

// int DrewsModule::task_spawn(int argc, char *argv[])
// {
// 	DrewsModule *instance = new DrewsModule(1, 0);

// 	if (instance) {
// 		_object.store(instance);
// 		_task_id = task_id_is_work_queue;

// 		if (instance->init()) {
// 			return PX4_OK;
// 		}

// 	} else {
// 		PX4_ERR("alloc failed");
// 	}

// 	delete instance;
// 	_object.store(nullptr);
// 	_task_id = -1;

// 	return PX4_ERROR;
// }

// DrewsModule::DrewsModule(int example_param, bool example_flag)
// 	: ModuleParams(nullptr), WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
// {
// }

// DrewsModule *DrewsModule::instantiate(int argc, char *argv[])
// {
// 	int example_param = 0;
// 	bool example_flag = false;
// 	bool error_flag = false;

// 	int myoptind = 1;
// 	int ch;
// 	const char *myoptarg = nullptr;

// 	// parse CLI arguments
// 	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
// 		switch (ch) {
// 		case 'p':
// 			example_param = (int)strtol(myoptarg, nullptr, 10);
// 			break;

// 		case 'f':
// 			example_flag = true;
// 			break;

// 		case '?':
// 			error_flag = true;
// 			break;

// 		default:
// 			PX4_WARN("unrecognized flag");
// 			error_flag = true;
// 			break;
// 		}
// 	}

// 	if (error_flag) {
// 		return nullptr;
// 	}

// 	DrewsModule *instance = new DrewsModule(example_param, example_flag);

// 	if (instance == nullptr) {
// 		PX4_ERR("alloc failed");
// 	}

// 	return instance;
// }

// void DrewsModule::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
// {
// 	vehicle_thrust_setpoint_s v_thrust_sp = {};
// 	v_thrust_sp.timestamp = hrt_absolute_time();
// 	v_thrust_sp.timestamp_sample = timestamp_sample;
// 	/*** CUSTOM ***/
// 	// v_thrust_sp.xyz[0] = 0.0f;
// 	// v_thrust_sp.xyz[1] = 0.0f;

// 	// _thrust_setpoint.copyTo(v_thrust_sp.xyz);
// 	/*** END-CUSTOM ***/
// 	// TODO: THIS WAS THERE BEFORE v_thrust_sp.xyz[2] = PX4_ISFINITE(_thrust_sp) ? -_thrust_sp : 0.0f; // Z is Down

// 	v_thrust_sp.xyz[2] = -1;

// 	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
// }


// void DrewsModule::Run()
// {
// 	PX4_INFO("DREW WE ARE IN the RUN part of your MODULE! LETS FRICKEN GOOOOOOOOO\n");
// }

// void DrewsModule::parameters_update(bool force)
// {
// 	// check for parameter updates
// 	if (_parameter_update_sub.updated() || force) {
// 		// clear update
// 		parameter_update_s update;
// 		_parameter_update_sub.copy(&update);

// 		// update parameters from storage
// 		updateParams();
// 	}
// }

// int DrewsModule::print_usage(const char *reason)
// {
// 	if (reason) {
// 		PX4_WARN("%s\n", reason);
// 	}

// 	PRINT_MODULE_DESCRIPTION(
// 		R"DESCR_STR(
// ### Description
// Section that describes the provided module functionality.

// This is a template for a module running as a task in the background with start/stop/status functionality.

// ### Implementation
// Section describing the high-level implementation of this module.

// ### Examples
// CLI usage example:
// $ module start -f -p 42

// )DESCR_STR");

// 	PRINT_MODULE_USAGE_NAME("module", "drews_module");
// 	PRINT_MODULE_USAGE_COMMAND("start");
// 	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
// 	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
// 	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

// 	return 0;
// }

// int drews_module_main(int argc, char *argv[])
// {
// 	PX4_INFO("DREW WE ARE IN YOUR MODULE! LETS FRICKEN GOOOOOOOOO\n\n");
// 	return DrewsModule::main(argc, argv);
// }

// New implimentation vvvvv
#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/events.h>

using namespace matrix;
using namespace time_literals;
using math::radians;

DrewsModule::DrewsModule(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_mc) : ORB_ID(actuator_controls_0)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_vehicle_status.vehicle_type = vehicle_status_s::VEHICLE_TYPE_ROTARY_WING;

	parameters_updated();
	_controller_status_pub.advertise();
}

DrewsModule::~DrewsModule()
{
	perf_free(_loop_perf);
}

bool
DrewsModule::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// Set the desired trajectory...
	double yaw_des = 3.14159/2;
	DrewTrajPoint traj_point = {{0, 0, -1}, {0, 0, 0}, {0, 0, 0}, yaw_des, 0}; // NED FRAME!
	_geometric_control.SetTrajectoryPoint(traj_point);

	return true;
}

void
DrewsModule::parameters_updated()
{
	// rate control parameters
	// The controller gain K is used to convert the parallel (P + I/s + sD) form
	// to the ideal (K * [1 + 1/sTi + sTd]) form
	const Vector3f rate_k = Vector3f(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());

	_rate_control.setGains(
		rate_k.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get())),
		rate_k.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get())));

	_rate_control.setIntegratorLimit(
		Vector3f(_param_mc_rr_int_lim.get(), _param_mc_pr_int_lim.get(), _param_mc_yr_int_lim.get()));

	_rate_control.setFeedForwardGain(
		Vector3f(_param_mc_rollrate_ff.get(), _param_mc_pitchrate_ff.get(), _param_mc_yawrate_ff.get()));


	// manual rate control acro mode rate limits
	_acro_rate_max = Vector3f(radians(_param_mc_acro_r_max.get()), radians(_param_mc_acro_p_max.get()),
				  radians(_param_mc_acro_y_max.get()));

	_actuators_0_circuit_breaker_enabled = circuit_breaker_enabled_by_val(_param_cbrk_rate_ctrl.get(), CBRK_RATE_CTRL_KEY);
}

void
DrewsModule::Run()
{
	bool geometric = true;

	// PX4_INFO("DREWS MODULE IS Run()-ing!!");
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Vector3f torque_sp{0, 0, 0};
	// publishTorqueSetpoint(torque_sp, hrt_absolute_time());
	// publishThrustSetpoint(hrt_absolute_time());
	// return;
	// if (_local_pos_setpoint_sub.update(&_local_position_setpoint)) {
	// 		std::cout << "NEW TRAJECTORY SETPOINT! --> " << _local_position_setpoint.x << _local_position_setpoint.y << _local_position_setpoint.z << std::endl;
	// 		int r = 0;
	// 		int x = 5/r;
	// 		(void)x;
	// 		assert(false);
	// }


	// Test to see if we can get new local_position...
	// vehicle_local_position_s local_pos;
	// if (_vehicle_local_pos_sub.update(&local_pos)) {
	// 	PX4_INFO("RECIEVED UPDATE DREW: The new position is %f, %f, %f\n", (double)local_pos.x, (double)local_pos.y, (double)local_pos.z);
	// 	(void)local_pos;
	// }

	vehicle_odometry_s curr_odom;
	if (_vehicle_odometry_sub.update(&curr_odom)) {
		//NOTE: The position is in the earth-fixed NED frame. -- 0
		//NOTE: The velocity data is wrt the earth-fixed FRD frame (arbitrary heading reference) -- 1

		// q is the Quaternion rotation from FRD body frame to reference frame

		matrix::Vector3d position({curr_odom.x, curr_odom.y, curr_odom.z});
		matrix::Quaterniond orientation({(double)curr_odom.q[0], (double)curr_odom.q[1], (double)curr_odom.q[2], (double)curr_odom.q[3]});
		matrix::Vector3d velocity({curr_odom.vx, curr_odom.vy, curr_odom.vz});
		std::cout << "The velocity is " << velocity(0) << " " << velocity(1) << " " <<  velocity(2) << std::endl;

		matrix::Vector3d angular_velocity{curr_odom.rollspeed, curr_odom.pitchspeed, curr_odom.yawspeed}; // TODO: We are missing this right now...
		DrewOdometry odom = {position, orientation, velocity, angular_velocity};
		_geometric_control.SetOdometry(odom);
	}

	// return;


	// Perform an update for the geometric control...
	matrix::Vector3d acceleration;
	matrix::Vector3d ang_acceleration;
	_geometric_control.ComputeDesiredAcceleration(&acceleration);
	_geometric_control.ComputeDesiredAngularAcc(acceleration, &ang_acceleration);
	//std::cout << "[GEOMETRIC CONTROLLER]: des_accel: " << acceleration(0) << " " << acceleration(1) << " " <<  acceleration(2) << std::endl;
	
	DrewOdometry odom = _geometric_control.GetOdometry();
	matrix::Dcm<double> R = matrix::Dcm<double>(odom.orientation);
	double projected_acceleration = -acceleration.dot(R.col(2));

	double desired_force = (projected_acceleration*_geometric_control.vehicle_mass);
	double geometric_normalized_thrust = desired_force / 22.78; //26.7813;

	geometric_normalized_thrust = std::min(0.99, geometric_normalized_thrust);
	matrix::Vector3f geometric_normalized_ang_accel;
	float factor = 1;
	(void)(factor);
	geometric_normalized_ang_accel(0) = (float)ang_acceleration(0) / factor;
	geometric_normalized_ang_accel(1) = (float)ang_acceleration(1) / factor;
	geometric_normalized_ang_accel(2) = (float)ang_acceleration(2) / factor;
	std::cout << "[GEOMETRIC CONTROLLER]: desired accel: " << acceleration(0) << " " << acceleration(1) << " " <<  acceleration(2) << std::endl;
	std::cout << "[GEOMETRIC CONTROLLER]: desired angular accel: " << geometric_normalized_ang_accel(0) << " " << geometric_normalized_ang_accel(1) << " " <<  geometric_normalized_ang_accel(2) << std::endl;


	//std::cout << "[GEOMETRIC CONTROLLER]: des_force: " << desired_force << " normalized: " << geometric_normalized_thrust << std::endl;

	// I think we can get all of this from the vehicle_odometry topic maybe?

	



	/* run controller on gyro changes */
	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {

		// grab corresponding vehicle_angular_acceleration immediately after vehicle_angular_velocity copy
		vehicle_angular_acceleration_s v_angular_acceleration{};
		_vehicle_angular_acceleration_sub.copy(&v_angular_acceleration);

		const hrt_abstime now = angular_velocity.timestamp_sample;

		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		const Vector3f angular_accel{v_angular_acceleration.xyz};
		const Vector3f rates{angular_velocity.xyz};

		/* check for updates in other topics */
		_v_control_mode_sub.update(&_v_control_mode);

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
				_maybe_landed = vehicle_land_detected.maybe_landed;
			}
		}

		_vehicle_status_sub.update(&_vehicle_status);

		// use rates setpoint topic
		vehicle_rates_setpoint_s v_rates_sp;

		/*** CUSTOM ***/
		tilting_servo_sp_s tilting_servo_sp;
		/*** END-CUSTOM ***/

		if (_v_rates_sp_sub.update(&v_rates_sp)) {
			_rates_sp(0) = PX4_ISFINITE(v_rates_sp.roll)  ? v_rates_sp.roll  : rates(0);
			_rates_sp(1) = PX4_ISFINITE(v_rates_sp.pitch) ? v_rates_sp.pitch : rates(1);
			_rates_sp(2) = PX4_ISFINITE(v_rates_sp.yaw)   ? v_rates_sp.yaw   : rates(2);
			_thrust_sp = -v_rates_sp.thrust_body[2];
			_thrust_sp = geometric_normalized_thrust;


			/*** CUSTOM ***/
			// PX4_INFO("tilt_sp: %f \n", (double)vehicle_rates_setpoint.tilt_servo);
			_thrust_setpoint = Vector3f(v_rates_sp.thrust_body);
			/*** END-CUSTOM ***/
		}

		if(_param_tilting_type.get() == 0 && _param_mpc_pitch_on_tilt.get() && _param_airframe.get() == 11){
			if(_tilting_servo_sp_sub.update(&tilting_servo_sp))
				_tilting_angle_sp = tilting_servo_sp.angle[0];
		}
		else {
			_tilting_angle_sp = tilting_servo_sp.angle[0] = 0.0f;
		}

		// run the rate controller
		if (_v_control_mode.flag_control_rates_enabled && !_actuators_0_circuit_breaker_enabled) {

			// reset integral if disarmed
			if (!_v_control_mode.flag_armed || _vehicle_status.vehicle_type != vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				_rate_control.resetIntegral();
			}

			// update saturation status from control allocation feedback
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> saturation_positive;
				Vector<bool, 3> saturation_negative;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							saturation_positive(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							saturation_negative(i) = true;
						}
					}
				}

				// TODO: send the unallocated value directly for better anti-windup
				_rate_control.setSaturationStatus(saturation_positive, saturation_negative);
			}

			// run rate controller
			const Vector3f att_control = _rate_control.update(rates, _rates_sp, angular_accel, dt, _maybe_landed || _landed);
			(void)att_control;
			// publish rate controller status
			rate_ctrl_status_s rate_ctrl_status{};
			_rate_control.getRateControlStatus(rate_ctrl_status);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// publish actuator controls
			actuator_controls_s actuators{};

			// These are torques...
			
			actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(att_control(0)) ? att_control(0) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(att_control(1)) ? att_control(1) : 0.0f;
			actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(att_control(2)) ? att_control(2) : 0.0f;

			std::cout << "[PID CONTROLLER]: desired angular accel: " << att_control(0) << " " << att_control(1) << " " <<  att_control(2) << std::endl;

			// This is a thrust... 
			actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_thrust_sp) ? _thrust_sp : 0.0f;
			actuators.control[actuator_controls_s::INDEX_LANDING_GEAR] = _landing_gear;
			if (geometric){
				actuators.control[actuator_controls_s::INDEX_ROLL] = PX4_ISFINITE(geometric_normalized_ang_accel(0)) ? geometric_normalized_ang_accel(0) : 0.0f;
				actuators.control[actuator_controls_s::INDEX_PITCH] = PX4_ISFINITE(geometric_normalized_ang_accel(1)) ? geometric_normalized_ang_accel(1) : 0.0f;
				actuators.control[actuator_controls_s::INDEX_YAW] = PX4_ISFINITE(geometric_normalized_ang_accel(2)) ? geometric_normalized_ang_accel(2) : 0.0f;
			}

			

			/*** CUSTOM ***/
			if(_param_airframe.get() == 11 && _param_tilting_type.get() == 0 && _param_mpc_pitch_on_tilt.get()){
				actuators.control[actuator_controls_s::INDEX_FLAPS] = PX4_ISFINITE(_tilting_angle_sp) ? _tilting_angle_sp : 0.0f;
			}
			/*** END-CUSTOM ***/

			actuators.timestamp_sample = angular_velocity.timestamp_sample;

			if (!_vehicle_status.is_vtol) {
				
				if(geometric){
					publishTorqueSetpoint(geometric_normalized_ang_accel, angular_velocity.timestamp_sample);
				}else{
					publishTorqueSetpoint(att_control, angular_velocity.timestamp_sample);
				}
				
				publishThrustSetpoint(angular_velocity.timestamp_sample);
			}

			actuators.timestamp = hrt_absolute_time();
			_actuators_0_pub.publish(actuators);

			updateActuatorControlsStatus(actuators, dt);

		} else if (_v_control_mode.flag_control_termination_enabled) {
			if (!_vehicle_status.is_vtol) {
				// publish actuator controls
				actuator_controls_s actuators{};
				actuators.timestamp = hrt_absolute_time();
				_actuators_0_pub.publish(actuators);
			}
		}
	}

}

void DrewsModule::publishTorqueSetpoint(const Vector3f &torque_sp, const hrt_abstime &timestamp_sample)
{
	vehicle_torque_setpoint_s v_torque_sp = {};
	v_torque_sp.timestamp = hrt_absolute_time();
	v_torque_sp.timestamp_sample = timestamp_sample;
	v_torque_sp.xyz[0] = (PX4_ISFINITE(torque_sp(0))) ? torque_sp(0) : 0.0f;
	v_torque_sp.xyz[1] = (PX4_ISFINITE(torque_sp(1))) ? torque_sp(1) : 0.0f;
	v_torque_sp.xyz[2] = (PX4_ISFINITE(torque_sp(2))) ? torque_sp(2) : 0.0f;

	_vehicle_torque_setpoint_pub.publish(v_torque_sp);
}

void DrewsModule::publishThrustSetpoint(const hrt_abstime &timestamp_sample)
{
	vehicle_thrust_setpoint_s v_thrust_sp = {};
	v_thrust_sp.timestamp = hrt_absolute_time();
	v_thrust_sp.timestamp_sample = timestamp_sample;
	/*** CUSTOM ***/
	// v_thrust_sp.xyz[0] = 0.0f;
	// v_thrust_sp.xyz[1] = 0.0f;

	_thrust_setpoint.copyTo(v_thrust_sp.xyz);
	/*** END-CUSTOM ***/
	v_thrust_sp.xyz[2] = PX4_ISFINITE(_thrust_sp) ? -_thrust_sp : 0.0f; // Z is Down
	
	// std::cout << "[PID CONTROLLER]: thrust setpoint" << v_thrust_sp.xyz[2] << std::endl;

	// v_thrust_sp.xyz[2] = -0.7;
	_vehicle_thrust_setpoint_pub.publish(v_thrust_sp);
}

void DrewsModule::updateActuatorControlsStatus(const actuator_controls_s &actuators, float dt)
{
	for (int i = 0; i < 4; i++) {
		_control_energy[i] += actuators.control[i] * actuators.control[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {

		actuator_controls_status_s status;
		status.timestamp = actuators.timestamp;

		for (int i = 0; i < 4; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_0_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int DrewsModule::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	DrewsModule *instance = new DrewsModule(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int DrewsModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DrewsModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter rate controller. It takes rate setpoints (in acro mode
via `manual_control_setpoint` topic) as inputs and outputs actuator control messages.

The controller has a PID loop for angular rate error.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("drews_module", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int drews_module_main(int argc, char *argv[])
{
	return DrewsModule::main(argc, argv);
}


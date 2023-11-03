/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** 
 * To make this work we need the following: 
 * d
 * The current "odometry" --> the world position (vector)
                              orientation wrt W frame (quaternion)
                              body velocity (vector),
                              body angular_velocity (vector)
 * A trajectory setpoint (x, y, z, yaw) <-- World frame
 * An inertia matrix 
 * 
 * 
 * I think we need the output acceleration and angular acceleration to be normalized? 
 * 
 * A bunch of gains... Can we steal some of these from the PID controller? 
 * 
 * I think an output thrust of 0.5 means it's supposed to be hovering if straight up... 
*/

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H

#include <matrix/matrix/math.hpp>

#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>


// Default values for the lee position controller and the Asctec Firefly.
static const matrix::Vector3d kDefaultPositionGain = matrix::Vector3d(6, 6, 6);
static const matrix::Vector3d kDefaultVelocityGain = matrix::Vector3d(4.7, 4.7, 4.7);
static const matrix::Vector3d kDefaultAttitudeGain = matrix::Vector3d(3, 3, 0.035);
static const matrix::Vector3d kDefaultAngularRateGain = matrix::Vector3d(0.52, 0.52, 0.025);

class LeePositionControllerParameters {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LeePositionControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    // calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  // matrix::Matrix4Xd allocation_matrix_;
  matrix::Vector3d position_gain_;
  matrix::Vector3d velocity_gain_;
  matrix::Vector3d attitude_gain_;
  matrix::Vector3d angular_rate_gain_;
  // RotorConfiguration rotor_configuration_;
};

class LeePositionController {
 public:
  LeePositionController();
  ~LeePositionController();
  void InitializeParameters();
  // void CalculateRotorVelocities(matrix::VectorXd* rotor_velocities) const;

  // void SetOdometry(const EigenOdometry& odometry);
  // void SetTrajectoryPoint(
  //   const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  LeePositionControllerParameters controller_parameters_;
  // VehicleParameters vehicle_parameters_;

 private:
  bool initialized_params_;
  bool controller_active_;

  matrix::Vector3d normalized_attitude_gain_;
  matrix::Vector3d normalized_angular_rate_gain_;
  // matrix::Matrix4d angular_acc_to_rotor_velocities_;

  // mav_msgs::EigenTrajectoryPoint command_trajectory_;
  // EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(const matrix::Vector3d& acceleration,
                                matrix::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(matrix::Vector3d* acceleration) const;
};

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H

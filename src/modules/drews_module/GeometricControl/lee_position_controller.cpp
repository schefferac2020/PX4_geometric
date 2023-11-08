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

#include "lee_position_controller.hpp"
#include <iostream>
/**
 * X Set the parameters 
 * X I need a way of updating the odometry....
 * X Be able to set a goal point...
 * Calculate the desired acceleration
*/

LeePositionController::LeePositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

LeePositionController::~LeePositionController() {}

void LeePositionController::InitializeParameters() {
  double kDefaultInertiaXx = 0.0347563;
  double kDefaultInertiaYy = 0.0458929;
  double kDefaultInertiaZz = 0.0977;
  
  inertia_matrix(0, 0) = kDefaultInertiaXx;
  inertia_matrix(1, 1) = kDefaultInertiaYy;
  inertia_matrix(2, 2) = kDefaultInertiaZz;

  vehicle_mass = 1.56779;


  // (1x3) = (1 x 3) (3 * 3)
  normalized_attitude_gain_ = (controller_parameters_.attitude_gain_.transpose() * inertia_matrix.I()).T(); // TODO: Maybe these aren't supposed to be transpose? 
  normalized_angular_rate_gain_ = (controller_parameters_.angular_rate_gain_.transpose() * inertia_matrix.I()).T();
  // matrix::SquareMatrix<float, 4> I;  
  // I.setZero();
  // I.slice<3, 3>(0, 0) = // Some inertia matrix here...
  // I(3, 3) = 1;
  // angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // // A^{ \dagger} = A^T*(A*A^T)^{-1}
  // angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
  //     * (controller_parameters_.allocation_matrix_
  //     * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}

// void LeePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
  // assert(rotor_velocities);
  // assert(initialized_params_);

  // rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // // Return 0 velocities on all rotors, until the first command is received.
  // if (!controller_active_) {
  //   *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
  //   return;
  // }

  // Eigen::Vector3d acceleration;
  // ComputeDesiredAcceleration(&acceleration);

  // Eigen::Vector3d angular_acceleration;
  // ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // // Project thrust onto body z axis.
  // double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  // Eigen::Vector4d angular_acceleration_thrust;
  // angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  // angular_acceleration_thrust(3) = thrust;

  // *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  // *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  // *rotor_velocities = rotor_velocities->cwiseSqrt();
// }

void LeePositionController::SetOdometry(const DrewOdometry& odometry) {
  std::cout << "Just recieved a new odometry measurement!" << std::endl;
  odometry_ = odometry;
}

void LeePositionController::SetTrajectoryPoint(
    const DrewTrajPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

matrix::Vector3d cwiseProduct(const matrix::Vector3d& a, const matrix::Vector3d& b){
  return matrix::Vector3d({a(0)*b(0), a(1)*b(1), a(2)*b(2)});
}

void LeePositionController::ComputeDesiredAcceleration(matrix::Vector3d* acceleration) const {
  assert(acceleration);

  matrix::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;
  std::cout << "The normed position error is " << position_error.norm() << std::endl;
  
  // Transform velocity to world frame.
  const matrix::Quaterniond q_W_I = odometry_.orientation;
  matrix::Quaterniond test;
  matrix::Vector3d velocity_W = q_W_I.rotateVector(odometry_.velocity);

  matrix::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;
  std::cout << "The normed velocity error is " << velocity_error.norm() << std::endl;

  
  matrix::Vector3d e3({0, 0, 1});
  
  *acceleration = (cwiseProduct(position_error, controller_parameters_.position_gain_)
      + cwiseProduct(velocity_error, controller_parameters_.velocity_gain_)) / vehicle_mass
      - 9.81 * e3 - command_trajectory_.acceleration_W;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const matrix::Vector3d& acceleration,
                                                     matrix::Vector3d* angular_acceleration) const {
  // assert(angular_acceleration);

  // Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  // // Get the desired rotation matrix.
  // Eigen::Vector3d b1_des;
  // double yaw = command_trajectory_.getYaw();
  // b1_des << cos(yaw), sin(yaw), 0;

  // Eigen::Vector3d b3_des;
  // b3_des = -acceleration / acceleration.norm();

  // Eigen::Vector3d b2_des;
  // b2_des = b3_des.cross(b1_des);
  // b2_des.normalize();

  // Eigen::Matrix3d R_des;
  // R_des.col(0) = b2_des.cross(b3_des);
  // R_des.col(1) = b2_des;
  // R_des.col(2) = b3_des;

  // // Angle error according to lee et al.
  // Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  // Eigen::Vector3d angle_error;
  // vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // // TODO(burrimi) include angular rate references at some point.
  // Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  // angular_rate_des[2] = command_trajectory_.getYawRate();

  // Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  // *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
  //                          - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
  //                          + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
}

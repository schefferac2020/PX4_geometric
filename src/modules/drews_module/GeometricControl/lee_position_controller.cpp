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
  double kDefaultInertiaXx = 0.029125;
  double kDefaultInertiaYy = 0.029125;
  double kDefaultInertiaZz = 0.055225;
  
  inertia_matrix(0, 0) = kDefaultInertiaXx;
  inertia_matrix(1, 1) = kDefaultInertiaYy;
  inertia_matrix(2, 2) = kDefaultInertiaZz;

  vehicle_mass = 1.5;

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
  odometry_ = odometry;
}

DrewOdometry LeePositionController::GetOdometry() {
  return odometry_;
}

void LeePositionController::SetTrajectoryPoint(
    const DrewTrajPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

matrix::Vector3d cwiseProduct(const matrix::Vector3d& a, const matrix::Vector3d& b){
  return matrix::Vector3d({a(0)*b(0), a(1)*b(1), a(2)*b(2)});
}

void vectorFromSkewMatrix(matrix::Matrix3d& skew_matrix, matrix::Vector3d* vector) {
  // *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);

  (*vector)(0) = skew_matrix(2, 1);
  (*vector)(1) = skew_matrix(0, 2);
  (*vector)(2) = skew_matrix(1, 0);
}

void LeePositionController::ComputeDesiredAcceleration(matrix::Vector3d* acceleration) const {
  assert(acceleration);

  // Position is already in the NED frame
  matrix::Vector3d position_error;
  position_error = command_trajectory_.position_W - odometry_.position;
  std::cout << "The current position is " << odometry_.position(0) << odometry_.position(1) << odometry_.position(2) << std::endl;
  std::cout << "The desired position is " << command_trajectory_.position_W(0) << command_trajectory_.position_W(1) << command_trajectory_.position_W(2) << std::endl;
  std::cout << "position_err: " << position_error(0) << " " << position_error(1) << " " << position_error(2) << " " << std::endl;
  
  // I think velocity is already in the NED frame
  // const matrix::Quaterniond q_W_I = odometry_.orientation;
  // matrix::Vector3d velocity_W = q_W_I.rotateVector(odometry_.velocity);

  // matrix::Vector3d velocity_error;
  // velocity_error = velocity_W - command_trajectory_.velocity_W;
  matrix::Vector3d velocity_W = odometry_.velocity;
  matrix::Vector3d velocity_error = command_trajectory_.velocity_W - velocity_W;
  // std::cout << "The normed velocity error is " << velocity_error.norm() << std::endl;
  // std::cout << "Odometry position: " << odometry_.position(0) << " " << odometry_.position(1) << " " << odometry_.position(2) << std::endl;
  matrix::Vector3d e3({0, 0, 1});
  
  *acceleration = (cwiseProduct(position_error, controller_parameters_.position_gain_)  // I thinks this acceleration would be in the body frame? 
      + cwiseProduct(velocity_error, controller_parameters_.velocity_gain_)) / vehicle_mass
      - 9.81 * e3 - command_trajectory_.acceleration_W;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const matrix::Vector3d& acceleration,
                                                     matrix::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  matrix::Dcm<double> R = matrix::Dcm<double>(odometry_.orientation);
  matrix::Euler<double> euler(R);
  std::cout << "curr_angle: " << euler.phi() << " " << euler.theta() << " " << euler.psi() << std::endl;
  // std::cout << "THE VALUE OF R IS vvv\n";
  // std::cout << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
  //            << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
  //            << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n";

  
  double yaw = command_trajectory_.yaw;
  matrix::Vector3d b1_des(cos(yaw), sin(yaw), 0);
  std::cout << "1. b1_des: " << b1_des(0) << " " << b1_des(1) << " " << b1_des(2) << "\n"; 

  matrix::Vector3d b3_des;

  b3_des = -acceleration / acceleration.norm();
  std::cout << "2. b3_des: " << b3_des(0) << " " << b3_des(1) << " " << b3_des(2) << "\n"; 



  matrix::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();
  std::cout << "3. b2_des: " << b2_des(0) << " " << b2_des(1) << " " << b2_des(2) << "\n"; 

  matrix::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  std::cout << "4. b2_des: " << b2_des.cross(b3_des)(0) << " " << b2_des.cross(b3_des)(1) << " " << b2_des.cross(b3_des)(2) << "\n"; 


  std::cout << "R_des: " << std::endl;
  std::cout << R_des(0, 0) << " " << R_des(0, 1) << " " << R_des(0, 2) << "\n"
            << R_des(1, 0) << " " << R_des(1, 1) << " " << R_des(1, 2) << "\n"
            << R_des(2, 0) << " " << R_des(2, 1) << " " << R_des(2, 2) << "\n";
  
  std::cout << "R: " << std::endl;
  std::cout << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << "\n"
            << R(1, 0) << " " << R(1, 1) << " " << R(1, 2) << "\n"
            << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n";

  matrix::Euler<double> euler_des(R_des);

  std::cout << "desired_angle: " << euler_des.phi() << " " << euler_des.theta() << " " << euler_des.psi() << std::endl;
  

  matrix::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  matrix::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);
  angle_error *= -1;
  std::cout << "angle_error: " << angle_error(0) << " " << angle_error(1) << " " << angle_error(2) << " " << std::endl;

  matrix::Vector3d angular_rate_des(0, 0, 0);
  angular_rate_des(2) = command_trajectory_.yaw_rate;
  matrix::Vector3d angular_rate_error = (odometry_.angular_velocity) - R.transpose() * R_des * angular_rate_des; // TODO changed this from --=> dometry_.angular_velocity - R_des.transpose() * R * angular_rate_des; Because we thing the angular rates are in 
  (void)angular_rate_error;
  // std::cout << "1. angle_error: " << angle_error(0) << " " << angle_error(1) << " " << angle_error(2) << "\n"; 
  std::cout << "angular_rate_error: " << angular_rate_error(0) << " " << angular_rate_error(1) << " " << angular_rate_error(2) << "\n"; 
  std::cout << "angular_velocy: " << odometry_.angular_velocity(0) << " " << odometry_.angular_velocity(1) << " " << odometry_.angular_velocity(2) << "\n"; 

  // auto val = odometry_.angular_velocity.cross(inertia_matrix * odometry_.angular_velocity);

  *angular_acceleration = cwiseProduct(angle_error, controller_parameters_.attitude_gain_)
                          -cwiseProduct(angular_rate_error, controller_parameters_.angular_rate_gain_)
                           + odometry_.angular_velocity.cross(inertia_matrix * odometry_.angular_velocity); // we don't need the inertia matrix here
}

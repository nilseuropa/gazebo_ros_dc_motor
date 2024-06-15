//
// Created by david on 2023.02.02..
//

#ifndef gazebo_ros_motors_backlash_h
#define gazebo_ros_motors_backlash_h


#include <rclcpp/logger.hpp>
class Backlash {
public:
  double calcTorque(double wheel_omega, double motor_torque, double dt, rclcpp::Logger logger);
  bool isInDeadZone();

  double alfa_{0.0};
  double alfa_2{0.0};
  double motor_inertal_{0.0};
  double filter{0.0};
  double filter_value{0.0};
  double omega_motor_{0.0};
private:
  double fi_{0.0};
};


#endif // gazebo_ros_motors_backlash_h

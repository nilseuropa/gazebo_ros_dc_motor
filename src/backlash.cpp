//
// Created by david on 2023.02.02..
//

#include "gazebo_ros_motors/backlash.h"

#include <iostream>
#include <rclcpp/logging.hpp>


double Backlash::calcTorque(double wheel_omega, double motor_torque, double dt, rclcpp::Logger logger) {


  double torque_wheel{0.0};

  if(!isInDeadZone()) {
    omega_motor_ = wheel_omega;
  }

  omega_motor_ += motor_torque * dt / motor_inertal_;
  fi_ += (wheel_omega - omega_motor_) * dt;

  if(fi_ < -alfa_2) {
    fi_ = -alfa_2;
  }

  if(fi_ > alfa_2){
    fi_ = alfa_2;
  }

  if(isInDeadZone()) {
    torque_wheel = 0.0;
  } else {
    torque_wheel = motor_torque;
  }
  RCLCPP_INFO(logger, "wheel_o: %f, torque_wheel: %f, fi: %f, omega_motor: %f", wheel_omega, torque_wheel, fi_, omega_motor_);
  double filtered_torque_wheel{0.0};
  filtered_torque_wheel = filter * filter_value + torque_wheel * (1 - filter);
  filter_value = filtered_torque_wheel;
  return filtered_torque_wheel;
}
bool Backlash::isInDeadZone() {
  return (-alfa_2 < fi_) && (fi_ < alfa_2);
}

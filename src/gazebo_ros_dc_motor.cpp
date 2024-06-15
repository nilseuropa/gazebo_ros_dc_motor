#include "gazebo_ros_motors/gazebo_ros_dc_motor.h"

#include <assert.h>

#include <algorithm>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>
#include <iostream>
namespace gazebo {

// Constructor
GazeboRosMotor::GazeboRosMotor() {}

// Destructor
GazeboRosMotor::~GazeboRosMotor() {
  FiniChild();
}

void GazeboRosMotor::set_up_ros() {
  mParamSubscirber = std::make_shared<rclcpp::ParameterEventHandler>(gazebo_ros_);

  this->cmd_vel_subscriber_ = this->gazebo_ros_->create_subscription<std_msgs::msg::Float32>(
      plugin_name_ + "/" + command_topic_name, 10, [this](std_msgs::msg::Float32 cmd_msg) {
        this->cmdVelCallback(cmd_msg);
      });
  this->velocity_publisher_ = this->gazebo_ros_->create_publisher<std_msgs::msg::Float32>(plugin_name_ + "/" + velocity_topic_name, 10);
  this->effor_publisher_ = this->gazebo_ros_->create_publisher<std_msgs::msg::Float32>(plugin_name_ + "/" + "effort", 10);
  this->current_publisher_ = this->gazebo_ros_->create_publisher<std_msgs::msg::Float32>(plugin_name_ + "/" + current_topic_name, 10);

  for(auto& [name, value] : mPublishParam) {
    gazebo_ros_->declare_parameter(name, *value);
    //captured structured bindings are a C++20 extension and case warning
    std::string name_by_value{name};
    bool* value_ptr_by_value{value};
    mParamCallbacksHandlers.push_back(mParamSubscirber->add_parameter_callback(name,[value_ptr_by_value, this, name_by_value](const rclcpp::Parameter& newParam) {
      *value_ptr_by_value = newParam.as_bool();
      RCLCPP_WARN(
          this->gazebo_ros_->get_logger(),
          "%s is changed to %s",
          newParam.get_name().c_str(),
          newParam.value_to_string().c_str()
          );
    }));
  }

  gazebo_ros_->declare_parameter("update_rate", 100.0);
  mParamCallbacksHandlers.push_back(mParamSubscirber->add_parameter_callback("update_rate",[this](const rclcpp::Parameter& newParam) {
    double new_update_rate = newParam.as_double();
    if (new_update_rate > 0.0) {
      this->update_rate_ = new_update_rate;
      this->update_period_ = 1.0 / this->update_rate_;
    }
      }));

  gazebo_ros_->declare_parameter("backlash_angle", 0.15);
  mParamCallbacksHandlers.push_back(mParamSubscirber->add_parameter_callback("backlash_angle", [this](const rclcpp::Parameter& newParam) {
    this->backlash_angle = newParam.as_double();
    this->motor_backlash.alfa_2 = this->backlash_angle * this->gear_ratio_ / 2;
  }));

  gazebo_ros_->declare_parameter("gear_ratio", 62.0);
  mParamCallbacksHandlers.push_back(mParamSubscirber->add_parameter_callback("gear_ratio", [this](const rclcpp::Parameter& newParam) {
    this->gear_ratio_ = newParam.as_double();
    this->motor_backlash.alfa_2 = this->backlash_angle * this->gear_ratio_ / 2;
  }));

  for(auto& [name, value] : mMotorParams) {
    gazebo_ros_->declare_parameter(name, *value);
    //captured structured bindings are a C++20 extension and case warning
    std::string name_by_value{name};
    double* value_ptr_by_value{value};
    mParamCallbacksHandlers.push_back(mParamSubscirber->add_parameter_callback(name,[value_ptr_by_value, this, name_by_value](const rclcpp::Parameter& newParam) {
      auto temp = *value_ptr_by_value;
      *value_ptr_by_value = newParam.as_double();
      if(!this->ValidateParameters()) {
        RCLCPP_WARN(
            this->gazebo_ros_->get_logger(),
            "%s %6.3f discarded, keeping previous value of %6.3f",
            newParam.get_name().c_str(),
            *value_ptr_by_value,
            temp);
        *value_ptr_by_value = temp;
      } else {
        RCLCPP_WARN(
            this->gazebo_ros_->get_logger(),
            "%s is change to %6.3f",
            newParam.get_name().c_str(),
            *value_ptr_by_value);
      }
    }
    ));
  }
  RCLCPP_INFO(gazebo_ros_->get_logger(), "declared parameters");
}

// Load the controller
void GazeboRosMotor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  this->parent = _parent;
  gazebo_ros_ = gazebo_ros::Node::Get(_sdf);

  if (auto res = _sdf->GetAttribute("name"); res) {
    this->plugin_name_ = res->GetAsString();
    RCLCPP_INFO(gazebo_ros_->get_logger(), "Plugin name %s", this->plugin_name_.c_str());
  } else {
    RCLCPP_ERROR(gazebo_ros_->get_logger(), "There is no name attribute");
    return;
  }

  GetAtributeAsString(_sdf, command_topic_param, command_topic_name);
  GetAtributeAsString(_sdf, velocity_topic_param, velocity_topic_name);
  GetAtributeAsString(_sdf, current_topic_param, current_topic_name);

  set_up_ros();

  // motor joint

  if (auto res = _sdf->GetElement("motor_shaft_joint"); res) {
    if(joint_ = _parent->GetJoint(res->GetValue()->GetAsString()), joint_) {
      RCLCPP_INFO(gazebo_ros_->get_logger(), "Get motor_shaft_joint");
    } else {
      RCLCPP_INFO(gazebo_ros_->get_logger(), "Could not get link motor_shaft_joint joint");
    }
  } else {
    RCLCPP_FATAL(gazebo_ros_->get_logger(), "There is no element with name motor_shaft_joint joint");
    return;
  }

  if (auto res = _sdf->GetElement("motor_wrench_frame"); res) {
    if(link_ = _parent->GetLink(res->GetValue()->GetAsString()), link_) {
      RCLCPP_INFO(gazebo_ros_->get_logger(), "Get motor_wrench_frame");
    } else {
      RCLCPP_INFO(gazebo_ros_->get_logger(), "Could not get link motor_wrench_frame");
    }
  } else {
    RCLCPP_FATAL(gazebo_ros_->get_logger(), "There is no element with name motor_wrench_frame");
    return;
  }

  joint_state_publisher_ = gazebo_ros_->create_publisher<sensor_msgs::msg::JointState>(joint_->GetName() + "/joint_state", 10);
  RCLCPP_INFO(gazebo_ros_->get_logger(), "%s: Advertise joint_state", joint_->GetName().c_str());


  last_update_time_ = parent->GetWorld()->SimTime();


  free_wheel_subscriber_
      = gazebo_ros_->create_subscription<std_msgs::msg::Bool>("free_wheel", 10, [this](const std_msgs::msg::Bool& msg) {
          this->free_wheel = msg.data;
  });


  // brake command subscriber
  brake_cmd_subscriber_
      = gazebo_ros_->create_subscription<std_msgs::msg::Bool>("brake", 10, [this](std_msgs::msg::Bool cmd_msg) {
          this->brakeCommandCallBack(cmd_msg);
        });
  brake_locked_ = false;
  RCLCPP_INFO(gazebo_ros_->get_logger(), "Subscribed to %s", "/brake");

  last_update_time_ = parent->GetWorld()->SimTime();

  // listen to the update event (broadcast every simulation iteration)
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin([this](auto& info){ //const common::UpdateInfo& info
    this->UpdateChild(info);
  });

  input_ = 0;
  encoder_counter_ = 0;
  internal_current_ = 0;
  internal_omega_ = 0;
  supply_voltage_ = motor_nominal_voltage_;
  RCLCPP_INFO(gazebo_ros_->get_logger(), "Finit load plugin");
}

void GazeboRosMotor::Reset() {
  last_update_time_ = parent->GetWorld()->SimTime();
  input_ = 0;
  encoder_counter_ = 0;
  internal_current_ = 0;
  internal_omega_ = 0;
  supply_voltage_ = motor_nominal_voltage_;
  brake_locked_ = false;
}

bool GazeboRosMotor::checkParameters() {
  if (this->armature_damping_ratio_ != 0.0 && !isnan(this->armature_damping_ratio_) && this->electric_inductance_ != 0.0
      && !isnan(this->electric_inductance_) && this->electric_resistance_ != 0.0 && !isnan(this->electric_resistance_)
      && this->electromotive_force_constant_ != 0.0 && !isnan(this->electromotive_force_constant_)
      && this->moment_of_inertia_ != 0.0 && !isnan(this->moment_of_inertia_))
    return true;
  else
    return false;
}

bool GazeboRosMotor::ValidateParameters() {
  const double& d = this->armature_damping_ratio_;
  const double& L = this->electric_inductance_;
  const double& R = this->electric_resistance_;
  const double& Km = this->electromotive_force_constant_;
  const double& J = this->moment_of_inertia_;

  bool ok = checkParameters();
  // Check if d^2 L^2 + J^2 R^2 - 2 J L (2 Km^2 + d R) > 0 (which appears under sqrt)
  double Om = 0;
  if (d * d * L * L + J * J * R * R > 2 * J * L * (2 * Km * Km + d * R)) {
    Om = sqrt(d * d * L * L + J * J * R * R - 2 * J * L * (2 * Km * Km + d * R));
    // OK, roots are real, not complex
  } else {
    ok = false;
    RCLCPP_WARN(gazebo_ros_->get_logger(),
                   "Incorrect DC motor parameters: d^2 L^2 + J^2 R^2 - 2 J L (2 Km^2 + d R) > 0 not satisfied!");
  }

  if (ok) {
    // Check if -dL-JR+Om < 0 (Other real root is always negative: -dL-JR-Om)
    if (Om < d * L + J * R) {
      // OK, both real roots are stable
    } else {
      ok = false;
      RCLCPP_WARN(gazebo_ros_->get_logger(),
          "Incorrect DC motor parameters: sqrt(d^2 L^2 + J^2 R^2 - 2 J L (2 Km^2 + d R)) < d*L+J*R not satisfied!");
    }
  }
  return ok;
}

void GazeboRosMotor::publishWheelJointState(double velocity, double effort, common::Time current_time) {
  if (this->publish_motor_joint_state_ && publish_motor_joint_state_) {
    joint_state_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    joint_state_.name.resize(1);
    joint_state_.position.resize(1);
    joint_state_.velocity.resize(1);
    joint_state_.effort.resize(1);
    physics::JointPtr joint = joint_;
    double position = joint->Position(0);
    joint_state_.name[0] = joint->GetName();
    joint_state_.position[0] = position;
    joint_state_.velocity[0] = velocity;
    joint_state_.effort[0] = effort;
    joint_state_publisher_->publish(joint_state_);
  }
}

// Velocity publisher
void GazeboRosMotor::publishRotorVelocity(double m_vel, double effor) {
  std_msgs::msg::Float32 vel_msg;
  std_msgs::msg::Float32 effor_msg;

  if(!free_wheel) {
    vel_msg.data = m_vel; // (rad/sec)
  } else {
    vel_msg.data = 0.0;
  }

  if (this->velocity_publisher_ && publish_velocity_)
    velocity_publisher_->publish(vel_msg);

  if(this->effor_publisher_) {
    effor_msg.data = effor;
    effor_publisher_->publish(effor_msg);
  }
}

// Simple incremental encoder emulation
void GazeboRosMotor::publishEncoderCount(double m_vel, double dT) {
  std_msgs::msg::Int32 counter_msg;
  double rev_in_rad = m_vel * dT;
  encoder_counter_ += ((rev_in_rad) / 2 * M_PI) * encoder_pulses_per_revolution_;
  counter_msg.data = encoder_counter_;
  if (this->encoder_publisher_ && publish_encoder_)
    encoder_publisher_->publish(counter_msg);
}

void GazeboRosMotor::publishMotorCurrent() {
  std_msgs::msg::Float32 c_msg;
  c_msg.data = internal_current_; // (amps)
  if (this->current_publisher_ && publish_current_)
    current_publisher_->publish(c_msg);
}

void GazeboRosMotor::publishMotorLoad(double torque) {
  std_msgs::msg::Float32 l_msg;
  l_msg.data = torque;
  if (this->load_publisher_ && publish_load)
    load_publisher_->publish(l_msg);
}

// Motor Model update function
void GazeboRosMotor::motorModelUpdate(double dt, double output_shaft_omega, double actual_load_torque) {
  internal_omega_ = output_shaft_omega * gear_ratio_; // external shaft angular veloc. converted to internal side
  double prop_err = input_ - internal_omega_;
  v_int_err += dt * prop_err;


  mInput_voltage = mKp_v * prop_err + mKi_v * v_int_err;

  if (mInput_voltage > 1.0) {
    mInput_voltage = 1.0;
  } else if (mInput_voltage < -1.0) {
    mInput_voltage = -1.0;
  }


  motor_backlash.motor_inertal_ = moment_of_inertia_;

  /*if(motor_backlash.isInDeadZone()) {
    actual_load_torque = 0.0;
  }*/

  double T = actual_load_torque / gear_ratio_;        // external loading torque converted to internal side
  double V = mInput_voltage * motor_nominal_voltage_;                // input voltage (command input for motor velocity)

  // DC motor exact solution for current and angular velocity (omega)
  const double& d = armature_damping_ratio_;
  const double& L = electric_inductance_;
  const double& R = electric_resistance_;
  const double& Km = electromotive_force_constant_;
  const double& J = moment_of_inertia_;
  double i0 = internal_current_;
  double o0 = internal_omega_;
  double d2 = pow(d, 2);
  double L2 = pow(L, 2);
  double J2 = pow(J, 2);
  double R2 = pow(R, 2);
  double Km2 = pow(Km, 2);
  double Km3 = Km2 * Km;
  double Om = sqrt(d2 * L2 + J2 * R2 - 2 * J * L * (2 * Km2 + d * R));
  double eOp1 = exp((Om * dt) / (J * L)) + 1.0;
  double eOm1 = eOp1 - 2.0; // = exp((Om*t)/(J*L)) - 1.0;
  double eA = exp(((d * L + Om + J * R) * dt) / (2.0 * J * L));
  double emA = 1.0 / eA; // = exp(-((d*L + Om + J*R)*t)/(2.0*J*L));

  double i_t = (emA
                * (i0 * (Km2 + d * R)
                       * (d * L * (d * eOp1 * L + eOm1 * Om) + eOp1 * J2 * R2
                          - J * (4 * eOp1 * Km2 * L + 2 * d * eOp1 * L * R + eOm1 * Om * R))
                   - d * L * (d * (-2 * eA + eOp1) * L + eOm1 * Om) * (Km * T + d * V)
                   - (-2 * eA + eOp1) * J2 * R2 * (Km * T + d * V)
                   + J
                         * (Km3 * (-2 * eOm1 * o0 * Om + 4 * (-2 * eA + eOp1) * L * T)
                            - Km * R * (2 * d * eOm1 * o0 * Om - 2 * d * (-2 * eA + eOp1) * L * T + eOm1 * Om * T)
                            + 2 * Km2 * (2 * d * (-2 * eA + eOp1) * L + eOm1 * Om) * V
                            + d * (2 * d * (-2 * eA + eOp1) * L + eOm1 * Om) * R * V)))
               / (2. * (Km2 + d * R) * (d2 * L2 + J2 * R2 - 2 * J * L * (2 * Km2 + d * R)));

  double o_t
      = (emA
         * (-4 * eOp1 * J * pow(Km, 4) * L * o0
            + J * Km2 * R * (-6 * d * eOp1 * L * o0 + eOm1 * o0 * Om - 4 * (-2 * eA + eOp1) * L * T)
            + J * R2 * (-2 * d2 * eOp1 * L * o0 + d * eOm1 * o0 * Om - 2 * d * (-2 * eA + eOp1) * L * T + eOm1 * Om * T)
            + 4 * (-2 * eA + eOp1) * J * Km3 * L * V - J * Km * (-2 * d * (-2 * eA + eOp1) * L + eOm1 * Om) * R * V
            + J2 * R2 * (eOp1 * Km2 * o0 + d * eOp1 * o0 * R + (-2 * eA + eOp1) * R * T - (-2 * eA + eOp1) * Km * V)
            + L
                  * (pow(d, 3) * eOp1 * L * o0 * R + 2 * eOm1 * Km2 * Om * (i0 * Km - T)
                     + d2
                           * (eOp1 * Km2 * L * o0 - eOm1 * o0 * Om * R + (-2 * eA + eOp1) * L * R * T
                              - (-2 * eA + eOp1) * Km * L * V)
                     - d * eOm1 * Om * (Km2 * o0 + R * T + Km * (-2 * i0 * R + V)))))
        / (2. * (Km2 + d * R) * (d2 * L2 + J2 * R2 - 2 * J * L * (2 * Km2 + d * R)));

  // Update internal variables
  internal_current_ = i_t;
  internal_omega_ = o_t;
  ignition::math::Vector3d applied_torque;
  // TODO: axis as param
  applied_torque.Z() = Km * i_t * gear_ratio_;


  //applied_torque.Z() = motor_backlash.calcTorque(output_shaft_omega * gear_ratio_, applied_torque_z/gear_ratio_, dt, gazebo_ros_->get_logger()) * gear_ratio_;

  if (brake_locked_) {
    joint_->SetParam("fmax", 0, 50.0); // TODO: brake force param
    joint_->SetParam("vel", 0, 0.0);
  } else {
    joint_->SetParam("fmax", 0, 0.0);
    if(!free_wheel) {
      this->link_->AddRelativeTorque(applied_torque);
    }
  }
}

// Plugin update function
void GazeboRosMotor::UpdateChild(const gazebo::common::UpdateInfo & _info) {

  common::Time current_time = _info.simTime;
  double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
  double current_output_speed = link_->RelativeAngularVel().Z();

  ignition::math::Vector3d current_torque = this->link_->RelativeTorque();
  double actual_load = current_torque.Z();
  motorModelUpdate(seconds_since_last_update, current_output_speed, actual_load);

  std::random_device rd{};
  std::mt19937 gen{rd()};

  if ( seconds_since_last_update >= update_period_ ) {
    publishWheelJointState( current_output_speed, current_torque.Z(), current_time);
    publishMotorCurrent();

    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<double> noise{current_output_speed,velocity_noise_};

    double  current_noisy_output_speed = noise(gen);
    //publishRotorVelocity( this->motor_backlash.omega_motor_ / gear_ratio_);
    publishRotorVelocity(current_output_speed, mInput_voltage);
    publishEncoderCount( current_noisy_output_speed , seconds_since_last_update );
    publishMotorLoad(actual_load);
    last_update_time_ = current_time;
  }
}


// Finalize the controller
void GazeboRosMotor::FiniChild() {

}

// Callbacks from custom que

void GazeboRosMotor::brakeCommandCallBack(std_msgs::msg::Bool cmd_msg) {
  brake_locked_ = cmd_msg.data;
}

void GazeboRosMotor::cmdVelCallback(std_msgs::msg::Float32 cmd_msg) {
  input_ = cmd_msg.data * gear_ratio_;
}

void GazeboRosMotor::supplyVoltageCallBack(std_msgs::msg::Float32 voltage) {
  supply_voltage_ = voltage.data;
}
void GazeboRosMotor::GetAtributeAsString(sdf::ElementPtr _sdf, const std::string& atr_name, std::string& attribute) {
  if (auto res = _sdf->GetElement(atr_name); res) {
    RCLCPP_INFO(gazebo_ros_->get_logger(), "Get %s", atr_name.c_str());
    attribute = res->GetValue()->GetAsString();
  } else {
    RCLCPP_FATAL(gazebo_ros_->get_logger(), "There is no attribute with name %s", atr_name.c_str());
    return;
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosMotor)


// eof_ns
} // namespace gazebo

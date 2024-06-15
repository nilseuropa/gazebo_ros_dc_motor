#ifndef _DC_MOTOR_PLUGIN_H_
#define _DC_MOTOR_PLUGIN_H_

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
// ROS

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "gazebo_ros_motors/backlash.h"

namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosMotor : public ModelPlugin {

    public:

      GazeboRosMotor();
      ~GazeboRosMotor();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;
      void Reset() override;
      bool ValidateParameters();
      bool checkParameters();

    protected:

      virtual void UpdateChild(const gazebo::common::UpdateInfo & _info);
      virtual void FiniChild();

    private:

      void GetAtributeAsString(sdf::ElementPtr _sdf, const std::string& atr_name, std::string& attribute);

      gazebo_ros::Node::SharedPtr gazebo_ros_;
      std::string plugin_name_;
      event::ConnectionPtr update_connection_;
      physics::ModelPtr parent;
      physics::JointPtr joint_;
      physics::LinkPtr link_;

      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoder_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr effor_publisher_;


      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr current_publisher_;
      rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr load_publisher_;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
      rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr debug_publisher_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_vel_subscriber_;
      rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr supply_voltage_subscriber_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_cmd_subscriber_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr free_wheel_subscriber_;
      sensor_msgs::msg::JointState joint_state_;
      geometry_msgs::msg::WrenchStamped wrench_msg_;
      std::shared_ptr<rclcpp::ParameterEventHandler> mParamSubscirber;
      std::map<std::string, std::string> mTopicNames;
      std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> mParamCallbacksHandlers;
      std::shared_ptr<rclcpp::ParameterEventCallbackHandle> mParamCallbackHandeler;
      std::map<std::string, double*> mMotorParams{
          {"motor_nominal_voltage", &motor_nominal_voltage_},
          {"moment_of_inertia", &moment_of_inertia_},
          {"armature_damping_ratio", &armature_damping_ratio_},
          {"electromotive_force_constant", &electromotive_force_constant_},
          {"electric_resistance", &electric_resistance_},
          {"electric_inductance", &electric_inductance_},
          //{"gear_ratio", &gear_ratio_},
          {"encoder_ppr", &encoder_pulses_per_revolution_},
          {"velocity_noise", &velocity_noise_},
          {"kp_v", &mKp_v},
          {"ki_v", &mKi_v},
      };

      // Measurement noise
      double velocity_noise_{0.0};
      double supply_voltage_{0.0};
      // Topic params
      std::string command_topic_name{"command/velocity"};
      std::string velocity_topic_name{"velocity"}; /// topic for the motor shaft velocity (encoder side, before gearbox)
      std::string current_topic_name{"current"};
      std::string load_topic_{"load"};
      std::string encoder_topic_{"encoder"};
      std::string supply_topic_{"supply_voltage"};
      std::string wrench_frame_;

      std::string command_topic_param{"command_topic"};
      std::string velocity_topic_param{"velocity_topic"};
      std::string encoder_topic_param{"encoder_topic"};
      std::string current_topic_param{"current_topic"};
      std::string load_topic_param{"load_topic"};
      std::string supply_topic_param{"supply_topic"};



      bool publish_velocity_{true};
      bool publish_current_{false};
      bool publish_encoder_{true};
      bool publish_motor_joint_state_{true};
      bool publish_load{false};
      bool brake_locked_{false};
      bool free_wheel{false};
      bool publish_debug_{false};

      std::map<std::string, bool*> mPublishParam{
          {"publish_velocity",&publish_velocity_},
          {"publish_encoder",&publish_encoder_},
          {"publish_current",&publish_current_},
          {"publish_motor_joint_state", &publish_motor_joint_state_},
          {"publish_load", &publish_load},
      };

      double input_{0.0};
      double update_rate_{1000.0};

      void set_up_ros();

      // Gearbox
      double gear_ratio_{72.0}; // reduction ratio, eg 10.0 means 1/10-th output angular velocity compared to motor inner vel.
      double backlash_angle{0.05}; // backlash of the wheel in radian

      // Motor model
      double motor_nominal_voltage_{24.0}; /// the nominal voltage of the motor which corresponds to max angular velocity
      double moment_of_inertia_{0.001};
      double armature_damping_ratio_{0.0001};
      double electromotive_force_constant_{0.03}; // Nm/A = V/(rad/s)
      double electric_resistance_{13.8};
      double electric_inductance_{0.001};
      //gazebo_ros_motors::motorModelConfig current_config_;
      // Internal state variables
      double internal_current_{0.0};
      double internal_omega_{0.0};

      // Encoder model
      double encoder_counter_{0.0};
      double encoder_pulses_per_revolution_{111};

      // Speed PI controller
      double mKp_v{0.01};
      double mKi_v{0.01};
      double v_int_err{0.0};
      double mInput_voltage{0.0};
      // Helper variables
      double update_period_;
      common::Time last_update_time_;

      Backlash motor_backlash{};
      void publishRotorVelocity(double m_vel, double effor);
      void publishEncoderCount(long ctr);
      void cmdVelCallback(std_msgs::msg::Float32 cmd_msg);
      void supplyVoltageCallBack(std_msgs::msg::Float32 voltage);
      void brakeCommandCallBack(std_msgs::msg::Bool cmd_msg );
      void publishWheelJointState( double m_vel, double m_effort , common::Time current_time);
      void motorModelUpdate(double dt, double actual_omega, double current_torque);
      void publishEncoderCount(double m_vel, double dT);
      void publishMotorCurrent();
      void publishMotorLoad(double torque);
  };

}

#endif

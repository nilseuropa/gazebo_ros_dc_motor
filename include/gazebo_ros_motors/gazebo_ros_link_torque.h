#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_LINK_TORQUE_H_
#define GAZEBO_PLUGINS__GAZEBO_ROS_LINK_TORQUE_H_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>

#include "std_msgs/msg/string.hpp"

namespace gazebo {

class LinkTorquePlugin : public gazebo::ModelPlugin {

public:
  LinkTorquePlugin() {};
  virtual ~LinkTorquePlugin();
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

protected:
  virtual void OnUpdate();

private:
  gazebo_ros::Node::SharedPtr mRosNode;
  gazebo::event::ConnectionPtr mWorldUpdateCallback;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mCommandSubscriber;

  physics::ModelPtr mGazeboModel;
  physics::LinkPtr mTargetLink;

  std::string mPluginName;
  std::string mCommandTopic;
  enum {X, Y, Z} mAxis;
  const char mAxisNames[3] = {'X', 'Y', 'Z'};

  ignition::math::Vector3d mAccumulatedTorque;

  void commandCallback(const std_msgs::msg::Float32& cmd_msg);
};
} // namespace gazebo

#endif // GAZEBO_PLUGINS_LINK_TORQUE_H_

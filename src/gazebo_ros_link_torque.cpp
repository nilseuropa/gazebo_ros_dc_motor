
#include "gazebo_ros_motors/gazebo_ros_link_torque.h"

namespace gazebo {

LinkTorquePlugin::~LinkTorquePlugin() {
  mRosNode.reset();
}

void LinkTorquePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr _sdf) {

  this->mGazeboModel = model;
  mRosNode = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(mRosNode->get_logger(), "Loading Torque Gazebo Plugin in model %s", model->GetName().c_str());

  if (!rclcpp::ok()) {
    RCLCPP_FATAL_STREAM(
        mRosNode->get_logger(),
        "A ROS node for Gazebo has not been initialized, unable to load plugin. "
            << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  if (auto res = _sdf->GetAttribute("name"); res) {
    mPluginName = res->GetAsString();
    RCLCPP_INFO(mRosNode->get_logger(), "plugin name: %s", this->mPluginName.c_str());
  } else {
    mPluginName = "torqueplugin";
    RCLCPP_WARN(
        mRosNode->get_logger(), "There is no name parameter of this plugin, using name %s!", mPluginName.c_str());
  }

  if (auto res = _sdf->GetElement("command_topic"); res) {
    auto value = res->GetValue()->GetAsString();
    mCommandTopic = mPluginName + "/" + value;
  } else {
    mCommandTopic = mPluginName + "/torque";
    RCLCPP_WARN(mRosNode->get_logger(), "There is no mCommandTopic parameter, using %s!", mCommandTopic.c_str());
  }

  if (auto res = _sdf->GetElement("torque_link"); res) {
    auto targetLinkName = res->GetValue()->GetAsString();
    mTargetLink = mGazeboModel->GetLink(targetLinkName);

    if (mTargetLink == nullptr) {
      RCLCPP_FATAL(mRosNode->get_logger(), "Target link %s is not found!", targetLinkName.c_str());
      return;
    }
  }

  if (auto res = _sdf->GetElement("axis"); res) {
    auto value = res->GetValue()->GetAsString();
    switch (toupper(value.at(0))) {
      case 'X':
        mAxis = X;
        break;
      case 'Y':
        mAxis = Y;
        break;
      case 'Z':
        mAxis = Z;
        break;
      default:
        RCLCPP_FATAL(mRosNode->get_logger(), "Unsupported axis of rotation: %s!", value.c_str());
        return;
    }
  }

  RCLCPP_INFO(mRosNode->get_logger(), "Command topic: %s", this->mCommandTopic.c_str());
  RCLCPP_INFO(mRosNode->get_logger(), "Target link name: %s", this->mTargetLink->GetName().c_str());
  RCLCPP_INFO(mRosNode->get_logger(), "Axis of rotation:  %c", this->mAxisNames[mAxis]);

  mCommandSubscriber = mRosNode->create_subscription<std_msgs::msg::Float32>(
      mCommandTopic, 10, [this](std_msgs::msg::Float32 cmd_msg) {
        this->commandCallback(cmd_msg);
      });
  RCLCPP_INFO(mRosNode->get_logger(), "Subscribed to %s", mCommandTopic.c_str());

  mWorldUpdateCallback = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&LinkTorquePlugin::OnUpdate, this));
}

void LinkTorquePlugin::commandCallback(const std_msgs::msg::Float32& cmd_msg) {
  double input = cmd_msg.data;

  switch (mAxis) {
    case X:
      mAccumulatedTorque.X() = input;
      break;
    case Y:
      mAccumulatedTorque.Y() = input;
      break;
    case Z:
      mAccumulatedTorque.Z() = input;
      break;
  }
}

void LinkTorquePlugin::OnUpdate() {
  mTargetLink->AddRelativeTorque(mAccumulatedTorque);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LinkTorquePlugin)
} // namespace gazebo

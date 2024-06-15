#include "gazebo_ros_motors/gazebo_ros_bldc_motor.h"

#include <assert.h>

#include <algorithm>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <sdf/sdf.hh>
namespace gazebo {

GazeboBldcMotor::GazeboBldcMotor() {}

GazeboBldcMotor::~GazeboBldcMotor() {}


void GazeboBldcMotor::initMotor() {



  std::lock_guard<std::recursive_mutex> lock{mParamsMutex};
  mMotor = boost::shared_ptr<BLDCMotorFoC>(
      new BLDCMotorFoC(mMotorParameters[cRosParamElectricResistance],
                       mMotorParameters[cRosParamTorqueConstant],
                       mMotorParameters[cRosParamMaximumCurrent],
                       mMotorParameters[cRosParamSelfInductance],
                       mMotorParameters[cRosParamMutualInductance],
                       mMotorParameters[cRosParamStatorInductanceFluctuation],
                       mMotorParameters[cRosParamPolePairs],
                       mMotorParameters[cRosParamRotorInertia], // inertia doesn't metter here, it is also set to link
                                                                // _rotor, where it counts
                       mMotorParameters[cRosParamFrictionCoefficient],
                       mMotorParameters[cRosParamSupplyVoltage],
                       mParent->GetWorld()->Physics()->GetMaxStepSize()));

  // TODO 0.001, mParent->GetWorld()->Physics()->GetMaxStepSize() is the simulation resolution with ODE, that might
  // varies with other engines

  mMotor->reset_controller();
  mMotor->set_veloc_pi(mMotorParameters[cRosParamPiProportionalGainForVelocity],
                       mMotorParameters[cRosParamPiIntegralGainForVelocity]);
  mMotor->set_current_pi(mMotorParameters[cRosParamPiProportionalGainForCurrent],
                         mMotorParameters[cRosParamPiIntegralGainForCurrent]);
  mState = {0.0, 0.0, 0.0, 0.0};
  mMotorTorque = 0;
  mEncoderCounter = 0;
}

void GazeboBldcMotor::addPublishParameters() {

  mGazeboRosNode->declare_parameter(mCommandTopicParam, "cmd");
  mGazeboRosNode->declare_parameter(mVelocityTopicParam, "velocity");
  mGazeboRosNode->declare_parameter(mRotorVelocityTopicParam, "rotorvelocity");
  mGazeboRosNode->declare_parameter(mEncoderTopicParam, "encoder");
  mGazeboRosNode->declare_parameter(mCurrentTopicParam, "current");
  mGazeboRosNode->declare_parameter(mTorqueTopicParam, "torque");
  mGazeboRosNode->declare_parameter(mTorqueErrorTopicParam, "torqueerror");

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mCommandTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          mCmdVelSubscriber = this->mGazeboRosNode->create_subscription<std_msgs::msg::Float32>(
              mPluginName + "/" + newParam.as_string(), 10, [this](std_msgs::msg::Float32 cmd_msg) {
                this->cmdVelCallback(cmd_msg);
              });
          RCLCPP_INFO(
              this->mGazeboRosNode->get_logger(), "subscribed to command_topic %s", newParam.as_string().c_str());
        }
      }));

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mVelocityTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          this->mTorqueVelocityPublisher = this->mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(
              mPluginName + "/" + newParam.as_string(), 10);
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(), "created publisher to %s", newParam.as_string().c_str());
        }
      }));

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mRotorVelocityTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          this->mRotorVelocityPublisher = this->mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(
              mPluginName + "/" + newParam.as_string(), 10);
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(), "created publisher to %s", newParam.as_string().c_str());
        }
      }));

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mCurrentTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          this->mCurrentPublisher = this->mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(
              mPluginName + "/" + newParam.as_string(), 10);
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(), "created publisher to %s", newParam.as_string().c_str());
        }
      }));

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mTorqueTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          this->mTorquePublisher = this->mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(
              mPluginName + "/" + newParam.as_string(), 10);
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(), "created publisher to %s", newParam.as_string().c_str());
        }
      }));

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mTorqueErrorTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          this->mTorqueErrorPublisher = this->mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(
              mPluginName + "/" + newParam.as_string(), 10);
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(), "created publisher to %s", newParam.as_string().c_str());
        }
      }));

  mParamCallbacksHandlers.push_back(
      mRosParamSubscriber->add_parameter_callback(mEncoderTopicParam, [this](const rclcpp::Parameter& newParam) {
        if (newParam.as_string().compare("") != 0) {
          this->mEncoderPublisher = this->mGazeboRosNode->create_publisher<std_msgs::msg::Int32>(
              mPluginName + "/" + newParam.as_string(), 10);
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(), "created publisher to %s", newParam.as_string().c_str());
        }
      }));

  for (auto& [name, value] : mPublishesEnabled) {
    mGazeboRosNode->declare_parameter(name, *value);
    // captured structured bindings are a C++20 extension and case warning
    bool* value_ptr_by_value{value};
    mParamCallbacksHandlers.push_back(mRosParamSubscriber->add_parameter_callback(
        name, [value_ptr_by_value, this](const rclcpp::Parameter& newParam) {
          *value_ptr_by_value = newParam.as_bool();
          RCLCPP_INFO(this->mGazeboRosNode->get_logger(),
                      "%s is changed to %s",
                      newParam.get_name().c_str(),
                      newParam.value_to_string().c_str());
        }));
  }

  mJointStatePublisher
      = mGazeboRosNode->create_publisher<sensor_msgs::msg::JointState>(mJoint->GetName() + "/joint_state", 10);
  RCLCPP_INFO(mGazeboRosNode->get_logger(), "advertise joint_state: %s", mJointStatePublisher->get_topic_name());
}

void GazeboBldcMotor::commitParameterChange(std::string parameterName, double parameterValue) {
  if (cRosParamGearRatio == parameterName) {
    mGearRatio = parameterValue;
  } else if (cRosParamEncoderPulsesPerRevolution == parameterName) {
    mEncoderPulsesPerRevolution = parameterValue;
    mEncoderCounter = 0;
  } else if (cRosParamStandardDeviationOfNoise == parameterName) {
    noise = std::normal_distribution<double>(0, parameterValue);
  } else if (cRosParamRotorInertia == parameterName) {
    mpLoadLinkInertia->SetIZZ(mpLoadLinkInertia->IZZ()
                              + mMotorParameters[cRosParamGearRatio] * mMotorParameters[cRosParamRotorInertia]);
  } else if (cRosParamFrictionCoefficient == parameterName) {
    if (mJoint) {
      mJoint->SetParam(cStrFriction, 0, parameterValue);
      mJoint->SetParam(cStrFriction, 1, parameterValue);
      mJoint->SetParam(cStrFriction, 2, parameterValue);
    }
  } else if (cRosParamDampingCoefficient == parameterName) {
    if (mJoint) {
      mJoint->SetDamping(0, parameterValue);
      mJoint->SetDamping(1, parameterValue);
      mJoint->SetDamping(2, parameterValue);
    }
  } else {
    this->initMotor();
  }
}

void GazeboBldcMotor::addRosParameters() {

  for (auto& [name, value] : mMotorParameters) {
    mGazeboRosNode->declare_parameter(name, value);

    mParamCallbacksHandlers.push_back(
        mRosParamSubscriber->add_parameter_callback(name, [this](const rclcpp::Parameter& param) {

          std::lock_guard<std::recursive_mutex> lock{mParamsMutex};

          auto oldValue = this->mMotorParameters[param.get_name()];
          this->mMotorParameters[param.get_name()] = param.as_double();

          // plugin is practically executed in one thread only, so it is OK to set, validate and maybe revert the change
          if (!this->ValidateParameters()) {
            RCLCPP_WARN(this->mGazeboRosNode->get_logger(),
                        "%s %6.3f discarded, keeping previous value of %6.3f",
                        param.get_name().c_str(),
                        param.as_double(),
                        oldValue);
            this->mGazeboRosNode->set_parameter({param.get_name(), oldValue});
            return;
          }

          RCLCPP_INFO(this->mGazeboRosNode->get_logger(),
                      "%s is changed to %6.3f",
                      param.get_name().c_str(),
                      param.as_double());

          commitParameterChange(param.get_name(), param.as_double());
        }));
  }
}

void GazeboBldcMotor::Load(physics::ModelPtr aParentModel, sdf::ElementPtr aPluginSdf) {

  mParent = aParentModel;
  mGazeboRosNode = gazebo_ros::Node::Get(aPluginSdf);
  mRosParamSubscriber = std::make_shared<rclcpp::ParameterEventHandler>(mGazeboRosNode);

  if (auto res = aPluginSdf->GetAttribute("name"); res) {
    mPluginName = res->GetAsString();
    RCLCPP_INFO(mGazeboRosNode->get_logger(), "Plugin name: %s", mPluginName.c_str());
  } else {
    RCLCPP_ERROR(mGazeboRosNode->get_logger(), "There is no name attribute");
    throw new common::Exception(nullptr, 0, "GazeboBldcMotor plugin cannot load without name");
  }

  if (auto res = aPluginSdf->GetElement("motor_shaft_joint"); res) {
    mJoint = aParentModel->GetJoint(res->GetValue()->GetAsString());
  } else {
    mJoint = aParentModel->GetJoint(mPluginName);
  }

  if (!mJoint) {
    RCLCPP_FATAL(mGazeboRosNode->get_logger(), "Missing motor joint!");
    throw new common::Exception(nullptr, 0, "GazeboBldcMotor plugin cannot find joint");
  }

  if (auto res = aPluginSdf->GetElement("rotated_link"); res) {
    mLoadLink = aParentModel->GetLink(res->GetValue()->GetAsString());
  }

  if (mLoadLink == nullptr) {
    RCLCPP_INFO(mGazeboRosNode->get_logger(), "Could not find rotated link");
    throw new common::Exception(nullptr, 0, "GazeboBldcMotor plugin cannot find the rotated link");
  }

  if (mShaft = aParentModel->GetLink(mPluginName + "_rotor"), mShaft) {
    RCLCPP_INFO(mGazeboRosNode->get_logger(), "Get motor shaft link");
  } else {
    RCLCPP_INFO(
        mGazeboRosNode->get_logger(), "No motor shaft in URDF, motor drives %s directly", mLoadLink->GetName().c_str());
    mShaft = mLoadLink;
  }

  mpOriginalLoadLinkInertia = mShaft->GetInertial();

  mpLoadLinkInertia->SetInertiaMatrix(
      mpLoadLinkInertia->IXX(),
      mpLoadLinkInertia->IYY(),
      mpLoadLinkInertia->IZZ() + mMotorParameters[cRosParamGearRatio] * mMotorParameters[cRosParamRotorInertia],
      mpLoadLinkInertia->IXY(),
      mpLoadLinkInertia->IXZ(),
      mpLoadLinkInertia->IYZ());
  mShaft->SetInertial(mpLoadLinkInertia);

  initMotor();
  addPublishParameters();
  addRosParameters();
  Reset();

  mUpdateConnection = event::Events::ConnectWorldUpdateBegin([this](auto& info) { // const common::UpdateInfo& info
    this->updateChild(info);
  });

  // PI debug publishers:
  mDebugPublisherTorqRef
      = mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(mPluginName + "/debug_torque_reference", 10);
  mDebugPublisherIdIntegralErr
      = mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(mPluginName + "/debug_id_integral_error", 10);
  mDebugPublisherIqIntegralErr
      = mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(mPluginName + "/debug_iq_integral_error", 10);
  mDebugPublisherVoltageD
      = mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(mPluginName + "/debug_voltage_d", 10);
  mDebugPublisherVoltageQ
      = mGazeboRosNode->create_publisher<std_msgs::msg::Float32>(mPluginName + "/debug_voltage_q", 10);

  RCLCPP_INFO(mGazeboRosNode->get_logger(), "GazeboBldcMotor is loaded");
}

void GazeboBldcMotor::Reset() {
  mLastUpdateTime = 0;
  mLastStepTime = mParent->GetWorld()->SimTime();

  mMotor->reset_controller();
}

bool GazeboBldcMotor::ValidateParameters() {
  if (mMotorParameters[cRosParamElectricResistance] <= 0 || mMotorParameters[cRosParamTorqueConstant] <= 0
      || mMotorParameters[cRosParamSelfInductance] <= 0 || mMotorParameters[cRosParamFrictionCoefficient] <= 0
      || mMotorParameters[cRosParamPolePairs] <= 0) {
    RCLCPP_WARN(mGazeboRosNode->get_logger(),
                "Incorrect BLDC motor parameters: R, kt, Ls and beta should be greater than 0");
    return false;
  }

  if (mMotorParameters[cRosParamMutualInductance] < 0 || mMotorParameters[cRosParamStatorInductanceFluctuation] < 0
      || mMotorParameters[cRosParamRotorInertia] < 0) {
    RCLCPP_WARN(mGazeboRosNode->get_logger(),
                "Incorrect BLDC motor parameters: Ms and LM and J should not be negative");
    return false;
  }

  // no pre-defined sqrt(...) calls, that could be checked

  return true;
}


void GazeboBldcMotor::publishLoadJointState(common::Time current_time) {
  if (this->mJointStatePublisher != nullptr) {
    sensor_msgs::msg::JointState jointStateMsg;
    jointStateMsg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    jointStateMsg.name.resize(1);
    jointStateMsg.position.resize(1);
    jointStateMsg.velocity.resize(1);
    jointStateMsg.effort.resize(1);

    double position = mJoint->Position(2);
    jointStateMsg.name[0] = mJoint->GetName();
    jointStateMsg.position[0] = position;
    jointStateMsg.velocity[0] = mJoint->GetVelocity(2);
    jointStateMsg.effort[0] = mJoint->GetChild()->RelativeTorque().Z();
    mJointStatePublisher->publish(jointStateMsg);
  }
}

void GazeboBldcMotor::publish() {

  if (mPublishRotorVelocityEnabled && mRotorVelocityPublisher != nullptr) {
    std_msgs::msg::Float32 vel_msg;
    vel_msg.data = mRotorVelocity;
    mRotorVelocityPublisher->publish(vel_msg);
  }

  double noisyLoadVelocity = mLoadLink->RelativeAngularVel().Z();
  if (mPublishVelocityEnabled && mTorqueVelocityPublisher != nullptr) {
    std_msgs::msg::Float32 vel_msg;
    vel_msg.data = noisyLoadVelocity;
    mTorqueVelocityPublisher->publish(vel_msg);
  }

  if (mPublishEncoderEnabled && mEncoderPublisher != nullptr) {
    std_msgs::msg::Int32 counter_msg;
    counter_msg.data = mEncoderCounter;
    mEncoderPublisher->publish(counter_msg);
  }

  if (mPublishCurrentEnabled && mCurrentPublisher != nullptr) {
    double current = sqrt(mState[0] * mState[0] + mState[1] * mState[1]);

    std_msgs::msg::Float32 c_msg;
    c_msg.data = mMotorParameters[cRosParamCurrentOffset] + current + noise(gen);
    mCurrentPublisher->publish(c_msg);
  }

  if (mPublishTorqueEnabled && mTorquePublisher != nullptr) {
    std_msgs::msg::Float32 l_msg;
    l_msg.data = mMotorTorque;
    mTorquePublisher->publish(l_msg);
  }


}

void GazeboBldcMotor::updateChild(const gazebo::common::UpdateInfo& aUpdateInfo) {
  std::lock_guard<std::recursive_mutex> lock{mParamsMutex};
  common::Time currentTime = aUpdateInfo.simTime;
  double dt = currentTime.Double() - mLastStepTime.Double();
  mLastStepTime = currentTime;

  double rotationOfLoad = dt * mJoint->GetVelocity(2);
  mEncoderCounter += (rotationOfLoad / (2 * M_PI)) * mEncoderPulsesPerRevolution;

  // [1] correct the rotation and speed in the model according to the simulation:
  mRotorVelocity = mGearRatio * mJoint->GetVelocity(2);
  mState[3] -= dt * (mState[2] - mRotorVelocity);
  mState[2] = mRotorVelocity;
  mMotor->set_load(mShaft->RelativeTorque().Z() / mGearRatio);

  // [2] calculate motor torque and forward it to the link:
  mState = mMotor->step(mState);
  mMotorTorque
      = mGearRatio * mMotor->getShaftTorque(mState, 0); // TODO epsilon_m =0 mit tüntet el? = TE-J*epsilon_m-beta*omega;

  if (std::isnan(mMotorTorque)) {
    RCLCPP_ERROR(mGazeboRosNode->get_logger(), "calculated motor torque is NaN, resetting motor state");
    return;
  }

  ignition::math::Vector3d appliedTorque(0, 0, mMotorTorque);
  mShaft->AddRelativeTorque(appliedTorque);

  // [3] publish to ROS topics when appropriate:
  double secondsSinceLastUpdate = (currentTime - mLastUpdateTime).Double();
  if (secondsSinceLastUpdate >= mUpdatePeriod) {
    std_msgs::msg::Float32 msg;

    msg.data = mMotor->getIdIntegralError();
    mDebugPublisherIdIntegralErr->publish(msg);

    msg.data = mMotor->getIqIntegralError();
    mDebugPublisherIqIntegralErr->publish(msg);

    msg.data = mMotor->getVoltaged();
    mDebugPublisherVoltageD->publish(msg);

    msg.data = mMotor->getVoltageq();
    mDebugPublisherVoltageQ->publish(msg);

    msg.data = mMotor->getReferenceTorque();
    mDebugPublisherTorqRef->publish(msg);

    publish();
    publishLoadJointState(currentTime);
    mLastUpdateTime = currentTime;
  }
}

void GazeboBldcMotor::cmdVelCallback(std_msgs::msg::Float32 cmd_msg) {
  mMotor->set_target_ang_veloc(mGearRatio * cmd_msg.data);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBldcMotor)

} // namespace gazebo

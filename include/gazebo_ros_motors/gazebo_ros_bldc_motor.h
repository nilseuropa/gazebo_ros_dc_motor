#ifndef _DC_MOTOR_PLUGIN_H_
#define _DC_MOTOR_PLUGIN_H_

#include <map>
#include <mutex>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

// ROS
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "gazebo_ros_motors/bldcmotor_foc.h"

namespace gazebo {

static const char* const cRosParamElectricResistance = "electric_resistance";
static const char* const cRosParamSelfInductance = "self_inductance";
static const char* const cRosParamMaximumCurrent = "maximum_current";
static const char* const cRosParamMutualInductance = "mutual_inductance";
static const char* const cRosParamStatorInductanceFluctuation = "stator_inductance_fluctuation";
static const char* const cRosParamPolePairs = "pole_pairs";
static const char* const cRosParamTorqueConstant = "torque_constant";
static const char* const cRosParamRotorInertia = "rotor_inertia";
static const char* const cRosParamSupplyVoltage = "supply_voltage";
static const char* const cRosParamEncoderPulsesPerRevolution = "encoder_pulses_per_revolution";
static const char* const cRosParamStandardDeviationOfNoise = "standard_deviation_of_noise";
static const char* const cRosParamGearRatio = "gear_ratio";
static const char* const cRosParamCurrentOffset = "current_offset";
static const char* const cRosParamFrictionCoefficient = "friction_coefficient";
static const char* const cRosParamDampingCoefficient = "damping_coefficient";
static const char* const cRosParamPiProportionalGainForVelocity = "pi_proportional_gain_for_velocity";
static const char* const cRosParamPiIntegralGainForVelocity = "pi_integral_gain_for_velocity";
static const char* const cRosParamPiProportionalGainForCurrent = "pi_proportional_gain_for_current";
static const char* const cRosParamPiIntegralGainForCurrent = "pi_integral_gain_for_current";

static const char* const cRosTopicCommand = "command_topic";
static const char* const cRosTopicVelocity = "velocity_topic";
static const char* const cRosTopicRotorVelocity = "rotorvelocity_topic";
static const char* const cRosTopicEncoder = "encoder_topic";
static const char* const cRosTopicCurrent = "current_topic";
static const char* const cRosTopicTorque = "torque_topic";
static const char* const cRosTopicTorqueError = "torque_error_topic";

static const char* const cRosParamPublishVelocity = "publish_velocity";
static const char* const cRosParamPublishRotorVelocity = "publish_rotorvelocity";
static const char* const cRosParamPublishEncoder = "publish_encoder";
static const char* const cRosParamPublishCurrent = "publish_current";
static const char* const cRosParamPublishMotorJointState = "publish_motor_joint_state";
static const char* const cRosParamPublishTorque = "publish_torque";
static const char* const cRosParamPublishDebug = "publish_debug";

static const char* const cStrFriction = "friction";

class GazeboBldcMotor : public ModelPlugin {

public:
  GazeboBldcMotor();
  virtual ~GazeboBldcMotor();

  void Load(physics::ModelPtr aParentModel, sdf::ElementPtr aPluginSdf) override;
  void Reset() override;
  //  bool ValidateParameters();

protected:
  virtual void updateChild(const gazebo::common::UpdateInfo& aUpdateInfo);

private:
  /** Mutex for protect ros parameter settings from parameter callbacks*/
  std::recursive_mutex mParamsMutex;

  /** it is going to be the name of the joint when the plugin is loaded */
  std::string mPluginName;

  /** model that contains the motor */
  physics::ModelPtr mParent;

  /** the joint that rotates the rotor, it has the same name as plugin */
  physics::JointPtr mJoint{nullptr};

  /**
   * link that simulates the shaft of the motor behind the gearbox
   * there can be a revolute joint between mShaft and mLoadLink to simulate deadplay, in this case these links are
   *different if it is missing, the motor drives mLoadLink directly, in this case mShaft == MLoad
   *
   **/
  physics::LinkPtr mShaft{nullptr};

  /** the wheel aka. load that is finally rotated */
  physics::LinkPtr mLoadLink{nullptr};

  /**
   *  Keys of this map are the names of ROS parameters. ROS' parameter callbacks
   *  are registered to these parameter names.
   */
  std::map<std::string, double> mMotorParameters{
      {cRosParamElectricResistance, 0.54},       /// R
      {cRosParamSelfInductance, 0.5 / 1000.0},   // Ls
      {cRosParamMaximumCurrent, 13.8},           // I max
      {cRosParamMutualInductance, 0},            // Ms
      {cRosParamStatorInductanceFluctuation, 0}, // Lm
      {cRosParamPolePairs, 7},                   // p
      {cRosParamTorqueConstant, 0.133},          /// Kt
      {cRosParamRotorInertia, 102e-6},
      {cRosParamSupplyVoltage, 48.0},
      {cRosParamEncoderPulsesPerRevolution, 100}, // highlighted to member variable to speed up access
      {cRosParamStandardDeviationOfNoise, 0},     // highlighted to member mNoise
      {cRosParamGearRatio, 31.43},                // highlighted to member variable to speed up access

      {cRosParamCurrentOffset, 0.0}, // highlighted to member variable to speed up access

      {cRosParamFrictionCoefficient, 0.05},
      {cRosParamDampingCoefficient, 0.25},

      {cRosParamPiProportionalGainForVelocity, 0.3},
      {cRosParamPiIntegralGainForVelocity, 0.0},
      {cRosParamPiProportionalGainForCurrent, 0.4},
      {cRosParamPiIntegralGainForCurrent, 0.04}};

  // ROS2 params to configure topic names:

  const std::string mCommandTopicParam{cRosTopicCommand};
  const std::string mVelocityTopicParam{cRosTopicVelocity};
  const std::string mRotorVelocityTopicParam{cRosTopicRotorVelocity};
  const std::string mEncoderTopicParam{cRosTopicEncoder};
  const std::string mCurrentTopicParam{cRosTopicCurrent};
  const std::string mTorqueTopicParam{cRosTopicTorque};
  const std::string mTorqueErrorTopicParam{ cRosTopicTorqueError };

  // ROS2 subscriptions and publishers:

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mCmdVelSubscriber;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mRotorVelocityPublisher = nullptr;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mTorqueVelocityPublisher = nullptr;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mEncoderPublisher = nullptr;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mCurrentPublisher = nullptr;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mTorquePublisher = nullptr;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mTorqueErrorPublisher = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointStatePublisher = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mDebugPublisher = nullptr;

  // Flags to enable/disable publishing:

  bool mPublishDebugEnabled{true};
  bool mPublishVelocityEnabled{true};
  bool mPublishRotorVelocityEnabled{true};
  bool mPublishEncoderEnabled{true};
  bool mPublishCurrentEnabled{true};
  bool mPublishMotorJointStateEnabled{true};
  bool mPublishTorqueEnabled{true};

  /** Configuration to generate ROS2 parameters to enable/disable publishing */
  std::map<std::string, bool*> mPublishesEnabled{{cRosParamPublishVelocity, &mPublishVelocityEnabled},
                                                 {cRosParamPublishRotorVelocity, &mPublishRotorVelocityEnabled},
                                                 {cRosParamPublishEncoder, &mPublishEncoderEnabled},
                                                 {cRosParamPublishCurrent, &mPublishCurrentEnabled},
                                                 {cRosParamPublishMotorJointState, &mPublishMotorJointStateEnabled},
                                                 {cRosParamPublishTorque, &mPublishTorqueEnabled},
                                                 {cRosParamPublishDebug, &mPublishDebugEnabled}};


  /** period time of publishing calculated values (rotor velocity, joint state, etc.) */
  double mUpdatePeriod{0.01};

  /** reduction ratio, eg 10.0 means 1/10-th output angular velocity compared to motor inner vel. */
  double mGearRatio{0.0f};

  /** reduction ratio, eg 10.0 means 1/10-th output angular velocity compared to motor inner vel. */
  double mEncoderPulsesPerRevolution;

  /** last time of ROS topic publications, according to the simulation time */
  common::Time mLastUpdateTime{0};

  /** last simulation time, when update occured, according to the simulation time */
  common::Time mLastStepTime{0};

  // motor model:
  boost::shared_ptr<BLDCMotorFoC> mMotor{nullptr};
  /** 0: Current Id,  1: Current Iq,  2: Mechanical angular velocity, 3: Mechanical angle */
  nowtech::Vec4 mState{0.0, 0.0, 0.0, 0.0};
  double mMotorTorque{0};
  double mRotorVelocity{0};
  int32_t mEncoderCounter{0};

  physics::InertialPtr mpOriginalLoadLinkInertia;
  physics::InertialPtr mpLoadLinkInertia{new physics::Inertial()};

  // things for noise generation:
  std::normal_distribution<double> noise{0, mMotorParameters[cRosParamStandardDeviationOfNoise]};
  std::random_device mRandomDevice{};
  std::mt19937 gen{mRandomDevice()};

  // gazebo and ROS2 shared pointers, they must be hold:
  gazebo_ros::Node::SharedPtr mGazeboRosNode;
  std::shared_ptr<rclcpp::ParameterEventHandler> mRosParamSubscriber;
  std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> mParamCallbacksHandlers;
  event::ConnectionPtr mUpdateConnection;

  // ==================================================
  // Debug publishers:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mDebugPublisherTorqRef;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mDebugPublisherIdIntegralErr;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mDebugPublisherIqIntegralErr;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mDebugPublisherVoltageD;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mDebugPublisherVoltageQ;
  // ==================================================

  void initMotor();

  /** callback for setting "angular velocity target" */
  void cmdVelCallback(std_msgs::msg::Float32 cmd_msg);

  /** publishes data on ROS topics */
  void publish();

  /** publishes joint state */
  void publishLoadJointState(common::Time current_time);

  /** creates ROS parameters to modify names of ROS topics */
  void addPublishParameters();

  /** creates ROS parameters to tune the behavior of the plugin */
  void addRosParameters();

  /** checks the full parameter set */
  bool ValidateParameters();

  /** enforces new value of parameter in the math model */
  void commitParameterChange( std::string parameterName, double parameterValue);
};

} // namespace gazebo

#endif

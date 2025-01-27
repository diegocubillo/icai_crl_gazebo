#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

class md25_ros2_pluginPrivate;

class md25_ros2_motor
{
  /// \brief Callback for voltage command subscription

  /// \param[in] _msg Double message
  public: void OnCmdVolt(const std_msgs::msg::Float64 & msg);

  public: rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr encoderPublisher;
  public: rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr voltageSubscriber;

  /// \brief Joint Entity
  public: Entity jointEntity;
  public: std::string jointName;

  /// \brief Motor state.
  public: enum MotorState {
        DISABLED = 0,
        ENABLED = 1,
        NOT_AVAILABLE = 2
    };
  public: MotorState motorState = DISABLED;



  /// \brief Motor voltage expressed in driver register
  public: int motorVoltRegister = 0;

  /// \brief Motor voltage unquantized
  public: double motorVoltUnquantized = 0.0;

  /// \brief Commanded voltage input buffer
  public: double motorVoltCmdBuffer = 0.0;

  /// \brief mutex to protect motorVoltCmdBuffer
  public: std::mutex motorVoltCmdBufferMutex;

  /// \brief Objective register commanded for motor voltage
  public: int motorVoltCmdRegister = 0;

  /// \brief Quantized commanded motor voltage
  public: double motorVoltCmdQuantized = 0.0;


  // Encoder variables
  public: double prevJointPos = 0.0;
  public: int32_t encoderCount = 0;

  // Last time voltage was changed (in simulation time)
  public: std::chrono::_V2::steady_clock::duration prevVoltUpdateTime = std::chrono::seconds(0);


  /// \brief Motor internal variables
  public: double internalCurrent = 0.0;
  public: double prevInternalOmega = 0.0;
  public: double prevMotorVolt = 0.0;
  
  /// \brief Motor voltage
  public: double motorVolt = 0.0;

  /// \brief Encoder
  public: void EncoderSystem(EntityComponentManager &_ecm, const double &_radPerPulse);

  /// \brief Motor system
  public: void MotorSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, md25_ros2_pluginPrivate* _dataPtr,const double &_dt);
};

class md25_ros2_pluginPrivate
{
  /// \brief Callback for voltage subscription

  /// \brief ROS 2 communication node.
  public: rclcpp::Node::SharedPtr node;


  /// \brief Battery level (Not integrated with battery plugin yet)
  public: double batteryVoltage = 12.0; // Volts
  public: double electromotiveForceConstant = 0.539065; // Nm/A
  public: double electricResistance = 7.101; // Ohm
  public: double electricInductance = 0.0034; // Henry

  // transmission
  public: double gearRatio = 1.0;


  // Driver limitations

  /// \brief Register size from 0 to max voltage
  public: int registerSize = 127;

  /// \brief
  public: int voltageUpdatePeriod = 25;

  /// \brief Maximum voltage step change in 25ms
  public: int maxUpdateSteps = 10;

  public: double voltageQuantizationStep; //Volts

  // Maximum allowed voltage step in a simulation iteration
  public: double maxVoltageIncreasePerStep;

  // encoder
  // rad per pulse is calculated from sdf value of pulses per revolution
  public: double radPerPulse;

  // Encoder pulses per revolution
  public: int encoderPulsesPerRev = 360;

  // Encoder publishing rate [Hz]
  public: int encoderRate = 100;

  // Last time encoder was published (in simulation time)
  public: std::chrono::_V2::steady_clock::duration prevEncoderUpdateTime = std::chrono::seconds(0);

  // Better performance using registers instead of voltage
  public: bool performanceMode = true;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Motors
  public: md25_ros2_motor leftMotor;
  public: md25_ros2_motor rightMotor;

  public: int LoadMotorConfig(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);
  public: void AdvertiseTopics(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);
  private: int ValidateParameters();
};

class md25_ros2_plugin
    : public System, 
      public ISystemPreUpdate, 
      public ISystemConfigure
{
  public: md25_ros2_plugin();
  public: ~md25_ros2_plugin() override;
  public: void PreUpdate(const UpdateInfo &_info,
              EntityComponentManager &_ecm) override;
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;
  private: std::unique_ptr<md25_ros2_pluginPrivate> dataPtr;
};

}
}
}
}
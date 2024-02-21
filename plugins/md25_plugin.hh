#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>
#include <memory>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{

class md25_pluginPrivate;

class md25_motor
{
  /// \brief Callback for voltage command subscription

  /// \param[in] _msg Double message
  public: void OnCmdVolt(const msgs::Double &_msg);

  public: void OnTorqueMeasure(const msgs::Wrench &_msg);

  public: transport::Node::Publisher torquePublisher;

  public: transport::Node::Publisher jointVelocityPublisher;

  public: transport::Node::Publisher voltagePublisher;

  public: transport::Node::Publisher encoderPublisher;

  /// \brief Joint Entity
  public: Entity jointEntity;
  public: std::string jointName;

  /// \brief Measured torque before the motor action
  public: double measuredTorque=0.0;
  public: std::mutex measuredTorqueMutex;

  /// \brief Commanded voltage
  public: double motorVoltCmd = 0.01;
  public: double prevMotorVolt = 0.0;

  /// \brief mutex to protect motorVoltCmd
  public: std::mutex motorVoltCmdMutex;

  // encoder
  public: double prevJointPos = 0.0;
  public: int32_t encoderCount = 0;


  /// \brief motor internal current
  public: double internalCurrent = 0.0;

  /// \brief encoder
  public: void EncoderSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, const double &_radPerPulse);

  /// \brief motor system
  public: void MotorSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, md25_pluginPrivate* _dataPtr);
};

class md25_pluginPrivate
{
  /// \brief Callback for voltage subscription

  /// \brief Ignition communication node.
  public: transport::Node node;


  /// \brief Battery level (Not integrated with battery plugin yet)
  public: double batteryVoltage = 12.0; // Volts

  public: double motorNominalVoltage = 12.0; // Volts
  public: double momentOfInertia = 0.000319554; // kgm^2
  public: double armatureDampingRatio = 0.000931; // Nm/(rad/s)
  public: double electromotiveForceConstant = 0.539065; // Nm/A
  public: double electricResistance = 7.101; // Ohm
  public: double electricInductance = 0.0034; // Henry

  // transmission
  public: double gearRatio = 1.0;

  // driver limitations
  public: double voltageQuantizationStep; //Volts
  public: double maxVoltageIncreasePerStep; //Volts

  // encoder
  // rad per pulse is calculated from sdf value of pulses per revolution
  public: double radPerPulse;
  // Should be implemented in the future
  // public: int stepsPublishingRate = 10;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Motors
  public: md25_motor leftMotor;
  public: md25_motor rightMotor;

  public: void LoadMotorConfig(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);
  public: void AdvertiseTopics(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);
  private: bool ValidateParameters();
};

class md25_plugin
    : public System, 
      public ISystemPreUpdate, 
      public ISystemConfigure
{
  public: md25_plugin();
  public: ~md25_plugin() override;
  public: void PreUpdate(const UpdateInfo &_info,
              EntityComponentManager &_ecm) override;
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;
  private: std::unique_ptr<md25_pluginPrivate> dataPtr;
};

}
}
}
}
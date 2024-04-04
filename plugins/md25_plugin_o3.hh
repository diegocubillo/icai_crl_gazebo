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

class md25_plugin_o3Private;

class md25_motor
{
  /// \brief Callback for voltage command subscription

  /// \param[in] _msg Double message
  public: void OnCmdVolt(const msgs::Double &_msg);

  public: transport::Node::Publisher torquePublisher;

  public: transport::Node::Publisher jointVelocityPublisher;

  public: transport::Node::Publisher voltagePublisher;

  public: transport::Node::Publisher currentPublisher;

  public: transport::Node::Publisher encoderPublisher;

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



  /// \brief Unquantized motor voltage
  public: double motorVoltUnquantized = 0.0;

  /// \brief Commanded voltage input buffer
  public: double motorVoltCmdBuffer = 0.0;

  /// \brief mutex to protect motorVoltCmdBuffer
  public: std::mutex motorVoltCmdBufferMutex;

  /// \brief Quantized commanded motor voltage
  public: double motorVoltCmdQuantized = 0.0;


  // Encoder variables
  public: double prevJointPos = 0.0;
  public: int32_t encoderCount = 0;

  // Last time voltage was changed (in simulation time)
  public: std::chrono::_V2::steady_clock::duration prevVoltUpdateTime = std::chrono::seconds(0);


  /// \brief Motor internal variables
  public: double internalCurrent = 0.0;
  public: double internalOmegaPrev = 0.0;
  public: double prevMotorVolt = 0.0;
  
  /// \brief Motor voltage
  public: double motorVolt = 0.0;

  /// \brief Encoder
  public: void EncoderSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, const double &_radPerPulse);

  /// \brief Motor system
  public: void MotorSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, md25_plugin_o3Private* _dataPtr,const double &_dt);
};

class md25_plugin_o3Private
{
  /// \brief Callback for voltage subscription

  /// \brief Ignition communication node.
  public: transport::Node node;


  /// \brief Battery level (Not integrated with battery plugin yet)
  public: double batteryVoltage = 12.0; // Volts
  public: double motorNominalVoltage = 12.0; // Volts
  public: double electromotiveForceConstant = 0.539065; // Nm/A
  public: double electricResistance = 7.101; // Ohm
  public: double electricInductance = 0.0034; // Henry

  // transmission
  public: double gearRatio = 1.0;


  // Driver limitations

  /// \brief Register size from 0 to max voltage
  public: int registerSize = 127;

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
  public: int encoderRate = 10;

  // Last time encoder was published (in simulation time)
  public: std::chrono::_V2::steady_clock::duration prevEncoderUpdateTime = std::chrono::seconds(0);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Motors
  public: md25_motor leftMotor;
  public: md25_motor rightMotor;

  public: int LoadMotorConfig(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);
  public: void AdvertiseTopics(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);
  private: int ValidateParameters();
};

class md25_plugin_o3
    : public System, 
      public ISystemPreUpdate, 
      public ISystemConfigure
{
  public: md25_plugin_o3();
  public: ~md25_plugin_o3() override;
  public: void PreUpdate(const UpdateInfo &_info,
              EntityComponentManager &_ecm) override;
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;
  private: std::unique_ptr<md25_plugin_o3Private> dataPtr;
};

}
}
}
}
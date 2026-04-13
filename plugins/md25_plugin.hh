/// \file md25_plugin.hh
/// \brief MD25 motor driver plugin for Ignition Gazebo
///
/// This plugin simulates the behavior of an MD25 dual motor driver board,
/// providing realistic DC motor control with voltage quantization, current
/// simulation, and encoder feedback.
///
/// \author Diego Cubillo
/// \date 2025

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

/// \brief Individual motor controller class for MD25 plugin
///
/// This class represents a single DC motor with its associated control logic,
/// including voltage quantization, current calculation, and encoder simulation.
class md25_motor
{
  public:
    /// \brief Motor operational states
    enum MotorState {
        DISABLED = 0,      ///< Motor is disabled/not configured
        ENABLED = 1,       ///< Motor is enabled and operational
        NOT_AVAILABLE = 2  ///< Motor joint not found in model
    };

    /// \brief Gear backlash coupling states
    enum BacklashState {
        CONTACT_POSITIVE = 0,  ///< Engaged at positive end of backlash zone
        CONTACT_NEGATIVE = 1,  ///< Engaged at negative end of backlash zone
        FREE_PLAY = 2          ///< Motor decoupled within backlash zone
    };

    /// \brief Constructor
    md25_motor() = default;

    /// \brief Destructor
    ~md25_motor() = default;

    /// \brief Callback for voltage command subscription
    /// \param[in] _msg Double message containing voltage command in Volts
    void OnCmdVolt(const msgs::Double &_msg);

    /// \brief Updates encoder count and publishes encoder data
    /// \param[in] _info Update information from simulation
    /// \param[in] _ecm Entity Component Manager
    /// \param[in] _radPerPulse Radians per encoder pulse
    void EncoderSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, const double &_radPerPulse);

    /// \brief Main motor control system update
    /// \param[in] _info Update information from simulation
    /// \param[in] _ecm Entity Component Manager
    /// \param[in] _dataPtr Pointer to plugin private data
    /// \param[in] _dt Simulation time step in seconds
    void MotorSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, md25_pluginPrivate* _dataPtr, const double &_dt);

    /// \brief Initialize backlash angle (called after parameter loading)
    /// \param[in] _backlashWidth Total backlash zone width in radians
    void InitBacklash(double _backlashWidth)
    {
      this->backlashAngle = _backlashWidth;
      this->backlashState = CONTACT_POSITIVE;
    }

  public:
    // Publishers for motor telemetry
    transport::Node::Publisher torquePublisher;         ///< Publisher for motor output torque
    transport::Node::Publisher jointVelocityPublisher;  ///< Publisher for joint velocity
    transport::Node::Publisher voltagePublisher;        ///< Publisher for motor voltage
    transport::Node::Publisher currentPublisher;        ///< Publisher for motor current
    transport::Node::Publisher encoderPublisher;        ///< Publisher for encoder count
    transport::Node::Publisher backlashAnglePublisher;   ///< Publisher for backlash angle

    // Joint identification
    Entity jointEntity;       ///< Entity ID of the controlled joint
    std::string jointName;    ///< Name of the controlled joint
    MotorState motorState = DISABLED;  ///< Current motor state
    bool isLeftMotor = false;  ///< Whether the motor is at the left or right side of the robot

  private:
    // Voltage control variables
    int motorVoltRegister = 0;           ///< Motor voltage in driver register units
    double motorVoltUnquantized = 0.0;   ///< Unquantized motor voltage for non-performance mode
    double motorVoltCmdBuffer = 0.0;     ///< Commanded voltage input buffer
    std::mutex motorVoltCmdBufferMutex;  ///< Mutex to protect motorVoltCmdBuffer
    int motorVoltCmdRegister = 0;        ///< Target register value for motor voltage
    double motorVoltCmdQuantized = 0.0;  ///< Quantized commanded motor voltage
    double motorVolt = 0.0;              ///< Current motor voltage

    // Encoder variables
    double prevJointPos = 0.0;  ///< Previous joint position for encoder calculation
    int32_t encoderCount = 0;   ///< Current encoder count

    // Timing control
    std::chrono::_V2::steady_clock::duration prevVoltUpdateTime = std::chrono::seconds(0);  ///< Last voltage update time

    // Motor internal state variables
    double internalCurrent = 0.0;      ///< Internal motor current (A)
    double prevInternalOmega = 0.0;    ///< Previous internal angular velocity (rad/s)
    double prevMotorVolt = 0.0;        ///< Previous motor voltage for discrete model

    // Gear backlash state variables
    BacklashState backlashState = CONTACT_NEGATIVE;  ///< Current backlash coupling state
    double backlashAngle = 0.0;          ///< Current angle within backlash zone (rad)
    double internalMotorOmega = 0.0;     ///< Motor angular velocity when decoupled (rad/s)
};

/// \brief Private data class for MD25 plugin
///
/// Contains all configuration parameters and shared data for the MD25 plugin.
class md25_pluginPrivate
{
  public:
    /// \brief Constructor
    md25_pluginPrivate() = default;

    /// \brief Load motor configuration from SDF
    /// \param[in] _sdf SDF element containing plugin configuration
    /// \param[in] _ecm Entity Component Manager
    /// \return 0 on success, -1 on failure
    int LoadMotorConfig(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);

    /// \brief Advertise topics for motor communication
    /// \param[in] _sdf SDF element containing plugin configuration
    /// \param[in] _ecm Entity Component Manager
    void AdvertiseTopics(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm);

  public:
    // Communication
    transport::Node node;  ///< Ignition transport communication node

    // Motor instances
    md25_motor leftMotor;   ///< Left motor controller
    md25_motor rightMotor;  ///< Right motor controller

    // Physical parameters
    double batteryVoltage = 12.0;              ///< Battery voltage (V) - not integrated with battery plugin yet
    double electromotiveForceConstant = 0.539111;  ///< EMF constant (Nm/A)
    double electricResistance = 7.101;         ///< Electric resistance (Ohm)
    double electricInductance = 0.0034;        ///< Electric inductance (Henry)
    double motorAxisInertia = 0.00005;         ///< Rotor and gears inertia (Kg·m^2)
    double halfDifferentialVoltageDrop = -0.0104855/2.0; ///< Differential voltage between motors (V)
    double gearRatio = 1.0;                    ///< Gear ratio (motor to output)
    double backlashWidth = 1.8 * M_PI / 180;   ///< Total backlash zone width at wheel (rad)
    double motorViscousFriction = 0.0;         ///< Rotor viscous friction (Nm·s/rad), active only in free play
    double motorStaticFriction = 0.0;          ///< Rotor static friction (Nm), active only in free play 

    // Driver characteristics
    int registerSize = 127;           ///< Register size from 0 to max voltage
    int voltageUpdatePeriod = 25;     ///< Voltage update period (ms)
    int maxUpdateSteps = 10;          ///< Maximum voltage step change in 25ms
    double voltageQuantizationStep;   ///< Voltage quantization step (V)
    double maxVoltageIncreasePerStep; ///< Maximum allowed voltage step in simulation iteration

    // Encoder configuration
    double radPerPulse;              ///< Radians per encoder pulse
    int encoderPulsesPerRev = 360;   ///< Encoder pulses per revolution
    int encoderRate = 200;           ///< Encoder publishing rate (Hz)
    std::chrono::_V2::steady_clock::duration prevEncoderUpdateTime = std::chrono::seconds(0);  ///< Last encoder update time

    // Performance optimization
    bool performanceMode = true;  ///< Use registers instead of voltage for better performance

    // Model interface
    Model model{kNullEntity};  ///< Gazebo model interface

  private:
    /// \brief Validate plugin parameters
    /// \return 0 on success, -1 on failure
    int ValidateParameters();
};

/// \brief MD25 dual motor driver plugin for Ignition Gazebo
///
/// This plugin simulates the MD25 dual motor driver board, providing:
/// - Realistic DC motor control with voltage quantization
/// - Current simulation based on electrical motor model
/// - Encoder feedback simulation
/// - Configurable motor parameters and driver characteristics
///
/// ## SDF Parameters:
/// - `left_joint`: Name of the left motor joint (required)
/// - `right_joint`: Name of the right motor joint (required)
/// - `electromotive_force_constant`: EMF constant in Nm/A (default: 0.539111)
/// - `electric_resistance`: Motor resistance in Ohms (default: 7.101)
/// - `electric_inductance`: Motor inductance in Henry (default: 0.0034)
/// - `gear_ratio`: Gear ratio motor to output (default: 1.0)
/// - `backlash_width`: Gear backlash zone width at wheel in radians (default: ~0.0314)
/// - `motor_axis_inertia`: Motor rotor and gears inertia in Kg·m^2 (default: 0.00005)
/// - `motor_viscous_friction`: Rotor viscous friction in Nm·s/rad, free play only (default: 0.0)
/// - `motor_static_friction`: Rotor static friction in Nm, free play only (default: 0.0)
/// - `encoder_ppr`: Encoder pulses per revolution (default: 360)
/// - `encoder_rate`: Encoder publishing rate in Hz (default: 200)
/// - `max_update_steps`: Maximum register update steps (default: 10)
/// - `performance_mode`: Enable performance mode (default: true)
/// - `voltage_update_period`: Voltage update period in ms (default: 25)
/// - `left_volt_cmd_topic`: Custom topic for left motor voltage commands (optional)
/// - `right_volt_cmd_topic`: Custom topic for right motor voltage commands (optional)
///
/// ## Topics:
/// ### Subscribed:
/// - `/model/{model_name}/{joint_name}/motor_volt_cmd` (msgs::Double): Voltage command
///
/// ### Published:
/// - `/model/{model_name}/{joint_name}/motor_output_torque` (msgs::Double): Motor output torque
/// - `/model/{model_name}/{joint_name}/joint_velocity` (msgs::Double): Joint angular velocity
/// - `/model/{model_name}/{joint_name}/motor_voltage` (msgs::Double): Actual motor voltage
/// - `/model/{model_name}/{joint_name}/motor_current` (msgs::Double): Motor current
/// - `/model/{model_name}/{joint_name}/motor_encoder` (msgs::Int32): Encoder count
/// - `/model/{model_name}/{joint_name}/motor_backlash_angle` (msgs::Double): Backlash angle
class md25_plugin
    : public System, 
      public ISystemPreUpdate, 
      public ISystemConfigure
{
  public:
    /// \brief Constructor
    md25_plugin();

    /// \brief Destructor
    ~md25_plugin() override;

    /// \brief Configure the plugin
    /// \param[in] _entity Entity associated with this plugin
    /// \param[in] _sdf SDF element containing plugin configuration
    /// \param[in] _ecm Entity Component Manager
    /// \param[in] _eventMgr Event manager
    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   EntityComponentManager &_ecm,
                   EventManager &_eventMgr) override;

    /// \brief Update the plugin before physics update
    /// \param[in] _info Update information from simulation
    /// \param[in] _ecm Entity Component Manager
    void PreUpdate(const UpdateInfo &_info,
                   EntityComponentManager &_ecm) override;

  private:
    /// \brief Private data pointer
    std::unique_ptr<md25_pluginPrivate> dataPtr;
};

}
}
}
}
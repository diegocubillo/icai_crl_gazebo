#include "md25_ros2_plugin.hh"

#include <iostream>
#include <string>
#include <cstdint>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointPosition.hh"



using namespace ignition;
using namespace gazebo;
using namespace systems;



int md25_ros2_pluginPrivate::LoadMotorConfig(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm)
{
  // Load joint
  // Get mandatory params from SDF
  this->leftMotor.jointName = _sdf->Get<std::string>("left_joint");
  if (this->leftMotor.jointName.empty())
  {
    ignerr << "md25_ros2_plugin found an empty left_joint parameter. "
           << "This joint will not be initialized.\n";
  }
  else
  {
    // Check that mandatory params exist within the model
    this->leftMotor.jointEntity = this->model.JointByName(_ecm,
        this->leftMotor.jointName);
    if (this->leftMotor.jointEntity == kNullEntity)
    {
      ignerr << "Left joint with name [" << this->leftMotor.jointName << "] not found. "
      << "The md25_ros2_plugin may not control this joint.\n";
      this->leftMotor.motorState = md25_ros2_motor::NOT_AVAILABLE;
    }
    else this->leftMotor.motorState = md25_ros2_motor::ENABLED;
  }

  this->rightMotor.jointName = _sdf->Get<std::string>("right_joint");
  if (this->rightMotor.jointName.empty())
  {
    ignerr << "md25_ros2_plugin found an empty right_joint parameter. "
           << "This joint will not be initialized.\n";
  }
  else
  {
    // Check that mandatory params exist within the model
    this->rightMotor.jointEntity = this->model.JointByName(_ecm,
        this->rightMotor.jointName);
    if (this->rightMotor.jointEntity == kNullEntity)
    {
      ignerr << "Right joint with name [" << this->rightMotor.jointName << "] not found. "
      << "The md25_ros2_plugin may not control this joint.\n";
      this->rightMotor.motorState = md25_ros2_motor::NOT_AVAILABLE;
    }
    else this->rightMotor.motorState = md25_ros2_motor::ENABLED;
  }

  // return if both joints are disabled
  if (this->leftMotor.motorState == md25_ros2_motor::DISABLED &&
      this->rightMotor.motorState == md25_ros2_motor::DISABLED)
  {
    ignerr << "No valid joints found. The md25_ros2_plugin will not be initialized.\n";
    return -1;
  }

  // Check default values override
  if (_sdf->HasElement("electromotive_force_constant"))
  {
    this->electromotiveForceConstant = _sdf->Get<double>("electromotive_force_constant");
    ignmsg << "EMF constant initialized to ["
           << this->electromotiveForceConstant << " Nm/A]\n";
  }

  if (_sdf->HasElement("electric_resistance"))
  {
    this->electricResistance = _sdf->Get<double>("electric_resistance");
    ignmsg << "Electric resistance initialized to ["
           << this->electricResistance << " Ohm]\n";
  }

  if (_sdf->HasElement("electric_inductance"))
  {
    this->electricInductance = _sdf->Get<double>("electric_inductance");
    ignmsg << "Electric inductance initialized to ["
           << this->electricInductance << " Henry]\n";
  }

  if (_sdf->HasElement("gear_ratio"))
  {
    this->gearRatio = _sdf->Get<double>("gear_ratio");
    ignmsg << "Gear ratio initialized to ["
           << this->gearRatio << "]\n";
  }

  if (_sdf->HasElement("encoder_ppr"))
  {
    this->encoderPulsesPerRev = _sdf->Get<int>("encoder_ppr");
    ignmsg << "Encoder pulses per revolution initialized to ["
           << this->encoderPulsesPerRev << "]\n";
  }

  if (_sdf->HasElement("encoder_rate"))
  {
    encoderRate = _sdf->Get<int>("encoder_rate");
    ignmsg << "Encoder publishing rate initialized to ["
           << encoderRate << "]\n";
  }

  if (_sdf->HasElement("max_update_steps"))
  {
    maxUpdateSteps = _sdf->Get<int>("max_update_steps");
    ignmsg << "Max register update steps initialized to ["
           << maxUpdateSteps << "]\n";
  }

  if (_sdf->HasElement("performance_mode"))
  {
    performanceMode = _sdf->Get<bool>("performance_mode");
    ignmsg << "Performance mode initialized to ["
           << performanceMode << "]\n";
  }

  if (_sdf->HasElement("voltage_update_period"))
  {
    voltageUpdatePeriod = _sdf->Get<int>("voltage_update_period");
    ignmsg << "Voltage update period set to ["
           << voltageUpdatePeriod << "] milliseconds (only if performance mode is disabled).\n";
  }

  // Calculate encoder's rads per pulse
  if (this->encoderPulsesPerRev <= 0)
  {
    ignwarn << "Incorrect encoder pulses per revolution: it must be positive and non zero!\n";
  }
  else
    this->radPerPulse = 2.0 * 3.1415926 / (double)this->encoderPulsesPerRev;
  
  // Check parameters
  
  return this->ValidateParameters();
}

void md25_ros2_pluginPrivate::AdvertiseTopics(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm)
{
  // Initialize ROS 2 node
  this->node = rclcpp::Node::make_shared("md25_ros2_plugin");

  // Advertise publishers and subscribe to voltage commands if motors are not disabled
  if (this->leftMotor.motorState != md25_ros2_motor::DISABLED)
  {    
    // Subscribe to voltage commands
    std::string voltageSubscriberTopic;
    if (_sdf->HasElement("left_volt_cmd_topic"))
    {
      voltageSubscriberTopic = _sdf->Get<std::string>("left_volt_cmd_topic");
    }
    else
    {
      voltageSubscriberTopic = "/model/" + this->model.Name(_ecm) + "/" + this->leftMotor.jointName + "/cmd_volt";
    }

    this->leftMotor.voltageSubscriber = this->node->create_subscription<std_msgs::msg::Float64>(
      voltageSubscriberTopic, rclcpp::QoS(3).best_effort(), std::bind(&md25_ros2_motor::OnCmdVolt, &this->leftMotor, std::placeholders::_1));
    ignmsg << "Subscribed to topic [" << voltageSubscriberTopic << "]\n";

    // Advertise publishers
    this->leftMotor.encoderPublisher = this->node->create_publisher<std_msgs::msg::Int32>(
        "/model/" + this->model.Name(_ecm) + "/" + this->leftMotor.jointName + "/encoder",rclcpp::QoS(3).best_effort());
    ignmsg << "Advertised topic [/model/" << this->model.Name(_ecm) << "/" << this->leftMotor.jointName << "/encoder]\n";
  }

  if (this->rightMotor.motorState != md25_ros2_motor::DISABLED)
  {
    // Subscribe to voltage commands
    std::string voltageSubscriberTopic;
    if (_sdf->HasElement("right_volt_cmd_topic"))
    {
      voltageSubscriberTopic = _sdf->Get<std::string>("right_volt_cmd_topic");
    }
    else
    {
      voltageSubscriberTopic = "/model/" + this->model.Name(_ecm) + "/" + this->rightMotor.jointName + "/cmd_volt";
    }

    this->rightMotor.voltageSubscriber = this->node->create_subscription<std_msgs::msg::Float64>(
      voltageSubscriberTopic, rclcpp::QoS(3).best_effort(), std::bind(&md25_ros2_motor::OnCmdVolt, &this->rightMotor, std::placeholders::_1));
    ignmsg << "Subscribed to topic [" << voltageSubscriberTopic << "]\n";

    // Advertise publishers
    this->rightMotor.encoderPublisher = this->node->create_publisher<std_msgs::msg::Int32>(
        "/model/" + this->model.Name(_ecm) + "/" + this->rightMotor.jointName + "/encoder",rclcpp::QoS(3).best_effort());
    ignmsg << "Advertised topic [/model/" << this->model.Name(_ecm) << "/" << this->rightMotor.jointName << "/encoder]\n";
  }
}

int md25_ros2_pluginPrivate::ValidateParameters() {
    const double& L = this->electricInductance;
    const double& R = this->electricResistance;
    const double& Km = this->electromotiveForceConstant;
    int ok = 0;

    if (L<=0.0 || R<=0.0 || Km<=0.0)
    {
        ok = -1;
        ignwarn << "Incorrect DC motor parameters: R, L and Km must be positive and non zero!\n";
    }

    if (this->gearRatio<=0.0)
    {
        ok = -1;
        ignwarn << "Incorrect gear ratio: it must be positive and non zero!\n";
    }

    if (this->encoderRate<=0)
    {
        ok = -1;
        ignwarn << "Incorrect encoder rate: it must be positive and non zero!\n";
    }
    return ok;
}


md25_ros2_plugin::md25_ros2_plugin(): dataPtr(std::make_unique<md25_ros2_pluginPrivate>())
{
}

md25_ros2_plugin::~md25_ros2_plugin()
{
  rclcpp::shutdown();
}

void md25_ros2_plugin::Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm, EventManager &/*_eventMgr*/)
{
  ignmsg << "MD25 plugin configuration\n";
  // Initialize ROS 2 context
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  // Check that the plugin is attached to a model
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "md25_ros2_plugin should be attached to a model entity. "
           << "Failed to initialize.\n";
    return;
  }
  
  if (this->dataPtr->LoadMotorConfig(_sdf, _ecm) == 0)
  {
    this->dataPtr->AdvertiseTopics(_sdf, _ecm);
    // Spin ROS 2 node
    this->dataPtr->spinThread = std::thread([this]() {
      rclcpp::spin(this->dataPtr->node);
    });
  }
}

void md25_ros2_plugin::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly.\n";
  }

  if(_info.paused || (dataPtr->leftMotor.motorState == md25_ros2_motor::DISABLED && dataPtr->rightMotor.motorState == md25_ros2_motor::DISABLED))
    return;
  
  // Warn about md25 performance if dt != 1ms 
  static bool performanceWarning = false;
  if (dataPtr->performanceMode && _info.dt != std::chrono::milliseconds(1) && !performanceWarning)
  {
    performanceWarning = true;
    ignwarn << "Detected simulation step ["
        << std::chrono::duration_cast<std::chrono::milliseconds>(_info.dt).count()
        << "ms]. MD25 plugin works best at divisors of 25ms. If that's not the case, consider disabling performance mode.\n";
  }
  
  // Call encoder system at a fixed rate
  if (_info.simTime - this->dataPtr->prevEncoderUpdateTime >= std::chrono::duration<double>(1.0/this->dataPtr->encoderRate))
  {
    this->dataPtr->prevEncoderUpdateTime = _info.simTime;
    if (this->dataPtr->leftMotor.motorState != md25_ros2_motor::DISABLED)
      this->dataPtr->leftMotor.EncoderSystem(_ecm, this->dataPtr->radPerPulse);
    if (this->dataPtr->rightMotor.motorState != md25_ros2_motor::DISABLED)
      this->dataPtr->rightMotor.EncoderSystem(_ecm, this->dataPtr->radPerPulse);
  }

  // Calculate driver's voltage step (if battery voltage changes, this value should be 
  // updated in the subscriber callback)
  dataPtr->voltageQuantizationStep = dataPtr->batteryVoltage / dataPtr->registerSize;

  // Simulation sampling time in seconds
  double dt = (double)_info.dt.count()/1000000000.0;

  // Calculate maximum allowed voltage step for current sampling time
  if(!dataPtr->performanceMode)
    dataPtr->maxVoltageIncreasePerStep = dataPtr->maxUpdateSteps * dataPtr->voltageQuantizationStep*dt*dataPtr->voltageUpdatePeriod/0.025;

  // Update motor
  if (this->dataPtr->leftMotor.motorState != md25_ros2_motor::DISABLED)
    this->dataPtr->leftMotor.MotorSystem(_info, _ecm, this->dataPtr.get(), dt);
  if (this->dataPtr->rightMotor.motorState != md25_ros2_motor::DISABLED)
    this->dataPtr->rightMotor.MotorSystem(_info, _ecm, this->dataPtr.get(), dt);
}


void md25_ros2_motor::MotorSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, md25_ros2_pluginPrivate* _dataPtr, const double &_dt)
{
  // Create joint velocity component if one doesn't exist
  auto jointVelComp =
      _ecm.Component<components::JointVelocity>(this->jointEntity);
  if (jointVelComp == nullptr)
  {
    _ecm.CreateComponent(
        this->jointEntity, components::JointVelocity());
  }
  if (jointVelComp == nullptr)
    return;
  
  if (_info.simTime - this->prevVoltUpdateTime >= std::chrono::milliseconds(_dataPtr->voltageUpdatePeriod))
  {
    this->prevVoltUpdateTime = _info.simTime;
    
    if (_dataPtr->performanceMode){
      // Quantize and limit motor voltage command
      {
        std::lock_guard<std::mutex> lock(this->motorVoltCmdBufferMutex);
        this->motorVoltCmdRegister = std::clamp(static_cast<int>(this->motorVoltCmdBuffer/_dataPtr->voltageQuantizationStep),
                      -1*_dataPtr->registerSize, _dataPtr->registerSize);
      }

      // Apply upper and lower voltage step limits
      if (abs(this->motorVoltCmdRegister - this->motorVoltRegister) > _dataPtr->maxUpdateSteps)
      {
        (this->motorVoltCmdRegister > this->motorVoltRegister) ?
        this->motorVoltRegister += _dataPtr->maxUpdateSteps :
        this->motorVoltRegister -= _dataPtr->maxUpdateSteps;
      }
      else this->motorVoltRegister = this->motorVoltCmdRegister;

      // Get motor input voltage from register value to quantized voltage
      this->motorVolt = _dataPtr->voltageQuantizationStep*this->motorVoltRegister;
      // ignmsg << "Motor voltage command in joint [" << this->jointName << "] updated: " << this->motorVolt << " V\n";
    }else{
      // Quantize and limit motor voltage command
    {
      std::lock_guard<std::mutex> lock(this->motorVoltCmdBufferMutex);
      this->motorVoltCmdQuantized = std::clamp(this->motorVoltCmdBuffer,
                    -1*_dataPtr->batteryVoltage, _dataPtr->batteryVoltage);
    }

    this->motorVoltCmdQuantized = _dataPtr->voltageQuantizationStep*std::trunc(this->motorVoltCmdQuantized/_dataPtr->voltageQuantizationStep);

    // Apply upper and lower voltage step limits
    if (abs(this->motorVoltCmdQuantized - this->motorVoltUnquantized) > _dataPtr->maxVoltageIncreasePerStep)
    {
      (this->motorVoltCmdQuantized > this->motorVoltUnquantized) ?
      this->motorVoltUnquantized += _dataPtr->maxVoltageIncreasePerStep :
      this->motorVoltUnquantized -= _dataPtr->maxVoltageIncreasePerStep;
    }
    else this->motorVoltUnquantized = this->motorVoltCmdQuantized;

    // Quantization of motor input voltage
    this->motorVolt = _dataPtr->voltageQuantizationStep*std::trunc(this->motorVoltUnquantized/_dataPtr->voltageQuantizationStep);
    // ignmsg << "Motor voltage command in joint [" << this->jointName << "] updated: " << this->motorVolt << " V\n";
    }
  }

  if (!jointVelComp->Data().empty())
  {
    // Angular velocity in the rotor
    double internalOmega = jointVelComp->Data().at(0) * _dataPtr->gearRatio;
    
    // Shorter references to motor parameters
    const double& L = _dataPtr->electricInductance;
    const double& R = _dataPtr->electricResistance;
    const double& Km = _dataPtr->electromotiveForceConstant;
    const double& oPrev = this->prevInternalOmega;
    const double& iPrev = this->internalCurrent;
    const double& o0 = internalOmega;

    // Internal current calculated with a discrete mathematical model of the motor
    // Applying $i[k] = \frac{V[k] + V[k-1] - K_e (\omega [k] + \omega [k-1])-(R-\frac{2L}{T})\cdot i[k-1]}{(R+\frac{2L}{T})}$
    this->internalCurrent = (this->motorVolt + this->prevMotorVolt - Km * (o0 + oPrev) - (R - 2*L/_dt) * iPrev) / (R + 2*L/_dt);
    
    // Torque in the output shaft
    double torque = Km * this->internalCurrent * _dataPtr->gearRatio;
    
    // Update internal variables storing previous step values
    this->prevInternalOmega = o0;
    this->prevMotorVolt = motorVolt;


    
    // Apply torque to the joint
    auto torqueComp =
        _ecm.Component<components::JointForceCmd>(this->jointEntity);
    if (torqueComp == nullptr)
    {
      _ecm.CreateComponent(this->jointEntity,
                           components::JointForceCmd({torque}));
    }
    else
    {
      torqueComp->Data()[0] = torque;
      // ignmsg << "Torque in joint [" << this->jointName << "] updated: " << torque << " Nm\n";
    }
  }
}

  //subscriber message reception function
void md25_ros2_motor::OnCmdVolt(const std_msgs::msg::Float64 & msg)
{
  std::lock_guard<std::mutex> lock(this->motorVoltCmdBufferMutex);
  if (std::isnan(msg.data)) ignerr << "Received NaN voltage command. Ignoring.\n";
  else this->motorVoltCmdBuffer = msg.data;
}

void md25_ros2_motor::EncoderSystem(EntityComponentManager &_ecm, const double &_radPerPulse)
{
  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->jointEntity == kNullEntity)
    return;
  
  // Read joint position
  auto jointPosComp =
      _ecm.Component<components::JointPosition>(this->jointEntity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
        this->jointEntity, components::JointPosition());
  }

  if (jointPosComp == nullptr)
    return;  

  if (!jointPosComp->Data().empty())
  {
    double currentJointPos = jointPosComp->Data().at(0);
    int pulsesInc = (currentJointPos - this->prevJointPos)/_radPerPulse;
    if (pulsesInc)
    {
      this->encoderCount += pulsesInc;
      this->prevJointPos = this->encoderCount * _radPerPulse;
    }
   
      
    std_msgs::msg::Int32 encoderMsg;
    encoderMsg.data = this->encoderCount;
    this->encoderPublisher->publish(encoderMsg);
  }
  else ignerr << "No joint position read\n";
}


IGNITION_ADD_PLUGIN(md25_ros2_plugin,
    System,
    md25_ros2_plugin::ISystemPreUpdate,
    md25_ros2_plugin::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(md25_ros2_plugin,
                          "icai_crl_gazebo::MD25Plugin")
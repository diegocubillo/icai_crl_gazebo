#include "md25_plugin.hh"

#include <iostream>
#include <string>
#include <cstdint>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include <ignition/gazebo/Model.hh>



using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::md25_pluginPrivate
{
  /// \brief Callback for velocity subscription

  /// \param[in] _msg Velocity message
  public: void OnCmdVolt(const msgs::Double &_msg);

  public: void OnTorqueMeasure(const msgs::Wrench &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  public: transport::Node::Publisher torquePublisher;

  public: transport::Node::Publisher jointVelocityPublisher;

  public: transport::Node::Publisher voltagePublisher;

  public: transport::Node::Publisher encoderPublisher;

  /// \brief Joint Entity
  public: Entity jointEntity;

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

  public: double measuredTorque=0.0;
  public: std::mutex measuredTorqueMutex;

  /// \brief Commanded joint velocity
  public: double MotorVoltCmd = 0.0;
  public: double prevMotorVolt = 0.0;

  // encoder
  public: double prevJointPos = 0.0;
  public: int encoderPulsesPerRev = 360;
  public: double radPerPulse;
  public: int32_t encoderCount = 0;
  public: int stepsPublishingRate = 10;

  /// \brief mutex to protect MotorVoltCmd
  public: std::mutex MotorVoltCmdMutex;

  /// \brief voltage reference
  public: double voltageCmd = 0.01;

  /// \brief motor internal current
  public: double internalCurrent = 0.0;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

md25_plugin::md25_plugin(): dataPtr(std::make_unique<md25_pluginPrivate>())
{
}

md25_plugin::~md25_plugin()
{
}

bool md25_plugin::ValidateParameters() { // OOOOOOOOOOOOOOOOOOOOOOOOOOOOO change to single motor 0000000000000000000000000000000000000
    const double& d = this->dataPtr->armatureDampingRatio;
    const double& L = this->dataPtr->electricInductance;
    const double& R = this->dataPtr->electricResistance;
    const double& Km = this->dataPtr->electromotiveForceConstant;
    const double& J = this->dataPtr->momentOfInertia;
    bool ok = true;
    // Check if d^2 L^2 + J^2 R^2 - 2 J L (2 Km^2 + d R) > 0 (which appears under sqrt)
    double Om = 0; //OOOOOOOOOOOOOOOOOOOOOOOOOOOOO why initializing to zero 000000000000000000000000000000000000
    if (d*d*L*L + J*J*R*R > 2*J*L*(2*Km*Km + d*R)) {
        Om = sqrt(d*d*L*L + J*J*R*R - 2*J*L*(2*Km*Km + d*R));
        // OK, roots are real, not complex
    } else {
        ok = false;
        ignwarn << "Incorrect DC motor parameters: d^2 L^2 + J^2 R^2 - 2 J L (2 Km^2 + d R) > 0 not satisfied!" << std::endl;
    }

    if (ok) {
      // Check if -dL-JR+Om < 0 (Other real root is always negative: -dL-JR-Om)
      if (Om < d*L+J*R) {
        // OK, both real roots are stable
      } else {
        ok = false;
        ignwarn << "Incorrect DC motor parameters: sqrt(d^2 L^2 + J^2 R^2 - 2 J L (2 Km^2 + d R)) < d*L+J*R not satisfied!" << std::endl;
      }
    }
    return ok;
}


void md25_plugin::Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &/*_eventMgr*/)
{
    // Check that the plugin is attached to a model
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "md25_plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  
    // Get mandatory params from SDF
  auto jointName = _sdf->Get<std::string>("joint_name");
  if (jointName.empty())
  {
    ignerr << "md25_plugin found an empty joint_name parameter. "
           << "Failed to initialize.";
    return;
  }
  
    // Check that mandatory params exist within the model
  this->dataPtr->jointEntity = this->dataPtr->model.JointByName(_ecm,
      jointName);
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    ignerr << "Joint with name [" << jointName << "] not found. "
    << "The md25_plugin may not control this joint.\n";
    return;
  }



    // Check default (EMG30) override
  if (_sdf->HasElement("motor_nominal_voltage"))
  {
    this->dataPtr->motorNominalVoltage = _sdf->Get<double>("motor_nominal_voltage");
    ignmsg << "Motor nominal voltage initialized to ["
           << this->dataPtr->motorNominalVoltage << " Volt]" << std::endl;
  }

  if (_sdf->HasElement("moment_of_inertia"))
  {
    this->dataPtr->momentOfInertia = _sdf->Get<double>("moment_of_inertia");
    ignmsg << "Moment of inertia initialized to ["
           << this->dataPtr->momentOfInertia << " kgm^2]" << std::endl;
  }

  if (_sdf->HasElement("armature_damping_ratio"))
  {
    this->dataPtr->armatureDampingRatio = _sdf->Get<double>("armature_damping_ratio");
    ignmsg << "Armature damping ratio initialized to ["
           << this->dataPtr->armatureDampingRatio << " Nm/(rad/s)]" << std::endl;
  }

  if (_sdf->HasElement("electromotive_force_constant"))
  {
    this->dataPtr->electromotiveForceConstant = _sdf->Get<double>("electromotive_force_constant");
    ignmsg << "EMF constant initialized to ["
           << this->dataPtr->electromotiveForceConstant << " Nm/A]" << std::endl;
  }

  if (_sdf->HasElement("electric_resistance"))
  {
    this->dataPtr->electricResistance = _sdf->Get<double>("electric_resistance");
    ignmsg << "Electric resistance initialized to ["
           << this->dataPtr->electricResistance << " Ohm]" << std::endl;
  }

  if (_sdf->HasElement("electric_inductance"))
  {
    this->dataPtr->electricInductance = _sdf->Get<double>("electric_inductance");
    ignmsg << "Electric inductance initialized to ["
           << this->dataPtr->electricInductance << " Henry]" << std::endl;
  }

  if (_sdf->HasElement("gear_ratio"))
  {
    this->dataPtr->gearRatio = _sdf->Get<double>("gear_ratio");
    ignmsg << "Gear ratio initialized to ["
           << this->dataPtr->gearRatio << "]" << std::endl;
  }

  if (_sdf->HasElement("encoder_ppr"))
  {
    this->dataPtr->encoderPulsesPerRev = _sdf->Get<int>("encoder_ppr");
    ignmsg << "Encoder pulses per revolution initialized to ["
           << this->dataPtr->encoderPulsesPerRev << "]" << std::endl;
  }

  // Calculate driver's voltage step
  this->dataPtr->voltageQuantizationStep = this->dataPtr->motorNominalVoltage / 127.0;

  // Calculate encoder's rads per pulse
  this->dataPtr->radPerPulse = 2.0*3.1415926/(double)this->dataPtr->encoderPulsesPerRev;
  
  // Check parameters
  md25_plugin::ValidateParameters();


  // TO DO: NEED A VERIFICATION OF VALID TOPIC NAMES AND ADVERTISING OF PUBLISHERS VIA IGNMSG
  // Advertise publisher
  this->dataPtr->torquePublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + jointName + "/motor_output_torque"); 

  this->dataPtr->jointVelocityPublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + jointName + "/joint_velocity"); 

  this->dataPtr->voltagePublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + jointName + "/motor_voltage"); 

  this->dataPtr->encoderPublisher = this->dataPtr->node.Advertise<msgs::Int32>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + jointName + "/motor_encoder"); 

  
  // Subscribe to commands
  std::string topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/" + jointName +
      "/motor_voltage_cmd");
  if (topic.empty())
  {
    ignerr << "Failed to create topic for joint [" << jointName
           << "]" << std::endl;
    return;
  }
  if (_sdf->HasElement("topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("topic"));

    if (topic.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("topic")
             << "]" << " for joint [" << jointName
             << "]" << std::endl;
      return;
    }
  }
  this->dataPtr->node.Subscribe(topic, &md25_pluginPrivate::OnCmdVolt,
                                this->dataPtr.get());

  ignmsg << "md25_plugin subscribing to Double messages on [" << topic
         << "]" << std::endl;

  



  topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->model.Name(_ecm) + "/" + jointName +
      "/torque_sensor");
  if (topic.empty())
  {
    ignerr << "Failed to create topic for joint [" << jointName
           << "]" << std::endl;
    return;
  }
  if (_sdf->HasElement("torque_topic"))
  {
    topic = transport::TopicUtils::AsValidTopic(
        _sdf->Get<std::string>("torque_topic"));

    if (topic.empty())
    {
      ignerr << "Failed to create topic [" << _sdf->Get<std::string>("torque_topic")
             << "]" << " for joint [" << jointName
             << "]" << std::endl;
      return;
    }
  }
  this->dataPtr->node.Subscribe(topic, &md25_pluginPrivate::OnTorqueMeasure,
                                this->dataPtr.get());

  ignmsg << "md25_plugin subscribing to Wrench messages on [" << topic
         << "]" << std::endl;
}

void md25_plugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if(_info.paused)
    return;
  
  
  // Read joint position
  auto jointPosComp =
      _ecm.Component<components::JointPosition>(this->dataPtr->jointEntity);
  if (jointPosComp == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity, components::JointPosition());
  }

  if (jointPosComp == nullptr)
    return;  

  if (!jointPosComp->Data().empty())
  {
    double currentJointPos = jointPosComp->Data().at(0);
    int pulsesInc = (currentJointPos - this->dataPtr->prevJointPos)/this->dataPtr->radPerPulse;
    if (pulsesInc)
    {
      this->dataPtr->encoderCount += pulsesInc;
      this->dataPtr->prevJointPos = this->dataPtr->encoderCount * this->dataPtr->radPerPulse;
    }
   
      
    msgs::Int32 encoderMsg;
    encoderMsg.set_data(this->dataPtr->encoderCount);
    encoderMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->encoderPublisher.Publish(encoderMsg);
  }
  else std::cout << "Sin posicion" << std::endl << std::endl;
  //================================================================================================
  //OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO translate to english 0000000000000000000000000000000000000000
  


  // Create joint velocity component if one doesn't exist
  auto jointVelComp =
      _ecm.Component<components::JointVelocity>(this->dataPtr->jointEntity);
  if (jointVelComp == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity, components::JointVelocity());
  }
  if (jointVelComp == nullptr)
    return;  
  

  if (!jointVelComp->Data().empty())
  {
    // Publish joint angular velocity
    msgs::Double jointVelMsg;
    jointVelMsg.set_data(jointVelComp->Data().at(0));
    jointVelMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->jointVelocityPublisher.Publish(jointVelMsg);
    
    // Read last received voltage reference
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->MotorVoltCmdMutex);
      this->dataPtr->voltageCmd = this->dataPtr->MotorVoltCmd; // voltage input for motor
    }


    // Battery voltage limitation
    double V_motor = std::clamp(this->dataPtr->voltageCmd,-1*this->dataPtr->batteryVoltage,this->dataPtr->batteryVoltage);

    // Max voltage step for current sample time
    double dt = (double)_info.dt.count()/1000000000.0;
    this->dataPtr->maxVoltageIncreasePerStep = 10.0*this->dataPtr->voltageQuantizationStep*dt/0.025;
    //OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO Emplain how this works 0000000000000000000000000000000000000000000000000000000000

    // Apply upper and lower voltage step limits
    if (abs(V_motor - this->dataPtr->prevMotorVolt) > this->dataPtr->maxVoltageIncreasePerStep)
    {
      (V_motor > this->dataPtr->prevMotorVolt) ? V_motor = this->dataPtr->prevMotorVolt + this->dataPtr->maxVoltageIncreasePerStep : V_motor = this->dataPtr->prevMotorVolt - this->dataPtr->maxVoltageIncreasePerStep;
    }
    this->dataPtr->prevMotorVolt = V_motor;

    // Quantization of motor input voltage
    V_motor = this->dataPtr->voltageQuantizationStep*std::trunc(V_motor/this->dataPtr->voltageQuantizationStep);

    // Publish actual motor voltage input
    msgs::Double voltMsg;
    voltMsg.set_data(V_motor);
    voltMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->voltagePublisher.Publish(voltMsg);

    double T;
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->measuredTorqueMutex);
      T = this->dataPtr->measuredTorque / this->dataPtr->gearRatio; // external loading torque converted to internal side
    }

    // Calculate internal current with a discrete mathematical model of the motor
    double internalOmega = jointVelComp->Data().at(0) * this->dataPtr->gearRatio;
    const double& d = this->dataPtr->armatureDampingRatio;
    const double& L = this->dataPtr->electricInductance;
    const double& R = this->dataPtr->electricResistance;
    const double& Km = this->dataPtr->electromotiveForceConstant;
    const double& J = this->dataPtr->momentOfInertia;
    double i0 = this->dataPtr->internalCurrent;
    double o0 = internalOmega;
    double d2 = pow(d,2);
    double L2 = pow(L,2);
    double J2 = pow(J,2);
    double R2 = pow(R,2);
    double Km2 = pow(Km,2);
    double Km3 = Km2 * Km;
    double Om = sqrt(d2*L2 + J2*R2 - 2*J*L*(2*Km2 + d*R));
    double eOp1 = exp((Om*dt)/(J*L)) + 1.0;
    double eOm1 = eOp1 - 2.0; // = exp((Om*t)/(J*L)) - 1.0;
    double eA = exp(((d*L + Om + J*R)*dt)/(2.0*J*L));
    double emA = 1.0/eA; // = exp(-((d*L + Om + J*R)*t)/(2.0*J*L));
    double i_t = (emA*(i0*(Km2 + d*R)*(d*L*(d*eOp1*L + eOm1*Om) + eOp1*J2*R2 - J*(4*eOp1*Km2*L + 2*d*eOp1*L*R + eOm1*Om*R)) - d*L*(d*(-2*eA + eOp1)*L + eOm1*Om)*(Km*T + d*V_motor) - (-2*eA + eOp1)*J2*R2*(Km*T + d*V_motor) + J*(Km3*(-2*eOm1*o0*Om + 4*(-2*eA + eOp1)*L*T) - Km*R*(2*d*eOm1*o0*Om - 2*d*(-2*eA + eOp1)*L*T + eOm1*Om*T) + 2*Km2*(2*d*(-2*eA + eOp1)*L + eOm1*Om)*V_motor + d*(2*d*(-2*eA + eOp1)*L + eOm1*Om)*R*V_motor)))/ (2.*(Km2 + d*R)*(d2*L2 + J2*R2 - 2*J*L*(2*Km2 + d*R)));
    // double i_t = (V_motor*0.1-0.008*internalOmega)/R; //Modificacion para primeras pruebas
    //OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO Remove this comment 000000000000000000000000000000000000000000000000000

    double torque = Km * i_t * this->dataPtr->gearRatio;

    auto forceComp =
        _ecm.Component<components::JointForceCmd>(this->dataPtr->jointEntity);
    if (forceComp == nullptr)
    {
      _ecm.CreateComponent(this->dataPtr->jointEntity,
                           components::JointForceCmd({torque}));
    }
    else
    {
      forceComp->Data()[0] = torque;
    }

    msgs::Double torqueMsg;
    torqueMsg.set_data(torque);
    torqueMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->torquePublisher.Publish(torqueMsg);
  }
  
}

  //subscriber message reception functions
void md25_pluginPrivate::OnCmdVolt(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->MotorVoltCmdMutex);
  this->MotorVoltCmd = _msg.data();
}

void md25_pluginPrivate::OnTorqueMeasure(const msgs::Wrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->measuredTorqueMutex);
  this->measuredTorque = _msg.torque().z();
}


IGNITION_ADD_PLUGIN(md25_plugin,
    System,
    md25_plugin::ISystemPreUpdate,
    md25_plugin::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(md25_plugin,
                          "icai_crl_gazebo::MD25Plugin")
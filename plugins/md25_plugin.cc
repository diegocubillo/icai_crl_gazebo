#include "md25_plugin.hh"

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



void md25_pluginPrivate::LoadMotorConfig(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm)
{
  // Loads sdf value to calculate rad per pulse
  int encoderPulsesPerRev = 360;
  // Load joint
  // Get mandatory params from SDF
  this->leftMotor.jointName = _sdf->Get<std::string>("left_joint");
  if (this->leftMotor.jointName.empty())
  {
    ignerr << "md25_plugin found an empty left_joint parameter. "
           << "Failed to initialize.";
    return;
  }

  // Check that mandatory params exist within the model
  this->leftMotor.jointEntity = this->model.JointByName(_ecm,
      this->leftMotor.jointName);
  if (this->leftMotor.jointEntity == kNullEntity)
  {
    ignerr << "Left joint with name [" << this->leftMotor.jointName << "] not found. "
    << "The md25_plugin may not control this joint.\n";
    return;
  }

  // Check default values override
  if (_sdf->HasElement("moment_of_inertia"))
  {
    this->momentOfInertia = _sdf->Get<double>("moment_of_inertia");
    ignmsg << "Moment of inertia initialized to ["
           << this->momentOfInertia << " kgm^2]" << std::endl;
  }

  if (_sdf->HasElement("armature_damping_ratio"))
  {
    this->armatureDampingRatio = _sdf->Get<double>("armature_damping_ratio");
    ignmsg << "Armature damping ratio initialized to ["
           << this->armatureDampingRatio << " Nm/(rad/s)]" << std::endl;
  }

  if (_sdf->HasElement("electromotive_force_constant"))
  {
    this->electromotiveForceConstant = _sdf->Get<double>("electromotive_force_constant");
    ignmsg << "EMF constant initialized to ["
           << this->electromotiveForceConstant << " Nm/A]" << std::endl;
  }

  if (_sdf->HasElement("electric_resistance"))
  {
    this->electricResistance = _sdf->Get<double>("electric_resistance");
    ignmsg << "Electric resistance initialized to ["
           << this->electricResistance << " Ohm]" << std::endl;
  }

  if (_sdf->HasElement("electric_inductance"))
  {
    this->electricInductance = _sdf->Get<double>("electric_inductance");
    ignmsg << "Electric inductance initialized to ["
           << this->electricInductance << " Henry]" << std::endl;
  }

  if (_sdf->HasElement("gear_ratio"))
  {
    this->gearRatio = _sdf->Get<double>("gear_ratio");
    ignmsg << "Gear ratio initialized to ["
           << this->gearRatio << "]" << std::endl;
  }

  if (_sdf->HasElement("encoder_ppr"))
  {
    encoderPulsesPerRev = _sdf->Get<int>("encoder_ppr");
    ignmsg << "Encoder pulses per revolution initialized to ["
           << encoderPulsesPerRev << "]" << std::endl;
  }

// Calculate driver's voltage step
  this->voltageQuantizationStep = this->motorNominalVoltage / 127.0;

  // Calculate encoder's rads per pulse
  this->radPerPulse = 2.0*3.1415926/(double)encoderPulsesPerRev;
  
  // Check parameters
  this->ValidateParameters();
}

void md25_pluginPrivate::AdvertiseTopics(const std::shared_ptr<const sdf::Element> &_sdf, EntityComponentManager &_ecm)
{
  // Advertise publisher
  this->leftMotor.torquePublisher = this->node.Advertise<msgs::Double>(
      "/model/" + this->model.Name(_ecm) + "/" + this->leftMotor.jointName + "/motor_output_torque"); 

  this->leftMotor.jointVelocityPublisher = this->node.Advertise<msgs::Double>(
      "/model/" + this->model.Name(_ecm) + "/" + this->leftMotor.jointName + "/joint_velocity"); 

  this->leftMotor.voltagePublisher = this->node.Advertise<msgs::Double>(
      "/model/" + this->model.Name(_ecm) + "/" + this->leftMotor.jointName + "/motor_voltage"); 

  this->leftMotor.encoderPublisher = this->node.Advertise<msgs::Int32>(
      "/model/" + this->model.Name(_ecm) + "/" + this->leftMotor.jointName + "/motor_encoder");

  // Subscribe to commands
  std::string topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->model.Name(_ecm) + "/" + this->leftMotor.jointName +
      "/motor_voltage_cmd");
  if (topic.empty())
  {
    ignerr << "Failed to create topic for joint [" << this->leftMotor.jointName 
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
             << "]" << " for joint [" << this->leftMotor.jointName 
             << "]" << std::endl;
      return;
    }
  }
  this->node.Subscribe(topic, &md25_motor::OnCmdVolt, &(this->leftMotor));

  topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->model.Name(_ecm) + "/" + this->leftMotor.jointName +
      "/torque_sensor");
  if (topic.empty())
  {
    ignerr << "Failed to create topic for joint [" << this->leftMotor.jointName 
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
             << "]" << " for joint [" << this->leftMotor.jointName 
             << "]" << std::endl;
      return;
    }
  }
  this->node.Subscribe(topic, &md25_motor::OnTorqueMeasure,
                                &(this->leftMotor));

  ignmsg << "md25_plugin subscribing to Wrench messages on [" << topic
          << "]" << std::endl;
}

bool md25_pluginPrivate::ValidateParameters() {
    const double& d = this->armatureDampingRatio;
    const double& L = this->electricInductance;
    const double& R = this->electricResistance;
    const double& Km = this->electromotiveForceConstant;
    const double& J = this->momentOfInertia;
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


md25_plugin::md25_plugin(): dataPtr(std::make_unique<md25_pluginPrivate>())
{
}

md25_plugin::~md25_plugin()
{
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


    // Check default values override
  if (_sdf->HasElement("motor_nominal_voltage"))
  {
    this->dataPtr->motorNominalVoltage = _sdf->Get<double>("motor_nominal_voltage");
    ignmsg << "Motor nominal voltage initialized to ["
           << this->dataPtr->motorNominalVoltage << " Volt]" << std::endl;
  }
  
  this->dataPtr->LoadMotorConfig(_sdf, _ecm);

  this->dataPtr->AdvertiseTopics(_sdf, _ecm);
}

void md25_plugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if(_info.paused)
    return;
  
  // Update encoder
  this->dataPtr->leftMotor.EncoderSystem(_info, _ecm, this->dataPtr->radPerPulse);

  // Update motor
  this->dataPtr->leftMotor.MotorSystem(_info, _ecm, this->dataPtr.get());
}


void md25_motor::MotorSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, md25_pluginPrivate* _dataPtr)
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
  

  if (!jointVelComp->Data().empty())
  {
    // Publish joint angular velocity
    msgs::Double jointVelMsg;
    jointVelMsg.set_data(jointVelComp->Data().at(0));
    jointVelMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->jointVelocityPublisher.Publish(jointVelMsg);
   
    // Real motor voltage supplied by the driver
    double motorVolt;

    // Battery voltage limitation
    {
      std::lock_guard<std::mutex> lock(this->motorVoltCmdMutex);
      motorVolt = std::clamp(this->motorVoltCmd,-1*_dataPtr->batteryVoltage,_dataPtr->batteryVoltage);
    }

    // Max voltage step for current sample time
    double dt = (double)_info.dt.count()/1000000000.0;
    _dataPtr->maxVoltageIncreasePerStep = 10.0*_dataPtr->voltageQuantizationStep*dt/0.025;
    //OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO Explain how this works 0000000000000000000000000000000000000000000000000000000000
    //OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO Maybe move to configure 000000000000000000000000000000000000000000000000000000000

    // Apply upper and lower voltage step limits
    if (abs(motorVolt - this->prevMotorVolt) > _dataPtr->maxVoltageIncreasePerStep)
    {
      (motorVolt > this->prevMotorVolt) ? motorVolt = this->prevMotorVolt + _dataPtr->maxVoltageIncreasePerStep : motorVolt = this->prevMotorVolt - _dataPtr->maxVoltageIncreasePerStep;
    }
    this->prevMotorVolt = motorVolt;

    // Quantization of motor input voltage
    motorVolt = _dataPtr->voltageQuantizationStep*std::trunc(motorVolt/_dataPtr->voltageQuantizationStep);

    // Publish actual motor voltage input
    msgs::Double voltMsg;
    voltMsg.set_data(motorVolt);
    voltMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->voltagePublisher.Publish(voltMsg);

    double T;
    {
      std::lock_guard<std::mutex> lock(this->measuredTorqueMutex);
      T = this->measuredTorque / _dataPtr->gearRatio; // external loading torque converted to internal side
    }

    // Calculate internal current with a discrete mathematical model of the motor
    double internalOmega = jointVelComp->Data().at(0) * _dataPtr->gearRatio;
    const double& d = _dataPtr->armatureDampingRatio;
    const double& L = _dataPtr->electricInductance;
    const double& R = _dataPtr->electricResistance;
    const double& Km = _dataPtr->electromotiveForceConstant;
    const double& J = _dataPtr->momentOfInertia;
    double i0 = this->internalCurrent;
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
    double i_t = (emA*(i0*(Km2 + d*R)*(d*L*(d*eOp1*L + eOm1*Om) + eOp1*J2*R2 - J*(4*eOp1*Km2*L + 2*d*eOp1*L*R + eOm1*Om*R)) - d*L*(d*(-2*eA + eOp1)*L + eOm1*Om)*(Km*T + d*motorVolt) - (-2*eA + eOp1)*J2*R2*(Km*T + d*motorVolt) + J*(Km3*(-2*eOm1*o0*Om + 4*(-2*eA + eOp1)*L*T) - Km*R*(2*d*eOm1*o0*Om - 2*d*(-2*eA + eOp1)*L*T + eOm1*Om*T) + 2*Km2*(2*d*(-2*eA + eOp1)*L + eOm1*Om)*motorVolt + d*(2*d*(-2*eA + eOp1)*L + eOm1*Om)*R*motorVolt)))/ (2.*(Km2 + d*R)*(d2*L2 + J2*R2 - 2*J*L*(2*Km2 + d*R)));
    this->internalCurrent = i_t;
    //OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO Verify this is correct 000000000000000000000000000

    double torque = Km * i_t * _dataPtr->gearRatio;

    auto forceComp =
        _ecm.Component<components::JointForceCmd>(this->jointEntity);
    if (forceComp == nullptr)
    {
      _ecm.CreateComponent(this->jointEntity,
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
    this->torquePublisher.Publish(torqueMsg);
  }
  
}

  //subscriber message reception functions
void md25_motor::OnCmdVolt(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->motorVoltCmdMutex);
  this->motorVoltCmd = _msg.data();
}

void md25_motor::OnTorqueMeasure(const msgs::Wrench &_msg)
{
  std::lock_guard<std::mutex> lock(this->measuredTorqueMutex);
  this->measuredTorque = _msg.torque().z();
}

void md25_motor::EncoderSystem(const UpdateInfo &_info, EntityComponentManager &_ecm, const double &_radPerPulse)
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
   
      
    msgs::Int32 encoderMsg;
    encoderMsg.set_data(this->encoderCount);
    encoderMsg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->encoderPublisher.Publish(encoderMsg);
  }
  else std::cout << "No joint position read" << std::endl << std::endl;
}


IGNITION_ADD_PLUGIN(md25_plugin,
    System,
    md25_plugin::ISystemPreUpdate,
    md25_plugin::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(md25_plugin,
                          "icai_crl_gazebo::MD25Plugin")
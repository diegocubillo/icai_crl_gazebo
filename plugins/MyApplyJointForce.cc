/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "MyApplyJointForce.hh"

#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointForce.hh"
#include "ignition/gazebo/components/AngularAcceleration.hh"

// For some reason, convert<msgs::Time>() is defined here, but not in JointForceCmd.hh
#include "ignition/gazebo/components/JointVelocity.hh"

#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;



//////////////////////////////////////////////////
MyApplyJointForce::MyApplyJointForce()
  : dataPtr(std::make_unique<MyApplyJointForcePrivate>())
{
}

//////////////////////////////////////////////////
void MyApplyJointForce::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "MyApplyJointForce plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  // Get params from SDF
  auto sdfElem = sdfClone->GetElement("joint_name");
  if (sdfElem)
  {
    this->dataPtr->jointName = sdfElem->Get<std::string>();
  }

  if (this->dataPtr->jointName == "")
  {
    ignerr << "MyApplyJointForce found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  sdfElem = sdfClone->GetElement("link_name");
  if (sdfElem)
  {
    this->dataPtr->linkName = sdfElem->Get<std::string>();
  }

  if (this->dataPtr->linkName == "")
  {
    ignerr << "MyApplyJointForce found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  // Subscribe to commands
  // auto topic = transport::TopicUtils::AsValidTopic("/model/" +
  //     this->dataPtr->model.Name(_ecm) + "/joint/" + this->dataPtr->jointName +
  //     "/cmd_force");
  auto topic = transport::TopicUtils::AsValidTopic("/torque_cmd");
  if (topic.empty())
  {
    ignerr << "Failed to create valid topic for [" << this->dataPtr->jointName
           << "]" << std::endl;
    return;
  }
  this->dataPtr->node.Subscribe(topic, &MyApplyJointForcePrivate::OnCmdForce,
                                this->dataPtr.get());

  ignmsg << "MyApplyJointForce subscribing to Double messages on [" << topic
         << "]" << std::endl;


  // Advertise torque topics
  this->dataPtr->preTorquePublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/initial_torque");

  this->dataPtr->preTorqueCmdPublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/initial_torque_cmd");

  this->dataPtr->torquePublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/update_torque");

  this->dataPtr->torqueCmdPublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/update_torque_cmd");

  this->dataPtr->postTorquePublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/final_torque");

  this->dataPtr->postTorqueCmdPublisher = this->dataPtr->node.Advertise<msgs::Double>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/final_torque_cmd");

  this->dataPtr->angularAccelPublisher = this->dataPtr->node.Advertise<msgs::Vector3d>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/angular_accel");

  this->dataPtr->postAngularAccelPublisher = this->dataPtr->node.Advertise<msgs::Vector3d>(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/" + this->dataPtr->jointName + "/post_acceleration");
}

//////////////////////////////////////////////////
void MyApplyJointForce::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("MyApplyJointForce::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // If the link hasn't been identified yet, look for it
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->linkEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // make sure component JointForce is created
  auto initialTorque = _ecm.Component<components::JointForce>(
      this->dataPtr->jointEntity);
  if (initialTorque == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        components::JointForce({0}));
  }
  else
  {
    // Publish torque before adding command torque
    msgs::Double jointTorque;
    jointTorque.set_data(initialTorque->Data().at(0));
    jointTorque.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->preTorquePublisher.Publish(jointTorque);
  }



  // Update joint force command
  auto force = _ecm.Component<components::JointForceCmd>(
      this->dataPtr->jointEntity);

  std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

  if (force == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        components::JointForceCmd({this->dataPtr->jointForceCmd}));
  }
  else
  {
    // Add command torque
    force->Data()[0] += this->dataPtr->jointForceCmd;

    // Publish torque after adding command torque
    msgs::Double jointTorque;
    jointTorque.set_data(force->Data().at(0));
    jointTorque.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->preTorqueCmdPublisher.Publish(jointTorque);
  }

  auto angularAccel = _ecm.Component<components::AngularAcceleration>(
      this->dataPtr->linkEntity);
  if (angularAccel == nullptr)
  {
    _ecm.CreateComponent(
        this->dataPtr->linkEntity,
        components::AngularAcceleration({0, 0, 0}));
  }
  else
  {
    // Publish angular acceleration
    msgs::Vector3d linkAngularAccel;
    linkAngularAccel.set_x(angularAccel->Data().X());
    linkAngularAccel.set_y(angularAccel->Data().Y());
    linkAngularAccel.set_z(angularAccel->Data().Z());
    linkAngularAccel.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->angularAccelPublisher.Publish(linkAngularAccel);
  }
}



//////////////////////////////////////////////////
void MyApplyJointForce::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("MyApplyJointForce::Update");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  // make sure component JointForce is created
  auto initialTorque = _ecm.Component<components::JointForce>(
      this->dataPtr->jointEntity);
  if (initialTorque != nullptr)
  {
    // Publish torque before adding command torque
    msgs::Double jointTorque;
    jointTorque.set_data(initialTorque->Data().at(0));
    jointTorque.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->torquePublisher.Publish(jointTorque);
  }



  // Update joint force command
  auto force = _ecm.Component<components::JointForceCmd>(
      this->dataPtr->jointEntity);

  std::lock_guard<std::mutex> lock(this->dataPtr->jointForceCmdMutex);

  if (force != nullptr)
  {
    // Publish torque after adding command torque
    msgs::Double jointTorque;
    jointTorque.set_data(force->Data().at(0));
    jointTorque.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->torqueCmdPublisher.Publish(jointTorque);
  }
}



//////////////////////////////////////////////////
void MyApplyJointForce::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  IGN_PROFILE("MyApplyJointForce::PostUpdate");

  // if the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
  }

  if (this->dataPtr->jointEntity == kNullEntity)
    return;

  // if the link hasn't been identified yet, look for it
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->linkEntity == kNullEntity)
    return;

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  auto initialTorque = _ecm.Component<components::JointForce>(
      this->dataPtr->jointEntity);
  if (initialTorque != nullptr)
  {
    // Publish torque before adding command torque
    msgs::Double jointTorque;
    jointTorque.set_data(initialTorque->Data().at(0));
    jointTorque.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->postTorquePublisher.Publish(jointTorque);
    ignmsg << "preTorque published " << initialTorque->Data().at(0) << std::endl;
  }
  else
  {
    ignerr << "Failed to get JointForce component in postUpdate" << std::endl;
  }

  auto force = _ecm.Component<components::JointForceCmd>(
      this->dataPtr->jointEntity);
  if (force != nullptr)
  {
    // Publish torque after adding command torque
    msgs::Double jointTorque;
    jointTorque.set_data(force->Data().at(0));
    jointTorque.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->postTorqueCmdPublisher.Publish(jointTorque);
    ignmsg << "postTorque published " << force->Data().at(0) << std::endl;
  }
  else
  {
    ignerr << "Failed to get JointForceCmd component in postUpdate" << std::endl;
  }

  auto angularAccel = _ecm.Component<components::AngularAcceleration>(
      this->dataPtr->linkEntity);
  if (angularAccel != nullptr)
  {
    // Publish angular acceleration
    msgs::Vector3d linkAngularAccel;
    linkAngularAccel.set_x(angularAccel->Data().X());
    linkAngularAccel.set_y(angularAccel->Data().Y());
    linkAngularAccel.set_z(angularAccel->Data().Z());
    linkAngularAccel.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));
    this->dataPtr->postAngularAccelPublisher.Publish(linkAngularAccel);
  }
      
}






//////////////////////////////////////////////////
void MyApplyJointForcePrivate::OnCmdForce(const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->jointForceCmdMutex);
  this->jointForceCmd = _msg.data();
}

IGNITION_ADD_PLUGIN(MyApplyJointForce,
                    System,
                    MyApplyJointForce::ISystemConfigure,
                    MyApplyJointForce::ISystemPreUpdate,
                    MyApplyJointForce::ISystemUpdate,
                    MyApplyJointForce::ISystemPostUpdate)


// TODO(CH3): Deprecated, remove on version 8
IGNITION_ADD_PLUGIN_ALIAS(MyApplyJointForce,
                          "ignition::gazebo::systems::MyApplyJointForce")
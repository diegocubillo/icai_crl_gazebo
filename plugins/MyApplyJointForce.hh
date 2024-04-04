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

#include <ignition/gazebo/System.hh>
#include "ignition/gazebo/Model.hh"
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
  // Forward declaration
  class MyApplyJointForcePrivate
{
  /// \brief Callback for joint force subscription
  /// \param[in] _msg Joint force message
  public: void OnCmdForce(const msgs::Double &_msg);

  /// \brief Torque before adding command torque
  public: transport::Node::Publisher preTorquePublisher;

  /// \brief Torque command before adding command torque
  public: transport::Node::Publisher preTorqueCmdPublisher;

  /// \brief Torque during Update
  public: transport::Node::Publisher torquePublisher;

  /// \brief Torque command during Update
  public: transport::Node::Publisher torqueCmdPublisher;

  /// \brief Torque after adding command torque
  public: transport::Node::Publisher postTorquePublisher;

  /// \brief Torque command after adding command torque
  public: transport::Node::Publisher postTorqueCmdPublisher;

  /// \brief Angular acceleration
  public: transport::Node::Publisher angularAccelPublisher;

  /// \brief Angular acceleration after adding command torque
  public: transport::Node::Publisher postAngularAccelPublisher;

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint force
  public: double jointForceCmd;

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Model interface
  public: Model model{kNullEntity};
};

  /// \brief This system applies a force to the first axis of a specified joint.
  class MyApplyJointForce
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: MyApplyJointForce();

    /// \brief Destructor
    public: ~MyApplyJointForce() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    public: void Update(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void PostUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<MyApplyJointForcePrivate> dataPtr;
  };
  }
}
}
}

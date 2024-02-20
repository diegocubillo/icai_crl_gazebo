#include <ignition/gazebo/System.hh>
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
  private: bool ValidateParameters();
  private: std::unique_ptr<md25_pluginPrivate> dataPtr;
};

}
}
}
}
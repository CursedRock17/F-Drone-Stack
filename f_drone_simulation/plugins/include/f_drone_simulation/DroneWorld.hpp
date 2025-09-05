#ifndef F_DRONE_SIMULATION__DRONE_WORLD_HPP_
#define F_DRONE_SIMULATION__DRONE_WORLD_HPP_

// C/C++ STD Libraries 
#include <memory>

// Core Plugin Headers
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

// Req/Res Headers 
#include <gz/transport/Node.hh>

// Gazebo Message Headers
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/pose.pb.h>

// Component Plugin Headers
#include <gz/sim/components/Pose.hh>

// Helper (math) Headers
#include <gz/math/Pose3.hh>

namespace f_drone_simulation
{
  class DroneWorldPrivate;
  // This is the main plugin's class, inheriting from the Gazebo System which 
  // correctly updates the state of our world during simulation.
  class DroneWorld:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {
  public: 
    DroneWorld();
    ~DroneWorld() override = default;

    // Implements the Configure() callback, which is called when a system is 
    // initially loaded. 
    // _entity:       Contains the entity that the system is attached to
    // _element:      Contains the sdf Element with custom configuration
    //  ecm:          Provides an interface to all entities and components
    // _eventManager: Provides a mechanism for registering internal signals
    void Configure(
                const gz::sim::Entity & entity,
                const std::shared_ptr<const sdf::Element> & element,
                gz::sim::EntityComponentManager & ecm,
                gz::sim::EventManager &_eventManager) override;

    void PostUpdate(const gz::sim::UpdateInfo & info,
                const gz::sim::EntityComponentManager & ecm) override;

    void Reset(const gz::sim::UpdateInfo & info,
                gz::sim::EntityComponentManager & ecm) override;
  
  private:
    std::unique_ptr<DroneWorldPrivate> dataPtr;

    void spawnEntity();
  };
}  // Namespace: f_drone_simulation

#endif  // F_DRONE_SIMULATION__DRONE_WORLD_HPP_

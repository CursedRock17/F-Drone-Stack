// All necessary system headers
#include <gz/common/Console.hh>

// This header is required to register plugins
#include <gz/plugin/Register.hh>

// Internal Headers
#include "f_drone_simulation/DroneWorld.hpp"

class f_drone_simulation::DroneWorldPrivate
{
public:
  // Information about our world
  std::string worldName = "drone_world";

  // Transport Nodes
  gz::transport::Node modelNode;
  
  // Information about our model
  std::string modelPath = "models/model.sdf";
  std::string modelName = "model";
};

namespace f_drone_simulation {
// Default Constructor
DroneWorld::DroneWorld()
  : dataPtr(std::make_unique<DroneWorldPrivate>())
{
}

void DroneWorld::Configure(const gz::sim::Entity & entity,
                const std::shared_ptr<const sdf::Element> & element,
                gz::sim::EntityComponentManager & ecm,
                gz::sim::EventManager & eventManager)
{
  std::cout << "f_drone_simulation::DroneWorld::Configure Called" << std::endl;
  
  // Grab the model which we want spawned along with its name
  if (element->HasElement("model_path"))
    this->dataPtr->modelPath = element->Get<std::string>("model_path");
  if (element->HasElement("model_name"))
    this->dataPtr->modelName = element->Get<std::string>("model_name");

  // Spawn our drone on start up
  spawnEntity();
}

void DroneWorld::PostUpdate(const gz::sim::UpdateInfo & info,
                            const gz::sim::EntityComponentManager & ecm) 
{
}

void DroneWorld::Reset(const gz::sim::UpdateInfo & info,
                            gz::sim::EntityComponentManager & ecm) 
{
  std::cout << "f_drone_simulation::DroneWorld::Reset Called" << std::endl;

  // Respawn our Drone
  spawnEntity();
}
    
void DroneWorld::spawnEntity()
{
  // Prepare the input parameters.
  gz::msgs::EntityFactory req;
  req.set_sdf_filename(this->dataPtr->modelPath);
  req.set_name(this->dataPtr->modelName);
 
  // Ensure we get some sort of T/F response from the service
  gz::msgs::Boolean res;
  bool spawnStatus;
  unsigned int timeout = 1000;

  std::string modelCreateTopic = "/world/" + this->dataPtr->worldName + "/create";
  // Request the "/world/name/create" service.
  bool executed = this->dataPtr->modelNode.Request(
    modelCreateTopic, req, timeout, res, spawnStatus);
  if (executed)
  {
    if (spawnStatus) {
      std::cout << "Response: [" << res.data() << "]" << std::endl;
    } else {
      std::cerr << "Failed to Spawn Model: " << this->dataPtr->modelName << std::endl;
    }
  } else {
    std::cerr << "Service call timed out" << std::endl;
  }
}

}  // namespace: f_drone_simulation

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    f_drone_simulation::DroneWorld,
    f_drone_simulation::DroneWorld::System,
    f_drone_simulation::DroneWorld::ISystemConfigure,
    f_drone_simulation::DroneWorld::ISystemPostUpdate,
    f_drone_simulation::DroneWorld::ISystemReset
);

GZ_ADD_PLUGIN_ALIAS(f_drone_simulation::DroneWorld, "f_drone_simulation::DroneWorld")

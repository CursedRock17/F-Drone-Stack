// All necessary system headers
#include <gz/common/Console.hh>

// This header is required to register plugins
#include <gz/plugin/Register.hh>

// Internal Headers
#include "f_drone_gazebo/DroneWorld.hpp"

class f_drone_gazebo::DroneWorldPrivate
{
public:
  // Information about our world
  std::string worldName = "drone_world";

  // Transport Nodes
  gz::transport::Node modelNode;
  
  // Information about our model
  gz::sim::Entity droneEntity;
  std::string modelPath = "models/model.sdf";
  std::string modelName = "model";
  bool needsRespawn = false;
};

namespace f_drone_gazebo {
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
  std::cout << "f_drone_gazebo::DroneWorld::Configure Called" << std::endl;
  
  // Grab the model which we want spawned along with its name
  if (element->HasElement("model_path"))
    this->dataPtr->modelPath = element->Get<std::string>("model_path");
  if (element->HasElement("model_name"))
    this->dataPtr->modelName = element->Get<std::string>("model_name");
  if (element->HasElement("world_name"))
    this->dataPtr->worldName = element->Get<std::string>("world_name");

  /*
  // Establish a running entity
  this->dataPtr->droneEntity = ecm.EntityByName(this->dataPtr->modelName).value();

  */
  // Spawn our drone on start up
  spawnEntity();
}

void DroneWorld::PreUpdate(const gz::sim::UpdateInfo & info,
                           gz::sim::EntityComponentManager & ecm) 
{
  // Function called before the physics are applied in sim, so this is when actions need to be taken.
  if (this->dataPtr->needsRespawn == true) {
    spawnEntity();
    this->dataPtr->needsRespawn = false;
  }
}

void DroneWorld::PostUpdate(const gz::sim::UpdateInfo & info,
                            const gz::sim::EntityComponentManager & ecm) 
{
  // Function called after the physics are applied in sim. Place where you read back sensor data.
}

void DroneWorld::Reset(const gz::sim::UpdateInfo & info,
                            gz::sim::EntityComponentManager & ecm) 
{
  std::cout << "f_drone_gazebo::DroneWorld::Reset Called" << std::endl;

  // Respawn our Drone
  deleteEntity();
  this->dataPtr->needsRespawn = true;
}
    
void DroneWorld::spawnEntity()
{
  // Prepare the input parameters.
  gz::msgs::EntityFactory reqEntityFac;
  reqEntityFac.set_sdf_filename(this->dataPtr->modelPath);
  reqEntityFac.set_name(this->dataPtr->modelName);
  reqEntityFac.set_allow_renaming(false);
  
  // Get the coordinates of where we want our object to spawn
  auto entityPose = reqEntityFac.mutable_pose();
  entityPose->mutable_position()->set_x(0.0);
  entityPose->mutable_position()->set_y(0.0);
  entityPose->mutable_position()->set_z(0.05);
  entityPose->mutable_orientation()->set_x(0.0);
  entityPose->mutable_orientation()->set_y(0.0);
  entityPose->mutable_orientation()->set_z(-0.7071);
  entityPose->mutable_orientation()->set_w(0.7071);
 
  // Ensure we get some sort of T/F response from the service
  gz::msgs::Boolean res;
  bool spawnStatus;
  unsigned int timeout = 1000;

  std::string modelCreateTopic = "/world/" + this->dataPtr->worldName + "/create";
  // Request the "/world/name/create" service.
  bool executed = this->dataPtr->modelNode.Request(
    modelCreateTopic, reqEntityFac, timeout, res, spawnStatus);
  if (executed)
  {
    if (spawnStatus) {
      std::cout << "Spawn Response: [" << res.data() << "]" << std::endl;
    } else {
      std::cerr << "Failed to Spawn Model: " << this->dataPtr->modelName << std::endl;
    }
  } else {
    std::cerr << "Service call timed out" << std::endl;
  }
}

void DroneWorld::deleteEntity()
{
  // Prepare the input parameters.
  gz::msgs::Entity reqEntity;
  reqEntity.set_name(this->dataPtr->modelName);
  reqEntity.set_id(this->dataPtr->droneEntity);
 
  // Ensure we get some sort of T/F response from the service
  gz::msgs::Boolean res;
  bool removedStatus;
  unsigned int timeout = 1000;

  std::string modelCreateTopic = "/world/" + this->dataPtr->worldName + "/remove";
  // Request the "/world/name/create" service.
  bool executed = this->dataPtr->modelNode.Request(
    modelCreateTopic, reqEntity, timeout, res, removedStatus);
  if (executed)
  {
    if (removedStatus) {
      std::cout << "Delete Response: [" << res.data() << "]" << std::endl;
    } else {
      std::cerr << "Failed to Delete Model: " << this->dataPtr->modelName << std::endl;
    }
  } else {
    std::cerr << "Service call timed out" << std::endl;
  }
}

}  // namespace: f_drone_gazebo

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    f_drone_gazebo::DroneWorld,
    f_drone_gazebo::DroneWorld::System,
    f_drone_gazebo::DroneWorld::ISystemConfigure,
    f_drone_gazebo::DroneWorld::ISystemPreUpdate,
    f_drone_gazebo::DroneWorld::ISystemPostUpdate,
    f_drone_gazebo::DroneWorld::ISystemReset
);

GZ_ADD_PLUGIN_ALIAS(f_drone_gazebo::DroneWorld, "f_drone_gazebo::DroneWorld")

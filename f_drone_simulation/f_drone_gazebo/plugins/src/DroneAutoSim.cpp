// All necessary system headers
#include <gz/common/Console.hh>

// This header is required to register plugins
#include <gz/plugin/Register.hh>

// Internal Headers
#include "f_drone_gazebo/DroneAutoSim.hpp"


class f_drone_gazebo::DroneAutoSimPrivate
{
public:
  // Entity representing the sensors in the world
  gz::sim::Entity imuLink{gz::sim::kNullEntity};
  gz::sim::Entity rangefinderLink{gz::sim::kNullEntity};

  // Transport Nodes
  gz::transport::Node imuNode;
  std::string imuTopicName;
  void imuCallback (const gz::msgs::IMU & imu_msg);
  
  gz::transport::Node rangefinderNode;
  std::string rangefinderTopicName;
  void rangefinderCallback (const gz::msgs::LaserScan & rangefinder_msg);
  
  // Motor Publishing Node
  gz::transport::Node actuatorNode;
  gz::transport::Node::Publisher actuatorPub;
  std::string actuatorTopicName;

  // Drone Model Information
  gz::sim::Model droneModel {gz::sim::kNullEntity};

  // Drone receives:
  SensorPacket sensorAutoSim;
  // Drone sends: 
  RotorPacket rotorPacket;
};

namespace f_drone_gazebo {
// Default Constructor
DroneAutoSim::DroneAutoSim()
  : dataPtr(std::make_unique<DroneAutoSimPrivate>())
{
}

void DroneAutoSim::Configure(const gz::sim::Entity & entity,
                const std::shared_ptr<const sdf::Element> & element,
                gz::sim::EntityComponentManager & ecm,
                gz::sim::EventManager & eventManager)
{
  std::cout << "f_drone_gazebo::DroneAutoSim::Configure Called" << std::endl;

  // Initialize our model
  this->dataPtr->droneModel = gz::sim::Model(entity);
  if (!this->dataPtr->droneModel.Valid(ecm))
  {
    std::cerr << "DroneAutoSim plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Need To Find the IMU in the Model - locate IMU link by name if provided
  std::string imuLinkName = "drone_frame"; // replace with your IMU link
  if (element->HasElement("imu_link")) {
    imuLinkName = element->Get<std::string>("imu_link");
  }
  this->dataPtr->imuLink = ecm.EntityByComponents(
    gz::sim::components::Name(imuLinkName), gz::sim::components::ParentEntity(entity));

  if (this->dataPtr->imuLink == gz::sim::kNullEntity) {
    std::cerr << "could not find imu link [" << imuLinkName << "]" << std::endl;
  }

  // Now we can find the Rangefinder (gpu_lidar)
  std::string rangefinderLinkName = "drone_frame"; // replace with your IMU link
  if (element->HasElement("rangefinder_link"))
    rangefinderLinkName = element->Get<std::string>("rangefinder_link");

  this->dataPtr->rangefinderLink = ecm.EntityByComponents(
    gz::sim::components::Name(rangefinderLinkName),
    gz::sim::components::ParentEntity(entity));

  if (this->dataPtr->rangefinderLink == gz::sim::kNullEntity) {
    std::cerr << "Could not find Rangefinder link [" << rangefinderLinkName << "]" << std::endl;
  }

  // IMU Node Setup
  this->dataPtr->imuTopicName = "/imu";
  auto imuTopic = gz::transport::TopicUtils::AsValidTopic(
    this->dataPtr->imuTopicName.c_str());
  if (imuTopic.empty())
  {
    std::cerr << "Failed to create valid topic for IMU" << std::endl;
    return;
  }
  // Create our subscription to the topic
  this->dataPtr->imuNode.Subscribe(imuTopic, &DroneAutoSimPrivate::imuCallback,
                                this->dataPtr.get());

  std::cout << "DroneAutoSim subscribing to IMU messages on [" << imuTopic
         << "]" << std::endl;
  
  // Rangefinder Node Setup
  this->dataPtr->rangefinderTopicName = "/radar";
  auto rangefinderTopic = gz::transport::TopicUtils::AsValidTopic(
    this->dataPtr->rangefinderTopicName.c_str());
  if (rangefinderTopic.empty())
  {
    std::cerr << "Failed to create valid topic for Rangefinder" << std::endl;
    return;
  }
  // Create our subscription to the topic
  this->dataPtr->rangefinderNode.Subscribe(rangefinderTopic, 
                                           &DroneAutoSimPrivate::rangefinderCallback,
                                           this->dataPtr.get());

  std::cout << "DroneAutoSim subscribing to LaserScan messages on [" << rangefinderTopic
         << "]" << std::endl;
  
  // Actuator Node Setup
  this->dataPtr->actuatorTopicName = "/drone/gazebo/command/motor_speed";
  this->dataPtr->actuatorPub = this->dataPtr->actuatorNode.Advertise<
    gz::msgs::Actuators>(this->dataPtr->actuatorTopicName);
  if (!this->dataPtr->actuatorPub) {
    std::cout << "Unable to Advertise topic: " << this->dataPtr->actuatorTopicName << std::endl;
    return;
  }
}

void DroneAutoSim::PreUpdate(const gz::sim::UpdateInfo & info,
                            gz::sim::EntityComponentManager & ecm) 
{
  // 1. Read IMU + rangefinder from Gazebo
  if (this->dataPtr->imuLink == gz::sim::kNullEntity || 
      this->dataPtr->rangefinderLink == gz::sim::kNullEntity) {
    std::cout << "Could not find imu/rangefinder link "  << std::endl;
    return;
  }

  // Do calculations of rotor velocities
  for (int i = 0; i < 4; i++) 
  {
    this->dataPtr->rotorPacket.pwm[i] = 100;
  }

  // 2. Read actuator commands from Teensy
  // Apply motor forces here: convert rotorPacket.pwm[i] → thrust
  // and set force/torque on each rotor joint
  gz::msgs::Actuators actuatorMsg;
  const float throttleConst = 1.0;
  for (int i = 0; i < 4; i++) {
    actuatorMsg.add_velocity(throttleConst * this->dataPtr->rotorPacket.pwm[i]);
  }

  // Publish our message to our drone topic 
  this->dataPtr->actuatorPub.Publish(actuatorMsg);
}

void DroneAutoSim::PostUpdate(const gz::sim::UpdateInfo & info,
                             const gz::sim::EntityComponentManager & ecm) 
{
}

void DroneAutoSim::Reset(const gz::sim::UpdateInfo & info,
                            gz::sim::EntityComponentManager & ecm) 
{
}
    
}  // namespace: f_drone_gazebo

void f_drone_gazebo::DroneAutoSimPrivate::imuCallback (const gz::msgs::IMU & imu_msg)
{
  // Move the current accelerometer and gyrometer to the packet to publish
  this->sensorAutoSim.ax = imu_msg.linear_acceleration().x();
  this->sensorAutoSim.ay = imu_msg.linear_acceleration().y();
  this->sensorAutoSim.az = imu_msg.linear_acceleration().z();

  this->sensorAutoSim.gx = imu_msg.angular_velocity().x();
  this->sensorAutoSim.gy = imu_msg.angular_velocity().y();
  this->sensorAutoSim.gz = imu_msg.angular_velocity().z();
}
  
void f_drone_gazebo::DroneAutoSimPrivate::rangefinderCallback (
  const gz::msgs::LaserScan & rangefinder_msg)
{
  // Check the first ray to be sent out and make that our range
  if (std::isfinite(rangefinder_msg.ranges(0))) {
    this->sensorAutoSim.range = rangefinder_msg.ranges(0);
  }
}

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    f_drone_gazebo::DroneAutoSim,
    f_drone_gazebo::DroneAutoSim::System,
    f_drone_gazebo::DroneAutoSim::ISystemConfigure,
    f_drone_gazebo::DroneAutoSim::ISystemPreUpdate,
    f_drone_gazebo::DroneAutoSim::ISystemPostUpdate,
    f_drone_gazebo::DroneAutoSim::ISystemReset
);

GZ_ADD_PLUGIN_ALIAS(f_drone_gazebo::DroneAutoSim, "f_drone_gazebo::DroneAutoSim")

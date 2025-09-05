#ifndef F_DRONE_SIMULATION__DRONE_SERIAL_HPP_
#define F_DRONE_SIMULATION__DRONE_SERIAL_HPP_

// C/C++ STD Libraries 
#include <memory>

// Core Plugin Headers
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

// Pub/Sub Headers 
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

// Gazebo Message Headers
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>

// Component Plugin Headers
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LaserRetro.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

// Helper (math) Headers
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>

namespace f_drone_simulation
{
  struct SensorPacket {
    float ax, ay, az;    // accelerometer
    float gx, gy, gz;    // gyro
    float range;         // altitude (m)
  };

  // Need a union to ensure float values and byte stream don't mix values
  union SensorSerial {
    SensorPacket simulatedPkt;
    unsigned char serialStream[sizeof(SensorPacket)];
  };
  
  // Add a header and footer to keep Teensy in-sync with Gazebo
  const unsigned char pktHeader[2] = {0xAA, 0x55};
  const unsigned char pktFooter = 0xFE;

  struct RotorPacket {
  // Mapped to OneShot 125 (125 -> 250)
    int pwm[4];     
  };

  class DroneSerialPrivate;
  // This is the main plugin's class, inheriting from the Gazebo System which 
  // correctly updates the state of our model during simulation.
  class DroneSerial:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemReset
  {
  public: 
    DroneSerial();
    ~DroneSerial() override = default;

    // Implements the Configure() callback, which is called when a system is 
    // initially loaded. 
    // entity:       Contains the entity that the system is attached to
    // element:      Contains the sdf Element with custom configuration
    // ecm:          Provides an interface to all entities and components
    // eventManager: Provides a mechanism for registering internal signals
    void Configure(
                const gz::sim::Entity & entity,
                const std::shared_ptr<const sdf::Element> & element,
                gz::sim::EntityComponentManager & ecm,
                gz::sim::EventManager & eventManager) override;

    void PostUpdate(const gz::sim::UpdateInfo & info,
                const gz::sim::EntityComponentManager & ecm) override;

    void Reset(const gz::sim::UpdateInfo & info,
                gz::sim::EntityComponentManager & ecm) override;
  
  private:
    std::unique_ptr<DroneSerialPrivate> dataPtr;

    void initializeSerialPort(std::string portName);
  };
}  // Namespace: f_drone_simulation

#endif  // F_DRONE_SIMULATION__DRONE_SERIAL_HPP_

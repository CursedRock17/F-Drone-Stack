// All necessary system headers
#include <gz/common/Console.hh>

// This header is required to register plugins
#include <gz/plugin/Register.hh>

// Internal Headers
#include "f_drone_simulation/DroneSerial.hpp"

// Linux Serial Port Headers
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>

class f_drone_simulation::DroneSerialPrivate
{
public:
  // Serial Port Info
  int serial_port;
  
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

  // Teensy receives:
  SensorSerial sensorSerial;
  // Teensy sends:
  RotorPacket rotorPacket;
};

namespace f_drone_simulation {
// Default Constructor
DroneSerial::DroneSerial()
  : dataPtr(std::make_unique<DroneSerialPrivate>())
{
}

void DroneSerial::Configure(const gz::sim::Entity & entity,
                const std::shared_ptr<const sdf::Element> & element,
                gz::sim::EntityComponentManager & ecm,
                gz::sim::EventManager & eventManager)
{
  std::cout << "f_drone_simulation::DroneSerial::Configure Called" << std::endl;
  
  // Find the attached Serial Port tag
  std::string port = "/dev/ttyACM0";
  if (element->HasElement("port"))
    port = element->Get<std::string>("port");

  initializeSerialPort(port);

  // Initialize our model
  this->dataPtr->droneModel = gz::sim::Model(entity);
  if (!this->dataPtr->droneModel.Valid(ecm))
  {
    std::cerr << "DroneSerial plugin should be attached to a model entity. "
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
  this->dataPtr->imuNode.Subscribe(imuTopic, &DroneSerialPrivate::imuCallback,
                                this->dataPtr.get());

  std::cout << "DroneSerial subscribing to IMU messages on [" << imuTopic
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
                                           &DroneSerialPrivate::rangefinderCallback,
                                           this->dataPtr.get());

  std::cout << "DroneSerial subscribing to LaserScan messages on [" << rangefinderTopic
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

void DroneSerial::PostUpdate(const gz::sim::UpdateInfo & info,
                            const gz::sim::EntityComponentManager & ecm) 
{
  // 1. Read IMU + rangefinder from Gazebo
  if (this->dataPtr->serial_port < 0 || 
      this->dataPtr->imuLink == gz::sim::kNullEntity || 
      this->dataPtr->rangefinderLink == gz::sim::kNullEntity) {
    std::cout << "Could not find imu/rangefinder link "  << std::endl;
    return;
  }

  // Convert our SensorPacket struct to a stream of raw bytes so Serial Port can read
  static long int lastSend = 0;
  //if (info.simTime.count() - lastSend > 10) { // 100 Hz
    lastSend = info.simTime.count();
    write(this->dataPtr->serial_port, &pktHeader, 2);
    for (size_t i = 0; i < sizeof(this->dataPtr->sensorSerial.serialStream) / sizeof(unsigned char); i++) {
      write(this->dataPtr->serial_port, &this->dataPtr->sensorSerial.serialStream[i], 1);
    }
    write(this->dataPtr->serial_port, &pktFooter, 1);
  //}

  // 2. Read actuator commands from Teensy
  if (read(this->dataPtr->serial_port, &this->dataPtr->rotorPacket, 
           sizeof(this->dataPtr->rotorPacket)) == sizeof(this->dataPtr->rotorPacket))
  {
    // Apply motor forces here: convert rotorPacket.pwm[i] → thrust
    // and set force/torque on each rotor joint
    gz::msgs::Actuators actuatorMsg;
    for (int i = 0; i < 4; i++) {
      actuatorMsg.add_velocity(this->dataPtr->rotorPacket.pwm[i]);
    }

    // Publish our message to our drone topic 
    this->dataPtr->actuatorPub.Publish(actuatorMsg);
  }
}

void DroneSerial::Reset(const gz::sim::UpdateInfo & info,
                            gz::sim::EntityComponentManager & ecm) 
{
}
    
void DroneSerial::initializeSerialPort(std::string portName)
{
  // Open serial port
  this->dataPtr->serial_port = open(portName.c_str(), O_RDWR);
  // Check for errors
  if (this->dataPtr->serial_port < 0) {
    std::cout << "Error " << errno << " from open: " << strerror(errno) << std::endl;
  }

  // Create the serial connection
  struct termios tty {};
  if (tcgetattr(this->dataPtr->serial_port, &tty) != 0) {
    std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    return;
  }

  // Control Flags - Standard Configuration
  // Parity, Stop, Clear Size & # of Bits/Byte = 8, Allow Flow
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // Local Flags - Process the way data is received ensuring no duplicates
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;
  
  // Input Flags 
  // Disable Software Flow Control and Manipulation of Data
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  // Output Flags
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  // Setup Timing
  tty.c_cc[VTIME] = 5;  // Wait for up to 0.5s (5 deciseconds)
  tty.c_cc[VMIN]  = 1;  // Reading doesn't block

  // Setup Baud Rate - 115200
  auto baudRate = B115200;
  cfsetispeed(&tty, baudRate);
  cfsetospeed(&tty, baudRate);
  cfmakeraw(&tty);
  
  // Acquire non-blocking exclusive lock for our serial port
  if (flock(this->dataPtr->serial_port, LOCK_EX | LOCK_NB) == -1) {
      throw std::runtime_error("Serial port with file descriptor " +
          std::to_string(this->dataPtr->serial_port) + 
                               " is already locked by another process.");
  }

  // Save our settings
  if (tcsetattr(this->dataPtr->serial_port, TCSANOW, &tty) != 0) {
    std::cout << "Error " << errno << " from tcsetattr: " << strerror(errno) << std::endl;
  }
}

}  // namespace: f_drone_simulation

void f_drone_simulation::DroneSerialPrivate::imuCallback (const gz::msgs::IMU & imu_msg)
{
  // Move the current accelerometer and gyrometer to the packet to publish
  this->sensorSerial.simulatedPkt.ax = imu_msg.linear_acceleration().x();
  this->sensorSerial.simulatedPkt.ay = imu_msg.linear_acceleration().y();
  this->sensorSerial.simulatedPkt.az = imu_msg.linear_acceleration().z();

  this->sensorSerial.simulatedPkt.gx = imu_msg.angular_velocity().x();
  this->sensorSerial.simulatedPkt.gy = imu_msg.angular_velocity().y();
  this->sensorSerial.simulatedPkt.gz = imu_msg.angular_velocity().z();
}
  
void f_drone_simulation::DroneSerialPrivate::rangefinderCallback (
  const gz::msgs::LaserScan & rangefinder_msg)
{
  // Check the first ray to be sent out and make that our range
  if (std::isfinite(rangefinder_msg.ranges(0))) {
    this->sensorSerial.simulatedPkt.range = rangefinder_msg.ranges(0);
  }
}

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    f_drone_simulation::DroneSerial,
    f_drone_simulation::DroneSerial::System,
    f_drone_simulation::DroneSerial::ISystemConfigure,
    f_drone_simulation::DroneSerial::ISystemPostUpdate,
    f_drone_simulation::DroneSerial::ISystemReset
);

GZ_ADD_PLUGIN_ALIAS(f_drone_simulation::DroneSerial, "f_drone_simulation::DroneSerial")

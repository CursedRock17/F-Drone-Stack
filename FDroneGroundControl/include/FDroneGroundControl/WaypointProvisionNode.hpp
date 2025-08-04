// ROS 2 Based Headers
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// C/C++ Standard Headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// All Messages
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/msg/waypoint_list.hpp"

// Default Certain Types and Usages 
using std::placeholders::_1;
using namespace std::chrono_literals;

class WaypointProvisionNode : public rclcpp::Node
{
public: 
WaypointProvisionNode();
~WaypointProvisionNode();

private:
// Provide a list of waypoints could be [1 - MaxPoints]
rclcpp::Publisher<mavros_msgs::msg::WaypointList>::SharedPtr waypointPub;

// All Callback Functions
};

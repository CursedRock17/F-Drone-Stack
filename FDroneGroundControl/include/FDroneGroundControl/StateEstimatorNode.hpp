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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Default Certain Types and Usages 
using std::placeholders::_1;
using namespace std::chrono_literals;

class StateEstimatorNode : public rclcpp::Node
{
public: 
StateEstimatorNode();
~StateEstimatorNode();

private:
// Grab information from the drone
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr poseSub;
rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twistSub;
std::string droneName_;

// Update it's current state
std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

// All Callback Functions
void PoseCallback(const geometry_msgs::msg::PoseStamped & pose);
void TwistCallback(const geometry_msgs::msg::TwistStamped & twist);
};

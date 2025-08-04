#include "include/WaypointProvisionNode.hpp"

WaypointProvisionNode::WaypointProvisionNode() : rclcpp::Node("waypoint_provision_node")
{
  // Reference Frame Quality of Service to help us patch our topic info 
  rclcpp::QoS waypointQoS = rclcpp::Qos(10);

  // Create our Publisher
  waypointPub = this->create_publisher<mavros_msgs::msg::WaypointList>(
    "/drone/waypoints", waypointQos);
}

WaypointProvisionNode::~WaypointProvisionNode()
{
}


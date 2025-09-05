#include "FDroneGroundControl/StateEstimatorNode.hpp"

StateEstimatorNode::StateEstimatorNode() : rclcpp::Node("state_estimator_node")
{
  // Reference Frame Quality of Service to help us patch our topic info 
  rclcpp::QoS frameQoS = rclcpp::QoS(10);
  frameQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);
  // Create our Subscriptions
  odomSub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/model/drone/odom", frameQoS, std::bind(&StateEstimatorNode::OdomCallback, this, _1));

  // Acquire the name of a certain drone - defaulted to "f_drone"
  droneName_ = this->declare_parameter<std::string>("drone_name", "drone");

  // Initialize the Transform Broadcaster (Essentially just a publisher), based on the node
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

StateEstimatorNode::~StateEstimatorNode()
{
}

void StateEstimatorNode::OdomCallback(std::shared_ptr<nav_msgs::msg::Odometry> odom)
{
  geometry_msgs::msg::TransformStamped transformMsg;

  // Setup the header of our transform 
  transformMsg.header.stamp = this->get_clock()->now();
  transformMsg.header.frame_id = "world";
  transformMsg.child_frame_id = "drone/drone_frame"; // Really droneName_.c_str();

  // Get the 3D coordinates of the current drone position 
  transformMsg.transform.translation.x = odom->pose.pose.position.x;
  transformMsg.transform.translation.y = odom->pose.pose.position.y;
  transformMsg.transform.translation.z = odom->pose.pose.position.z;

  // Get the overall rotation of our current drone
  transformMsg.transform.rotation.x = odom->pose.pose.orientation.x;
  transformMsg.transform.rotation.y = odom->pose.pose.orientation.y;
  transformMsg.transform.rotation.z = odom->pose.pose.orientation.z;
  transformMsg.transform.rotation.w = odom->pose.pose.orientation.w;

  tfBroadcaster->sendTransform(transformMsg);
}

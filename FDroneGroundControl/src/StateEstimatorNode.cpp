#include "include/StateEstimatorNode.hpp"

StateEstimatorNode::StateEstimatorNode() : rclcpp::Node("state_estimator_node")
{
  // Reference Frame Quality of Service to help us patch our topic info 
  rclcpp::QoS frameQoS = rclcpp::Qos(10);
  // Create our Subscriptions
  poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone/odom/pose", frameQoS, std::bind(&StateEstimatorNode::PoseCallback, this, _1));
  twistSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/drone/odom/twist", frameQoS, std::bind(&StateEstimatorNode::TwistCallback, this, _1));

  // Acquire the name of a certain drone 
  droneName_ = this->declare_parameters<std::string>("dronename", "drone");

  // Initialize the Transform Broadcaster (Essentially just a publisher), based on the node
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

StateEstimatorNode::~StateEstimatorNode()
{
}

void StateEstimatorNode::PoseCallback(
  const std::shared_ptr<geometry_msgs::msg::PoseStamped> & pose) const 
{
  geometry_msgs::msg::TransformStamped transformMsg;

  // Setup the header of our transform 
  transformMsg.header.stamp = this->get_clock()->now();
  transformMsg.header.frame_id = "world";
  transformMsg.child_frame_id = droneName_.c_str();

  // Get the 3D coordinates of the current drone position 
  transformMsg.transform.translation.x = pose->position.x;
  transformMsg.transform.translation.y = pose->position.y;
  transformMsg.transform.translation.z = pose->position.z;

  // Get the overall rotation of our current drone
  transformMsg.transform.rotation.x = pose->orientation.x;
  transformMsg.transform.rotation.y = pose->orientation.y;
  transformMsg.transform.rotation.z = pose->orientation.z;
  transformMsg.transform.rotation.w = pose->orientation.w;

  tfBroadcaster->sendTransform(transformMsg);
}

void StateEstimatorNode::TwistCallback(
  const std::shared_ptr<geometry_msgs::msg::TwistStamped> & twist) const 
{

}

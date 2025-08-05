#include "fdrone_ground_control/StateEstimatorNode.hpp"

StateEstimatorNode::StateEstimatorNode() : rclcpp::Node("state_estimator_node")
{
  // Reference Frame Quality of Service to help us patch our topic info 
  rclcpp::QoS frameQoS = rclcpp::QoS(10);
  frameQoS.reliability(rclcpp::ReliabilityPolicy::Reliable);
  // Create our Subscriptions
  poseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/drone/odom/pose", frameQoS, std::bind(&StateEstimatorNode::PoseCallback, this, _1));
  twistSub = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/drone/odom/twist", frameQoS, std::bind(&StateEstimatorNode::TwistCallback, this, _1));

  // Acquire the name of a certain drone 
  droneName_ = this->declare_parameter<std::string>("dronename", "drone");

  // Initialize the Transform Broadcaster (Essentially just a publisher), based on the node
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

StateEstimatorNode::~StateEstimatorNode()
{
}

void StateEstimatorNode::PoseCallback(
  std::shared_ptr<geometry_msgs::msg::PoseStamped> poseMsg) 
{
  geometry_msgs::msg::TransformStamped transformMsg;

  // Setup the header of our transform 
  transformMsg.header.stamp = this->get_clock()->now();
  transformMsg.header.frame_id = "world";
  transformMsg.child_frame_id = droneName_.c_str();

  // Get the 3D coordinates of the current drone position 
  transformMsg.transform.translation.x = poseMsg->pose.position.x;
  transformMsg.transform.translation.y = poseMsg->pose.position.y;
  transformMsg.transform.translation.z = poseMsg->pose.position.z;

  // Get the overall rotation of our current drone
  transformMsg.transform.rotation.x = poseMsg->pose.orientation.x;
  transformMsg.transform.rotation.y = poseMsg->pose.orientation.y;
  transformMsg.transform.rotation.z = poseMsg->pose.orientation.z;
  transformMsg.transform.rotation.w = poseMsg->pose.orientation.w;

  tfBroadcaster->sendTransform(transformMsg);
}

void StateEstimatorNode::TwistCallback(
  std::shared_ptr<geometry_msgs::msg::TwistStamped> twistMsg) 
{
  RCLCPP_WARN(this->get_logger(), "Hello Unused Twist");
}

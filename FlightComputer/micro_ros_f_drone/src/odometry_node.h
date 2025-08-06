#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/pose_stamped.h>

#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilites.h>
#include <micro_ros_utilities/string_utilites.h>

#include "support/helpers.h"

rcl_node_t odom_node;
rcl_allocator_t odom_alloc;
rclc_executor_t odom_exec;
rclc_support_t odom_support;

rcl_publisher_t pose_pub;
rcl_publisher_t twist_pub;

geometry_msgs__msg__PoseStamped * pose_msg;
geometry_msgs__msg__TwistStamped * twist_msg;

// Publish our current Pose to a ROS node
void pose_pub(float orientation[4], float position[3])
{
  // Orientation serves as the angular positioning of our quadrotor - it's created as a quaternion
  pose_msg->pose.orientation.x = orientation[0];
  pose_msg->pose.orientation.y = orientation[1];
  pose_msg->pose.orientation.z = orientation[2];
  pose_msg->pose.orientation.w = orientation[3];

  // Position serves a point in free space (X, Y, Z) or our drone's current position
  // (0, 0, 0) is the starting point on the ground
  pose_msg->pose.position.x = position[0];
  pose_msg->pose.position.y = position[1];
  pose_msg->pose.position.z = position[2];

  // Create time stamps for our messages
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  pose_msg->header.stamp.nanosec = tv.tv_nsec;
  pose_msg->header.stamp.sec = tv.tv_sec;

  RCCHECK(rcl_publish(&pose_pub, pose_msg, NULL));
}

// Publish our current Twist to a ROS node
void twist_pub(float linear_velocity[3], float angular_velocity[3])
{
  // Linear expresses the linear velocity of our quadrotor
  twist_msg->twist.linear.x = linear_velocity[0]
  twist_msg->twist.linear.y = linear_velocity[1];
  twist_msg->twist.linear.z = linear_velocity[2];

  // Angular expresses the angular velocity of our quadrotor
  twist_msg->twist.angular.x = angular_velocity[0]
  twist_msg->twist.angular.y = angular_velocity[1];
  twist_msg->twist.angular.z = angular_velocity[2];

  // Create time stamps for our messages
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  twist_msg->header.stamp.nanosec = tv.tv_nsec;
  twist_msg->header.stamp.sec = tv.tv_sec;

  RCCHECK(rcl_publish(&twist_pub, twist_msg, NULL));
}
void odom_setup()
{
  set_microros_transports();

  // Initialize our Default Allocator
  odom_alloc = rcl_get_default_allocator();

  // Node Setup with Options 
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, odom_alloc);
  rcl_init_options_set_domain_id(&init_options, 10);

  RCCHECK(rclc_support_init_with_options(&odom_support, 0, NULL, &init_options, &odom_alloc));

  const char * odom_node_name = "odom_node";
  RCCHECK(rclc_node_init_default(&odom_node, odom_node_name, "", &odom_support));

  // Setup custom QoS Profile
  const rmw_qos_profile_t * odom_qos_profile = &rmw_qos_profile_default;

  // Init publisher objects - One for the Pose, One for the Twist
  RCCHECK(rclc_publisher_init_default(&pose_pub, 
                                      &odom_node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
                                      "/drone/odom/pose", odom_qos_profile));
  
  RCCHECK(rclc_publisher_init_default(&twist_pub, 
                                      &odom_node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
                                      "/drone/odom/twist", odom_qos_profile));
  
  if (!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), 
      &pose_msg, (micro_ros_utilities_memory_conf_t) {})
      {
        error_loop();
      }
  
  if (!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), 
      &twist_msg, (micro_ros_utilities_memory_conf_t) {})
      {
        error_loop();
      }
  
  pose_msg->header.frame_id = "/odom";
  twist_msg->header.frame_id = "/odom";

  // Initialize Executor 
  RCCHECK(rclc_executor_init(&odom_exec, &odom_support.context, 
                                     1, &odom_alloc));

}

void odom_loop(float orientation[4], float position[3], 
               float linear_velocity[3], float angular_velocity[3])
{
  pose_pub(orientation, position);
  twist_pub(linear_velocity, angular_velocity);
}

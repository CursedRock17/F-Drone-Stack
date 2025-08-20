#include <micro_ros_arduino.h>
#include <Arduino.h>

#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "support/helpers.h"

// Necessary publisher and message we want to provide to update "intelligence stack"
rcl_publisher_t odom_publisher;
nav_msgs__msg__Odometry odom_msg;

// Necessary object support
rclc_executor_t odom_executor;
rclc_support_t odom_support;
rcl_allocator_t odom_allocator;
rcl_node_t odom_node;
rcl_timer_t odom_timer;

// Publish our current Odometry to a ROS node
// Broken into a Pose and a Twist
void odom_pub_callback(float orientation[4], float position[3], 
                       float lin_velocity[3], float ang_velocity[3])
{
  //RCSOFTCHECK(rclc_executor_spin_some(&odom_executor, RCL_MS_TO_NS(100)));
  // Create time stamps for our messages
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  odom_msg.header.stamp.nanosec = tv.tv_nsec;
  odom_msg.header.stamp.sec = tv.tv_sec;

  // To a frame which the pose will actually point to
  odom_msg.header.frame_id =
    micro_ros_string_utilities_set(odom_msg.header.frame_id, "drone/odom");
  odom_msg.child_frame_id =
    micro_ros_string_utilities_set(odom_msg.child_frame_id, "drone/base_link");

  // Position serves a point in free space (X, Y, Z) or our drone's current position
  // (0, 0, 0) is the starting point on the ground "world-frame"
  odom_msg.pose.pose.position.x = position[0];
  odom_msg.pose.pose.position.y = position[1];
  odom_msg.pose.pose.position.z = position[2];

  // Orientation serves as the angular positioning of our quadrotor - it's created as a quaternion
  odom_msg.pose.pose.orientation.x = orientation[0];
  odom_msg.pose.pose.orientation.y = orientation[1];
  odom_msg.pose.pose.orientation.z = orientation[2];
  odom_msg.pose.pose.orientation.w = orientation[3];

  // Pose has an uncertainty, covariance 6x6 matrix related to it, ignore it for now
  // On reimplementation, go back and use i,j instead of just i
  for(int i = 0; i < 36; i++){
    odom_msg.pose.covariance[i] = 1e-9; 
  }
  
  // Twist provides the linear and angular velocities of our quad based on the center
  // of mass
  odom_msg.twist.twist.linear.x = lin_velocity[0];
  odom_msg.twist.twist.linear.y = lin_velocity[1];
  odom_msg.twist.twist.linear.z = lin_velocity[2];
  
  odom_msg.twist.twist.angular.x = ang_velocity[0];
  odom_msg.twist.twist.angular.y = ang_velocity[1];
  odom_msg.twist.twist.angular.z = ang_velocity[2];

  // Twist has an uncertainty, covariance 6x6 matrix related to it, ignore it for now
  // On reimplementation, go back and use i,j instead of just i
  for(int i = 0; i < 36; i++){
    odom_msg.twist.covariance[i] = 1e-9; 
  }

  RCCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void odom_setup()
{
  set_microros_transports();
  
  odom_allocator = rcl_get_default_allocator();

  //create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, odom_allocator));
  RCCHECK(rclc_support_init_with_options(&odom_support, 0, NULL, &init_options, &odom_allocator));

  // create node
  const char * odom_node_name = "odometry_node";
  RCCHECK(rclc_node_init_default(&odom_node, odom_node_name, "", &odom_support));

  // Setup custom QoS Profile
  const rmw_qos_profile_t * odom_qos_profile = &rmw_qos_profile_default;

  // Init publisher object - One for the Odometry
  RCCHECK(rclc_publisher_init(&odom_publisher, 
                              &odom_node,
                              ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
                              "/drone/odom", odom_qos_profile));

  // create executor
  RCCHECK(rclc_executor_init(&odom_executor, &odom_support.context, 1, &odom_allocator));
}

void odom_loop(float orientation[4], float position[3], 
               float linear_velocity[3], float angular_velocity[3])
{
  odom_pub_callback(orientation, position, linear_velocity, angular_velocity);
}

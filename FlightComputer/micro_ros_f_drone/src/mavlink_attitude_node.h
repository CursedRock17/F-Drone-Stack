#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <macros_msgs/msg/attitude_target.h>

#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilites.h>
#include <micro_ros_utilities/string_utilites.h>

#include "support/helpers.h"

rcl_publisher_t mavlink_pub;
rcl_allocator_t mavlink_alloc;
rclc_executor_t mavlink_exec;
rclc_support_t mavlink_support;

rcl_node_t mavlink_node;
rcl_timer_t mavlink_timer;

mavros_msgs__msg__AttitudeTarget * attitude_msg;

void timer_callback(rcl_timer_t * timer, int64_t timeout)
{
  UNUSED(timer);
  UNUSED(timeout);
}

void attitude_pub(float quaternion_[4], float imu_PID_[3], float thrust_)
{
  // Publish our current attitude to the ROS node so that it's info 
  // is discoverable to ground control
  attitude_msg->orientation.x = quaternion_[0];
  attitude_msg->orientation.y = quaternion_[1];
  attitude_msg->orientation.z = quaternion_[2];
  attitude_msg->orientation.w = quaternion_[3];

  // Roll, Pitch, Yaw Rates from controlRate()
  attitude_msg->body_rate.x = imu_PID_[0];
  attitude_msg->body_rate.y = imu_PID_[1];
  attitude_msg->body_rate.z = imu_PID_[2];

  attitude_msg->thrust = thrust_;

  // Create time stamps for our messages
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  attitude_msg->header.stamp.nanosec = tv.tv_nsec;
  attitude_msg->header.stamp.sec = tv.tv_sec;

  RCCHECK(rcl_publish(&mavlink_pub, attitude_msg, NULL));
}

void attitude_setup()
{
  set_microros_transports();

  // Initialize our Default Allocator
  mavlink_alloc = rcl_get_default_allocator();

  
  // Node Setup with Options 
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, mavlink_alloc);
  rcl_init_options_set_domain_id(&init_options, 9);

  RCCHECK(rclc_support_init_with_options(&mavlink_support, 0, NULL, &init_options, &mavlink_alloc));

  const char * mavlink_node_name = "mavlink_attitude_node";
  RCCHECK(rclc_node_init_default(&mavlink_node, mavlink_node_name, "", &mavlink_support));

  // Setup custom QoS Profile
  const rmw_qos_profile_t * mavlink_qos_profile = &rmw_qos_profile_default;


  // Init publisher objects
  RCCHECK(rclc_publisher_init_default(&mavlink_pub, 
                                      &mavlink_attitude_node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(mavros_msgs, msg, AttitudeTarget),
                                      "/drone/attitude", mavlink_qos_profile));
  
  if (!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(mavros_msgs, msg, AttitudeTarget), 
      &attitude_msg, (micro_ros_utilities_memory_conf_t) {})
      {
        error_loop();
      }
  
  attitude_msg->header.frame_id = "/";

  // Timer Setup
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &mavlink_timer, 
    &mavlink_support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Initialize Executor 
  RCCHECK(rclc_executor_init(&mavlink_exec, &mavlink_support.context, 
                                     1, &mavlink_alloc));

  // Add the timer to the Executor to take in drone info every 1000 ms (1 sec) 
  RCCHECK(rclc_exectutor_add_timer(&mavlink_exec, &mavlink_timer));

}

void attitude_loop(float quaternion_[4], float imu_PID_[3], float thrust_)
{
  attitude_pub(quaternion_, imu_PID_, thrust_);
}

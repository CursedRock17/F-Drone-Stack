#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <mavros_msgs/msg/waypointh>

#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilites.h>
#include <micro_ros_utilities/string_utilites.h>

#include "support/helpers.h"

rcl_node_t waypoint_node;
rcl_allocator_t waypoint_alloc;
rclc_executor_t waypoint_exec;
rclc_support_t waypoint_support;

rcl_subscription_t waypoint_sub;

mavros_msgs__msg__Waypoint * waypoint_msg

// Callback to grab any incoming waypoints
void waypoint_sub_callback(const void * waypoint_msg_)
{
}

void waypoint_setup()
{
  set_microros_transports();

  // Initialize our Default Allocator
  waypoint_alloc = rcl_get_default_allocator();

  // Node Setup with Options 
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, waypoint_alloc);
  rcl_init_options_set_domain_id(&init_options, 8);

  RCCHECK(rclc_support_init_with_options(&waypoint_support, 0, NULL, &init_options, &waypoint_alloc));

  const char * waypoint_node_name = "waypoint_node";
  RCCHECK(rclc_node_init_default(&waypoint_node, waypoint_node_name, "", &waypoint_support));

  // Setup custom QoS Profile
  const rmw_qos_profile_t * waypoint_qos_profile = &rmw_qos_profile_default;

  // Init publisher objects - One for the Pose, One for the Twist
  RCCHECK(rclc_subscription_init(&maypoint_sub,
                                 &waypoint_node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(mavros_msgs, msg, Waypoint),
                                 "/drone/waypoint", waypoint_qos_profile));
  
  if (!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(mavros_msgs, msg, Waypoint), 
      &waypoint_msg, (micro_ros_utilities_memory_conf_t) {})
      {
        error_loop();
      }
  
  waypoint_msg->header.frame_id = "/";

  // Initialize Executor 
  RCCHECK(rclc_executor_init(&waypoint_exec, &waypoint_support.context, 1, &waypoint_alloc));
  RCCHECK(rclc_executor_add_subscription(&waypoint_exec, &waypoint_sub, &waypoint_msg,
                                         &waypoint_sub_callback, ON_NEW_DATA));
}

void waypoint_loop( )
{
  waypoint_sub_callback(waypoint_msg);
}

#include <stdio.h>
#include <stdint.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <macros_msgs/msg/esc_telemetry.h>

#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilites.h>
#include <micro_ros_utilities/string_utilites.h>

#include "support/helpers.h"

rcl_publisher_t esc_telem_pub;
rcl_allocator_t esc_telem_alloc;
rclc_executor_t esc_telem_exec;
rclc_support_t esc_telem_support;

rcl_node_t esc_telem_node;
rcl_timer_t esc_telem_timer;

mavros_msgs__msg__ESCTelemetry * telem_msg;

void timer_update(rcl_timer_t * timer, int64_t timeout)
{
  UNUSED(timer);
  UNUSED(timeout);
}

// Publish the Current Telemetry (ESC Given) Values
void telem_pub(float temperature_, float voltage_, float current_, 
               float total_current_, int32_t rpm_, unsigned int count_)
{
  // Create time stamps for our messages
  struct timespec tv = {0};
  clock_gettime(0, &tv);

  // Only one ESC : the 4 in 1
  telem_msg->esc_telemetry[0].temperature = temperature_;
  telem_msg->esc_telemetry[0].voltage = voltage_;
  telem_msg->esc_telemetry[0].current = current_;
  telem_msg->esc_telemetry[0].total_current = total_current_;
  telem_msg->esc_telemetry[0].rpm = rpm_;
  telem_msg->esc_telemetry[0].count = count_; 

  telem_msg->header.stamp.nanosec = tv.tv_nsec;
  telem_msg->header.stamp.sec = tv.tv_sec;

  RCCHECK(rcl_publish(&esc_telem_pub, telem_msg, NULL));
}

void telem_init()
{
  set_microros_transports();

  // Initialize our Default Allocator
  esc_telem_alloc = rcl_get_default_allocator();

  
  // Node Setup
  RCCHECK(rclc_support_init(&esc_telem_support, 0, NULL, 
                                   &esc_telem_alloc));
  RCCHECK(rclc_node_init_default(&esc_telem_node,
                                 "esc_telem_telem_node","",
                                 &esc_telem_support));


  // Init publisher objects
  RCCHECK(rclc_publisher_init_default(&esc_telem_pub, 
                                      &esc_telem_telem_node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(mavros_msgs, msg, ESCTelemetry),
                                     "/telem"));
  
  if (!micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(mavros_msgs, msg, ESCTelemetry), 
      &telem_msg, (micro_ros_utilities_memory_conf_t) {})
      {
        error_loop();
      }
  
  telem_msg->header.frame_id = "/";

  // Timer Setup
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &esc_telem_timer, 
    &esc_telem_support,
    RCL_MS_TO_NS(timer_timeout),
    timer_update));

  // Initialize Executor 
  RCCHECK(rclc_executor_init(&esc_telem_exec, &esc_telem_support.context, 
                                     1, &esc_telem_alloc));
  RCCHECK(rclc_executor_add_timer(&esc_telem_exec, &esc_telem_timer));
}

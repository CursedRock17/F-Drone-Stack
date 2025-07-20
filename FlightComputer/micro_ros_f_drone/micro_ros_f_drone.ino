#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

// ------------ Declare Objects ----------------//

// Create Default Objects to Actually Publish Data
rcl_publisher_t publisher;
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;

rcl_node_t node;
rcl_timer_t timer;

// Access the messages we need to use
tf2_msgs__Msg__TFMessage * tf_message;

// Access the "hardware"
cIMU IMU;

// ------------ Helper Functions -------------//

// RCCheck ensures the functions return a clean value, otherwise we can't
// report the error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void timer_callback(rcl_timer_t * timer, int64_t last_time)
{
  RCLC_UNUSED(last_time);
  RCLC_UNUSED(timer)
}

// ------------ Duino Functions -----------//

void setup()
{
  set_microros_transports();
  IMU.begin();

  // Calibrate the IMU
  IMU.SEN.acc_cali_start();
  while (IMU.SEN.acc_cali_get_done() == false) {
    IMU.update();
  }

  delay(500);

  // Initialize our ROS-based structs
  allocator = rcl_get_default_allocator();

  // Create and correctly init our main node
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "teensy_tf_node", "", &support));

  // Create our publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  // Create our Timer
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create the executor
  RCCHECK(rclc_executor_init(&executor, &support.support, 1, &allocator));

}

void loop()
{

}

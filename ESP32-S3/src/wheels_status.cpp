#include "wheels_status.h"
#include <Arduino.h>
#include <std_msgs/msg/u_int8.h>
#include <rcl/publisher.h>
#include "ros_utils.h"

const int LEFT_TOUCH_PIN = 7;
const int RIGHT_TOUCH_PIN = 16;

rcl_publisher_t status_pub;
std_msgs__msg__UInt8 status_msg;
rcl_timer_t status_timer;

bool last_motor_status = false;

void update_status_message() {
  uint8_t status = 0;

  if (last_motor_status) status |= (1 << 0);
  if (digitalRead(LEFT_TOUCH_PIN) == HIGH)  status |= (1 << 1);
  if (digitalRead(RIGHT_TOUCH_PIN) == HIGH) status |= (1 << 2);

  status_msg.data = status;
  SAFE_PUB(&status_pub, status_msg);
}

void wheels_status_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    update_status_message();
  }
}

void init_wheels_status(rcl_node_t* node, rclc_executor_t* executor, rclc_support_t* support) {
  pinMode(LEFT_TOUCH_PIN, INPUT_PULLUP);
  pinMode(RIGHT_TOUCH_PIN, INPUT_PULLUP);

  rclc_publisher_init_default(&status_pub, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
    "wheels_status");
    
  const unsigned int timer_period_ms = 200;
  rclc_timer_init_default(&status_timer, support, RCL_MS_TO_NS(timer_period_ms), wheels_status_timer_callback);
  rclc_executor_add_timer(executor, &status_timer);
}

void update_wheels_status(bool motors_enabled) {
  last_motor_status = motors_enabled;
}

bool get_wheels_status() {
    return last_motor_status;
}
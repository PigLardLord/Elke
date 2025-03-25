#include "encoders.h"
#include <Arduino.h>
#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/empty.h>
#include <rclc/executor.h>
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

const int LEFT_ENCODER_PIN = 8;
const int RIGHT_ENCODER_PIN = 16;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// ROS objects
rcl_publisher_t pub_left, pub_right;
rcl_timer_t timer;
rcl_service_t reset_service;

std_msgs__msg__Int32 msg_left, msg_right;

void IRAM_ATTR leftEncoderISR() {
  leftTicks++;
}

void IRAM_ATTR rightEncoderISR() {
  rightTicks++;
}

void reset_callback(const void * req, void * res) {
  (void)req;
  (void)res;
  leftTicks = 0;
  rightTicks = 0;
  Serial.println("✅ Encoders reset!");
}

void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg_left.data = leftTicks;
    msg_right.data = rightTicks;

    RCSOFTCHECK(rcl_publish(&pub_left, &msg_left, NULL));
    RCSOFTCHECK(rcl_publish(&pub_right, &msg_right, NULL));
  }
}

void init_encoders(rcl_node_t* node, rclc_executor_t* executor, rclc_support_t* support){
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoderISR, RISING);

  // Init publishers
  rclc_publisher_init_default(&pub_left, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_encoder_ticks");

  rclc_publisher_init_default(&pub_right, node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_encoder_ticks");

  // Init timer
  const unsigned int timeout = 100; // ms
  rclc_timer_init_default(&timer, support, RCL_MS_TO_NS(timeout), encoder_timer_callback);

  // Init service
  rclc_service_init_default(&reset_service, node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
    "reset_encoders");

  // Add to executor
  rclc_executor_add_timer(executor, &timer);
  rclc_executor_add_service(executor, &reset_service, nullptr, nullptr, reset_callback);
}

long get_left_ticks() {
  return leftTicks;
}

long get_right_ticks() {
  return rightTicks;
}

void reset_encoders() {
  leftTicks = 0;
  rightTicks = 0;
}

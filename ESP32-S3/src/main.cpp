#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_srvs/srv/empty.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }


// === CONFIGURATION ===
const int leftEncoderPin = 8;
const int rightEncoderPin = 16;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

// === micro-ROS objects ===
rcl_publisher_t pub_left, pub_right;
rcl_timer_t timer;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_service_t reset_service;

std_msgs__msg__Int32 msg_left, msg_right;

// === Encoder ISRs ===
void IRAM_ATTR leftEncoderISR() {
  leftTicks++;
}

void IRAM_ATTR rightEncoderISR() {
  rightTicks++;
}

// === Reset service callback ===
void reset_callback(const void *req, void *res) {
  (void)req;
  (void)res;
  leftTicks = 0;
  rightTicks = 0;
  Serial.println("✅ Encoder ticks reset via service");
}

// === Timer callback ===
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg_left.data = leftTicks;
    msg_right.data = rightTicks;

    RCSOFTCHECK(rcl_publish(&pub_left, &msg_left, NULL));
    RCSOFTCHECK(rcl_publish(&pub_right, &msg_right, NULL));
  }
}


// === Setup ===
void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Set encoder pins
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderISR, RISING);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "encoder_calibration_node", "", &support);

  // Publishers
  rclc_publisher_init_default(&pub_left, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_encoder_ticks");

  rclc_publisher_init_default(&pub_right, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_encoder_ticks");

  // Timer
  const unsigned int timer_timeout = 100; // ms
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  // Service to reset encoders
  rclc_service_init_default(&reset_service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
    "reset_encoders");

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_service(&executor, &reset_service, NULL, NULL, reset_callback);

  msg_left.data = 0;
  msg_right.data = 0;

  Serial.println("✅ micro-ROS encoder calibration node is running");
}

// === Main loop ===
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}

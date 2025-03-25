#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "encoders.h"
#include "cmd_vel.h"
#include "motors.h"
#include "wheels_status.h"

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

void error_loop() {
  while (true) {
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "diff_drive_node", "", &support));
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));

  // Init modules
  init_encoders(&node, &executor, &support);
  init_cmd_vel_subscription(&node, &executor);
  init_wheels_status(&node, &executor, &support);
  init_motors();

  Serial.println("🚗 micro-ROS differential drive node ready!");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  float v_left = get_target_velocity_left();
  float v_right = get_target_velocity_right();
  update_motor_pwm(v_left, v_right);
  update_wheels_status(get_wheels_status());
  delay(10);
}
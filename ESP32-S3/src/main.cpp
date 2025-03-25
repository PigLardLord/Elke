#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "ros_utils.h"
#include "encoders.h"
#include "cmd_vel.h"
#include "motors.h"
#include "wheels_status.h"

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

void setup() {
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  init_ros_base(&allocator, &support, &node, &executor, "diff_drive_node");

  init_encoders(&node, &executor, &support);
  init_cmd_vel_subscription(&node, &executor);
  init_motors();
  init_wheels_status(&node, &executor, &support);

  Serial.println("🚀 micro-ROS base setup complete");
}

void loop() {
  spin_executor(&executor);

  update_motor_pwm(get_target_velocity_left(), get_target_velocity_right());
  update_wheels_status(get_wheels_status());
}
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "ros_utils.h"
#include "encoders.h"
#include "cmd_vel.h"
#include "motors.h"
#include "wheels_status.h"
#include <HardwareSerial.h>

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
HardwareSerial serial_port(1);

void ros_setup() {
  init_ros_base(&allocator, &support, &node, &executor, "diff_drive_node");
  init_encoders(&node, &executor, &support);
  init_cmd_vel_subscription(&node, &executor);
  init_motors();
  init_wheels_status(&node, &executor, &support);
}

void ros_teardown() {
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  serial_port.begin(115200, SERIAL_8N1, 44, 43); 
  set_microros_serial_transports(serial_port);
  ros_setup();

  Serial.println("🚀 micro-ROS base setup complete");
}

void loop() {
  
  if (!is_connected()) {
    Serial.println("⚠️ Lost connection to micro-ROS agent!");
    reconnect_micro_ros(ros_teardown, ros_setup);
  }

  update_motor_pwm(get_target_velocity_left(), get_target_velocity_right());
  update_wheels_status(get_wheels_status());
}
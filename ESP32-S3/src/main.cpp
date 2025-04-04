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

void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);
  Serial.println("✅ USB serial should work");
  
}

void loop() {
  // spin_executor(&executor);

  // update_motor_pwm(get_target_velocity_left(), get_target_velocity_right());
  // update_wheels_status(get_wheels_status());
}
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "ros_utils.h"
#include "encoders.h"
#include "cmd_vel.h"
#include "motors.h"
#include "wheels_status.h"

// Define micro-ROS allocator and other variables
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

void setup() {
  // Initialize USB CDC for micro-ROS
  Serial.begin(115200);

  while (!Serial) {
    delay(100);
  }

  set_microros_serial_transports(Serial);  // Use USB CDC (Serial) for micro-ROS
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();
  
  // Create init options
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("❌ Failed to initialize ROS 2");
    return;
  }
  
  // Create node
  if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) {
    Serial.println("❌ Failed to create node");
    return;
  }
  
  // Create executor
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("❌ Failed to initialize executor");
    return;
  }
  init_ros_base(&allocator, &support, &node, &executor, "esp32_node", 4);
  init_cmd_vel_subscription(&node, &executor);
  init_encoders(&node, &executor, &support);
  init_wheels_status(&node, &executor, &support);
  init_motors();
}

void loop() {
  spin_executor(&executor);
}
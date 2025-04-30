#include "ros_utils.h"
#include <Arduino.h>
#include <rmw_microros/ping.h>

void error_loop() {
  while (true) {
    delay(500);
  }
}

void wait_for_agent()
{
  const int max_attempts = 120;
  int attempt = 0;

  while (rmw_uros_ping_agent(1000, 5) != RCL_RET_OK) {
    Serial.println("Waiting for micro-ROS Agent...");
    delay(1000);

    attempt++;
    if (attempt >= max_attempts) {
      Serial.println("Agent not found, restarting ESP32...");
      ESP.restart();  // Optional: restart if Agent is never found
    }
  }
}


void init_ros_base(
  rcl_allocator_t* allocator,
  rclc_support_t* support,
  rcl_node_t* node,
  rclc_executor_t* executor,
  const char* node_name,
  size_t executor_handles
) {
  // Wait for micro-ROS Agent
  wait_for_agent();

  // Initialize ROS base
  *allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(support, 0, NULL, allocator));
  RCCHECK(rclc_node_init_default(node, node_name, "", support));
  RCCHECK(rclc_executor_init(executor, &support->context, executor_handles, allocator));
}

void spin_executor(rclc_executor_t* executor, unsigned int timeout_ms) {
  rcl_ret_t rc = rclc_executor_spin_some(executor, RCL_MS_TO_NS(timeout_ms));
  (void)rc;
}

#include "ros_utils.h"
#include <Arduino.h>
#include <rmw_microros/ping.h>

static bool agent_connected = false;

bool is_connected() {
  return agent_connected;
}

void reconnect_micro_ros(std::function<void()> teardown_fn, std::function<void()> setup_fn) {
  agent_connected = false;

  teardown_fn();

  delay(500);
  Serial.println("🔁 Attempting reconnection...");

  while (!rmw_uros_ping_agent(100, 5)) {
    Serial.println("🔄 Waiting for agent to come back...");
    delay(1000);
  }

  Serial.println("✅ Agent available! Reinitializing...");
  setup_fn();

  agent_connected = true;
}

void error_loop() {
  while (true) {
    Serial.println("❌ ERROR: Halting in error_loop()");
    delay(500);
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
  *allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(support, 0, NULL, allocator));
  RCCHECK(rclc_node_init_default(node, node_name, "", support));
  RCCHECK(rclc_executor_init(executor, &support->context, executor_handles, allocator));
}

void spin_executor(rclc_executor_t* executor, unsigned int timeout_ms) {
  rcl_ret_t rc = rclc_executor_spin_some(executor, RCL_MS_TO_NS(timeout_ms));
  (void)rc;
}

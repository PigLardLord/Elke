#pragma once

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Critical check: halt if a ROS function fails
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }

// Soft check: ignore result but compile-safe
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// Inline helper for cleaner publishing
inline void safe_publish(rcl_publisher_t* pub, const void* msg) {
  rcl_ret_t rc = rcl_publish(pub, msg, nullptr);
  (void)rc;
}

#define SAFE_PUB(pub, msg) safe_publish(pub, &msg)

// Setup helper
void init_ros_base(
  rcl_allocator_t* allocator,
  rclc_support_t* support,
  rcl_node_t* node,
  rclc_executor_t* executor,
  const char* node_name,
  size_t executor_handles = 4
);

// Spin helper (non-blocking)
void spin_executor(rclc_executor_t* executor, unsigned int timeout_ms = 100);

// Fatal halt loop
void error_loop();
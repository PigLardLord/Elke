#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void init_wheels_status(rcl_node_t* node, rclc_executor_t* executor, rclc_support_t* support);
void update_wheels_status(bool motors_enabled);
bool get_wheels_status();
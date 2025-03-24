#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

void init_encoders(rcl_node_t* node, rclc_executor_t* executor, rclc_support_t* support);
long get_left_ticks();
long get_right_ticks();
void reset_encoders();
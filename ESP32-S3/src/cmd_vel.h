#pragma once
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/executor.h>

void init_cmd_vel_subscription(rcl_node_t* node, rclc_executor_t* executor);
float get_target_velocity_left();
float get_target_velocity_right();
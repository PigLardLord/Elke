#include "cmd_vel.h"
#include "robot_params.h"

static float target_left_velocity = 0.0;
static float target_right_velocity = 0.0;

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear = msg->linear.x;    // Forward/backward velocity
  float angular = msg->angular.z;  // Rotational velocity

  // Differential drive kinematics
  target_left_velocity  = linear - (angular * HALF_WHEEL_BASE);
  target_right_velocity = linear + (angular * HALF_WHEEL_BASE);
}

void init_cmd_vel_subscription(rcl_node_t* node, rclc_executor_t* executor) {
  static rcl_subscription_t cmd_vel_sub;
  static geometry_msgs__msg__Twist cmd_vel_msg;

  rclc_subscription_init_default(
    &cmd_vel_sub,
    node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  rclc_executor_add_subscription(executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
}

float get_target_velocity_left() {
  return target_left_velocity;
}

float get_target_velocity_right() {
  return target_right_velocity;
}
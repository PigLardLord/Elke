#include "motors.h"
#include <Arduino.h>
#include "robot_params.h"

const int LEFT_PWM_PIN = 13;
const int RIGHT_PWM_PIN = 3;

const int PWM_RESOLUTION = 8;         // bits
const int PWM_FREQUENCY = 20000;      // Hz
const int PWM_MAX = 255;              // 2^8 - 1

// Motor speed limits (m/s) — define based on your robot
const float MAX_SPEED_MPS = 0.5f;

void init_motors() {
  ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION); // channel 0 → left motor
  ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION); // channel 1 → right motor
  ledcAttachPin(LEFT_PWM_PIN, 0);
  ledcAttachPin(RIGHT_PWM_PIN, 1);
}

void update_motor_pwm(float left_speed_mps, float right_speed_mps) {
  // Clip speeds to allowed range
  left_speed_mps = constrain(left_speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  right_speed_mps = constrain(right_speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);

  // Map speed to PWM
  int left_pwm = (int)((fabs(left_speed_mps) / MAX_SPEED_MPS) * PWM_MAX);
  int right_pwm = (int)((fabs(right_speed_mps) / MAX_SPEED_MPS) * PWM_MAX);

  // Apply PWM
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);

  // TODO: Add direction control via H-bridge (not just speed!)
}

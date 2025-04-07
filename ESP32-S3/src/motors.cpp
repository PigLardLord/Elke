#include "motors.h"
#include <Arduino.h>
#include "robot_params.h"

// Motor control pins
const int LEFT_IN1  = 46;
const int LEFT_IN2  = 9;
const int RIGHT_IN1 = 11;
const int RIGHT_IN2 = 12;

// PWM config
const int PWM_FREQUENCY = 20000;
const int PWM_RESOLUTION = 8;
const int PWM_MAX = 255;

// Speed limits (m/s)
const float MAX_SPEED_MPS = 0.5f;

// Direction inversion flags
const bool INVERT_LEFT_MOTOR  = true;
const bool INVERT_RIGHT_MOTOR = false;

void init_motors() {
  ledcAttach(LEFT_IN1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(LEFT_IN2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(RIGHT_IN1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(RIGHT_IN2, PWM_FREQUENCY, PWM_RESOLUTION);
}

void drive_motor(int pin1, int pin2, float speed_mps) {
  int pwm_value = static_cast<int>((fabs(speed_mps) / MAX_SPEED_MPS) * PWM_MAX);

  if (speed_mps > 0) {
    ledcWrite(pin1, pwm_value);
    ledcWrite(pin2, 0);
  } else if (speed_mps < 0) {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, pwm_value);
  } else {
    ledcWrite(pin1, 0);
    ledcWrite(pin2, 0);
  }
}

void update_motor_pwm(float left_speed_mps, float right_speed_mps) {
  // Constrain speeds
  left_speed_mps  = constrain(left_speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  right_speed_mps = constrain(right_speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);

  // Apply motor inversion
  if (INVERT_LEFT_MOTOR)  left_speed_mps  = -left_speed_mps;
  if (INVERT_RIGHT_MOTOR) right_speed_mps = -right_speed_mps;

  // Drive motors
  drive_motor(LEFT_IN1, LEFT_IN2, left_speed_mps);
  drive_motor(RIGHT_IN1, RIGHT_IN2, right_speed_mps);
}

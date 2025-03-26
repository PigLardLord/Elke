#include "motors.h"
#include <Arduino.h>
#include "robot_params.h"

// Motor control pins
const int LEFT_PWM_PIN  = 13;
const int RIGHT_PWM_PIN = 3;

const int LEFT_IN1  = 9;
const int LEFT_IN2  = 46;
const int RIGHT_IN1 = 11;
const int RIGHT_IN2 = 12;
const int STBY_PIN  = 10;

// PWM config
const int PWM_RESOLUTION = 8;         // bits
const int PWM_FREQUENCY = 20000;      // Hz
const int PWM_MAX = 255;

// Speed limits (m/s)
const float MAX_SPEED_MPS = 0.5f;

void init_motors() {
  // Set up PWM channels
  ledcAttach(0, PWM_FREQUENCY, PWM_RESOLUTION); // Channel 0 = left
  ledcAttach(1, PWM_FREQUENCY, PWM_RESOLUTION); // Channel 1 = right

  // Set direction pins
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  // Enable motor driver (STBY = HIGH)
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void set_motor_direction(bool in1, bool in2, int pin1, int pin2) {
  digitalWrite(pin1, in1);
  digitalWrite(pin2, in2);
}

void update_motor_pwm(float left_speed_mps, float right_speed_mps) {
  // Clamp speeds
  left_speed_mps  = constrain(left_speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  right_speed_mps = constrain(right_speed_mps, -MAX_SPEED_MPS, MAX_SPEED_MPS);

  // Get absolute speeds
  int left_pwm = (int)((fabs(left_speed_mps) / MAX_SPEED_MPS) * PWM_MAX);
  int right_pwm = (int)((fabs(right_speed_mps) / MAX_SPEED_MPS) * PWM_MAX);

  // Set direction
  if (left_speed_mps >= 0) {
    set_motor_direction(HIGH, LOW, LEFT_IN1, LEFT_IN2);  // Forward
  } else {
    set_motor_direction(LOW, HIGH, LEFT_IN1, LEFT_IN2);  // Reverse
  }

  if (right_speed_mps >= 0) {
    set_motor_direction(HIGH, LOW, RIGHT_IN1, RIGHT_IN2); // Forward
  } else {
    set_motor_direction(LOW, HIGH, RIGHT_IN1, RIGHT_IN2); // Reverse
  }

  // Apply PWM
  ledcWrite(0, left_pwm);
  ledcWrite(1, right_pwm);
}

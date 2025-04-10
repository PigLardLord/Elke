#pragma once
#define _USE_MATH_DEFINES
#include <cmath>

// ⚙️ Physical dimensions
constexpr float WHEEL_DIAMETER = 0.069f;  // meters
constexpr float WHEEL_BASE     = 0.238f;  // meters ← from wheel center to center
constexpr float HALF_WHEEL_BASE = WHEEL_BASE / 2.0f;

// 🧮 Encoder details
constexpr int TICKS_PER_REV = 263;  // measured manually
constexpr float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
constexpr float TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ≈ 1214

// ⚡ Motor control
constexpr float MAX_SPEED_MPS = 0.5f; // Max linear speed in m/s

// 🔁 Motor direction inversion
constexpr bool INVERT_LEFT_MOTOR  = true;   // Set to true if left wheel is wired in reverse
constexpr bool INVERT_RIGHT_MOTOR = false;  // Set to true if right wheel is wired in reverse

// 🎚️ PWM configuration
constexpr int PWM_FREQUENCY   = 20000; // 20 kHz
constexpr int PWM_RESOLUTION  = 8;     // bits
constexpr int PWM_MAX         = (1 << PWM_RESOLUTION) - 1; // 255 for 8-bit

// 🔌 Motor driver pins (DRV8871)
constexpr int LEFT_IN1_PIN  = 46;
constexpr int LEFT_IN2_PIN  = 9;
constexpr int RIGHT_IN1_PIN = 11;
constexpr int RIGHT_IN2_PIN = 12;

// 🧭 Encoder pins
const int LEFT_ENCODER_PIN = 8;
const int RIGHT_ENCODER_PIN = 15;

// 🛑 Bumper switches
constexpr int LEFT_BUMPER_PIN  = 7;
constexpr int RIGHT_BUMPER_PIN = 15;

// 🕳️ Step detector IR sensors (analog inputs)
constexpr int STEP_SENSOR_FL_PIN = 36; // Front-left
constexpr int STEP_SENSOR_FR_PIN = 37; // Front-right
constexpr int STEP_SENSOR_RL_PIN = 38; // Rear-left
constexpr int STEP_SENSOR_RR_PIN = 39; // Rear-right

// ⚡ Charging base reflective sensor (analog input)
constexpr int BASE_SENSOR_PIN = 35;

// 📡 Ultrasound sensor
constexpr int ULTRASOUND_TRIGGER_PIN = 45;
constexpr int ULTRASOUND_ECHO_PIN    = 48;

// 📍 LiDAR UART
constexpr int LIDAR_RX_PIN = 17;
constexpr int LIDAR_TX_PIN = 18;

// 🧭 IMU (assumed I²C)
constexpr int IMU_SDA_PIN = 3;
constexpr int IMU_SCL_PIN = 10;

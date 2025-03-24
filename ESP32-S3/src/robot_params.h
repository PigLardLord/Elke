#pragma once
#define _USE_MATH_DEFINES
#include <cmath>

// ⚙️ Physical dimensions
constexpr float WHEEL_DIAMETER = 0.069f;  // in meters
constexpr float WHEEL_BASE     = 0.238f;   // meters ← from wheel center to center
constexpr float HALF_WHEEL_BASE =  WHEEL_BASE / 2.0f;

// 🧮 Encoder details
constexpr int TICKS_PER_REV    = 263;     // measured manually
constexpr float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
constexpr float TICKS_PER_METER = TICKS_PER_REV / WHEEL_CIRCUMFERENCE; // ≈ 1214

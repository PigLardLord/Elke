# 📘 Project ELKE_MB

**ELKE** (Electronic Lifeform with Knowledge & Emotion) is a home robot designed as a pet-like mobile assistant. It aims to explore indoor environments, build maps (SLAM), interact naturally via language, and support cloud IoT plugins (Alexa-style). This document provides detailed wiring, hardware description, and system architecture for the custom motherboard and peripherals.

---

## ⚙️ Main Components

- **Microcontroller**: ESP32-S3 DevKitC-1-N16R8
- **Main Unit**: Raspberry Pi 5 running ROS 2
- **Motors**: From Xiaomi Vacuum robot, controlled via TB6612FNG driver
- **LiDAR**: LDS02RR (Xiaomi) powered by DRV8833
- **Wheel Encoders**: Hall sensors integrated in motors
- **Bumpers**: Optical IR sensors (SX1025)
- **Cliff Sensors**: Infrared range sensors controlled via ULN2803A
- **Wheel Ground Switches**: Mechanical switches to detect ground contact

---

## 🪧️ Architecture Overview

- The **ESP32-S3** handles:
  - Motor control via PWM
  - Encoder signal reading
  - Bumper and cliff detection
  - LiDAR motor control
  - micro-ROS communication over UART

- The **Raspberry Pi 5** handles:
  - SLAM & navigation via ROS 2
  - Voice interaction & AI
  - High-level decision-making

---

## 📮️ Communication Interfaces

| Interface | Type | Description |
|----------|------|-------------|
| UART (GPIO 43/44) | Serial | micro-ROS communication with Raspberry Pi |
| UART (LiDAR) | Serial | LiDAR command/control interface |

---

## 🔌 ESP32 Pin Mapping

> ⚠️ **Note:** GPIO 46 is **input-only** and not recommended for driving outputs. It has been replaced in this table with GPIO 6, which is safe for digital output.

| GPIO | Connected To | Description | Type |
|------|--------------|-------------|------|
| 3    | PWM-A (TB6612) | Left motor speed control | PWM out |
| 9    | AIN1          | Left motor direction 1 | Digital out |
| 6    | AIN2 (replaces GPIO 46) | Left motor direction 2 | Digital out |
| 13   | PWM-B (TB6612) | Right motor speed control | PWM out |
| 11   | BIN1          | Right motor direction 1 | Digital out |
| 12   | BIN2          | Right motor direction 2 | Digital out |
| 10   | STBY (TB6612) | Motor driver enable | Digital out |
| 7    | Left wheel switch | LOW = wheel lifted | Digital in |
| 16   | Right wheel switch | LOW = wheel lifted | Digital in |
| 8    | Left encoder | Hall sensor signal | Digital in |
| 15   | Right encoder | Hall sensor signal | Digital in |
| 20   | PWM (DRV8833) | LiDAR motor speed | PWM out |
| 21   | SLEEP (DRV8833) | LiDAR driver enable | Digital out |
| 47   | FAULT (DRV8833) | LiDAR fault status | Digital in |
| 44   | UART RX | From Raspberry Pi | Serial in |
| 43   | UART TX | To Raspberry Pi | Serial out |
| 48   | ULN2803 Pin 1 | Bumper driver enable | Digital out |
| 45   | Bumper left (SX1025) | Obstacle detected = LOW | Digital in |
| 0    | Bumper right (SX1025) | Obstacle detected = LOW | Digital in |
| 35   | ULN2803 Pin 2 | Cliff sensors power | Digital out |
| 36   | IR sensor analog (cliff) | Range 0–1V = distance | Analog in |

---

## 🛆 External Modules

### TB6612FNG - Motor Driver
- Drives both left and right motors with direction and PWM
- Requires: IN1, IN2, PWM (A/B), STBY

### DRV8833 - LiDAR Motor Driver
- Controls the spinning motor in the LDS02RR module
- Uses: PWM, SLEEP, FAULT

### ULN2803A - IR Power Driver
- Allows switching infrared emitters and bumpers without overloading the ESP32

---

## 🌈 Wire Color Convention

| Color | Purpose |
|-------|---------|
| **Red** | +5V Power |
| **Green** | +12V Motor Power |
| **Black** | GND |
| **Orange** | UART TX |
| **Blue** | UART RX |
| **Brown** | Bumpers |
| **Purple** | PWM (LiDAR) |
| **Teal** | ULN2803 Control |
| **Gray** | Fault Lines |


## 🗏️ Motor/Encoder Wire Reference (Xiaomi)

| Color | Function | Notes |
|--------|----------|-------|
| Red | +12V Motor | Direct drive voltage |
| Black | GND Motor | Motor ground |
| Orange | +5V Encoder | Power supply encoder |
| Marrone | GND Encoder | Common GND, open collector |
| Blu | Encoder signal | Digital pulse |
| Bianco | Wheel lift switch | LOW = lifted |

> Warning: Xiaomi may **invert wire colors between left and right motors**. Double-check pinouts.

---

## 🔍 Resources

- [ESP32-S3 DevKitC-1 Docs](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide.html)
- [Kaia LDS LiDAR Library](https://github.com/kaiaai/LDS)
- [TB6612FNG Driver Guide (SparkFun)](https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide)
- [Xiaomi LDS Reverse Engineering Blog](https://makerspet.com/blog/how-to-connect-xiaomi-lds02rr-lidar-to-esp32/)

---

## 📍 Future Work
- Add display for eyes & emotion rendering
- Integrate voice recognition module
- Power control module with solar extension
- Expandable battery modules
- Finalize chassis and PCB design


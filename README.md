# 🦾 Arduino Robotic Arm Controller – Record & Playback

A complete, real-time control system for a 4-DOF robotic arm using an Arduino Mega, PCA9685 servo driver, potentiometers, and pushbuttons. This project enables manual manipulation, motion recording, and continuous looped playback with pause/resume functionality—perfect for pick-and-place demonstrations or educational robotics.

---

## ✨ Features

- **Manual Mode** – Move the arm in real-time using four analog potentiometers.
- **Record Mode** – Press the RECORD button to start capturing joint positions at **50ms intervals** (up to 200 steps). Press again to stop.
- **Playback Mode** – Press PLAY to loop the recorded sequence indefinitely.
- **Pause/Resume** – Freeze the arm during playback with the PAUSE button; press PLAY to continue.
- **Visual Status** – Three LEDs (Red, Green, Yellow) clearly indicate Recording, Playing, and Paused states.
- **Continuous Rotation Base** – Supports a 360° servo with configurable stop/speed pulses.
- **Adjustable Limits** – Easily tune pulse ranges (`JOINT_MIN`/`MAX`, `BASE_MIN`/`MAX`) via compile-time constants.

---

## 🧰 Hardware Requirements

| Component | Quantity | Details |
| :--- | :--- | :--- |
| Arduino Mega 2560 | 1 | Main controller |
| PCA9685 16-Channel Servo Driver | 1 | I2C interface, powers servos |
| 360° Continuous Rotation Servo | 1 | Base rotation (stop at 1500µs) |
| 180° Standard Servos | 3 | Joint 1, Joint 2, Gripper |
| Potentiometers (10kΩ) | 4 | Analog control (A0–A3) |
| Pushbuttons (Tactile) | 3 | Record, Play, Pause |
| LEDs (Red, Green, Yellow) | 3 | Status indicators |
| 220Ω Resistors | 3 | For LEDs |
| External 5-6V Power Supply | 1 | Dedicated power for servos (high current) |
| 1000µF Capacitor | 1 | Stabilize servo power lines (optional but recommended) |

---

## 🔌 Wiring Guide

### PCA9685 → Arduino Mega
| PCA9685 | Arduino Mega |
| :--- | :--- |
| SDA | Pin 20 (SDA) |
| SCL | Pin 21 (SCL) |
| VCC | 5V |
| GND | GND |

### Servos → PCA9685
| Servo | PCA9685 Channel |
| :--- | :--- |
| Base (360°) | Channel 0 |
| Joint 1 | Channel 1 |
| Joint 2 | Channel 2 |
| Gripper | Channel 3 |

### Potentiometers → Arduino Mega
| Potentiometer | Connection |
| :--- | :--- |
| Left Pin | 5V |
| Right Pin | GND |
| Middle Pin (Base) | A0 |
| Middle Pin (Joint 1) | A1 |
| Middle Pin (Joint 2) | A2 |
| Middle Pin (Gripper) | A3 |

### Buttons & LEDs → Arduino Mega
| Component | Pin | Note |
| :--- | :--- | :--- |
| Record Button | 22 | Active LOW (internal pull-up) |
| Play Button | 24 | Active LOW (internal pull-up) |
| Pause Button | 26 | Active LOW (internal pull-up) |
| Record LED (Red) | 28 | Anode via 220Ω to pin |
| Play LED (Green) | 30 | Anode via 220Ω to pin |
| Pause LED (Yellow) | 32 | Anode via 220Ω to pin |

> **⚠️ Power Note:** Connect the external 5-6V supply to the **PCA9685 V+ terminal**. Keep the Arduino powered via USB or its own supply. **Ensure all grounds (Arduino, PCA9685, external supply) are connected together.**

---

## 🚀 Getting Started

1. **Install Libraries** (via Arduino Library Manager):
   - `Adafruit PWM Servo Driver Library`
   - `Wire` (built-in)

2. **Clone this repository**:
   ```bash
   git clone https://github.com/yourusername/arduino-robot-arm-recorder.git

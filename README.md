# Electronically Controlled Air Spring Testbed (ESP32)

This repository contains the firmware and control strategies for an **Electronically Controlled Air Spring / Pneumatic Suspension Testbed** developed as part of an undergraduate Final Year Project. 

The system utilizes an **ESP32** microcontroller to achieve closed-loop height stabilization and dynamic leveling using pneumatic solenoid valves (inlet and exhaust), laser distance feedback, dual pressure monitoring, and 6-DOF IMU motion tracking.

---

## 🚀 Key Features

* **Multiple Control Strategies**: Includes baseline **Bang-Bang (On-Off)**, **PID**, and adaptive **Takagi-Sugeno Fuzzy Logic** controllers.
* **Low-Frequency Software PWM**: Utilizes software-based pulsing (5 Hz – 7 Hz) to drive solenoid coils efficiently without thermal saturation.
* **Signal Filtering**: Applies independent **Kalman Filters** on time-of-flight distance and dual pressure sensors to eliminate measurement noise.
* **Real-time Telemetry**: High-frequency CSV streaming over Serial (up to 50 Hz) for external logging and analysis.
* **Dynamic Tuning Interface**: Accepts live target height ($T$) and controller gain updates ($P$, $I$, $D$) over standard Serial commands.

---

## 🛠️ Hardware Requirements & Pinout

### Microcontroller
* **ESP32 Development Board**

### Actuators
* **Motor Driver**: L298N / Dual H-Bridge Driver
* **Solenoid Valves**: 12V Pneumatic Valves (Inlet & Exhaust)

### Sensors
* **ToF Laser Distance Sensor**: VL53L0X (I2C)
* **IMU Motion Sensor**: MPU6050 6-Axis (I2C)
* **Pressure Sensors**: Analog Transducers (Input Supply & Air Spring Bag)

---

### ESP32 Pinout Mapping

| Component | Function | ESP32 GPIO |
| :--- | :--- | :--- |
| **I2C SDA** | Sensors (VL53L0X & MPU6050) | `GPIO 21` |
| **I2C SCL** | Sensors (VL53L0X & MPU6050) | `GPIO 22` |
| **Exhaust Valve (Motor 1)** | Enable A (`enA`) | `GPIO 25` |
| | Input 1 (`in1`) | `GPIO 16` |
| | Input 2 (`in2`) | `GPIO 17` |
| **Inlet Valve (Motor 2)** | Enable B (`enB`) | `GPIO 26` |
| | Input 3 (`in3`) | `GPIO 19` |
| | Input 4 (`in4`) | `GPIO 18` |
| **Analog Pressure Sensors** | Supply / Inlet Pressure | `GPIO 33` |
| | Air Spring Bag Pressure | `GPIO 32` |

---

## 📁 Firmware Options & Control Algorithms

The codebase includes several iterations of the control system:

### 1. Bang-Bang (On-Off) Control (`Bang_Bang_Control.ino`)
* Simple threshold-based binary switching with a wide tolerance deadband ($\pm 20\text{ mm}$).
* **Best for**: Initial hardware sanity checks, wiring validation, and manual testing.

### 2. PID Control (`PID_Control.ino`)
* Dynamic PID calculation using height error ($\text{Target} - \text{Current}$).
* Features anti-windup clamping (`I_LIMIT = 50.0`) and deadzone resetting ($\pm 4\text{ mm}$).
* Translates continuous PID output into a 7 Hz software pulse width (20% – 100% duty cycle).

### 3. Fuzzy Logic Control (`Fuzzy_Control.ino`)
* Dual-input, single-output (DISO) Sugeno-style Fuzzy Inference System.
* **Inputs**: Height Error ($e$) and Pressure Differential ($\Delta P$) across valves.
* **Asymmetric Singletons**: Features custom-tuned rule matrices accounting for asymmetric charge (supply vs. spring pressure) and discharge behavior.
* Includes coasting logic to disable pulsing early and let system momentum carry height cleanly into the deadzone without overshoot.

---

## 📊 Serial Communication & Data Telemetry

### Serial Data Stream Format (CSV)
All primary working scripts stream comma-separated values (CSV) over Serial at **115200 baud**:

```csv
TIMESTAMP_MS, TARGET_H, CURRENT_H, STATUS_DUTY, P_INPUT_RAW, P_SPRING_RAW, AX, AY, AZ, GX, GY, GZ

## Telemetry & Parameters

* **`STATUS_DUTY`**: Positive values indicate filling duty cycle (Inlet ON), negative values indicate draining duty cycle (Exhaust ON), and `0` indicates hold/deadzone.
* **`P_INPUT_RAW` / `P_SPRING_RAW`**: Kalman-filtered 12-bit ADC raw readings (or converted pressure in Bar depending on firmware version).
* **`AX`–`GZ`**: Raw accelerometer and gyroscope motion vectors from MPU6050.

---

## Real-Time Commands via Serial

You can send the following character commands over the Serial interface to modify parameters live:

* `T150.0` – Update target height to 150 mm
* `P2.5` – Update Proportional gain ($K_p$)
* `I0.1` – Update Integral gain ($K_i$)
* `D0.5` – Update Derivative gain ($K_d$)

---

## 🧰 Dependencies & Libraries

Ensure the following C++ libraries are installed in your Arduino IDE or PlatformIO workspace before compilation:

* **`Wire.h`** (Built-in ESP32 I2C support)
* **VL53L0X Library** by Pololu
* **MPU6050 Library** by Electronic Cats

---

## ⚡ Getting Started

1. **Hardware Setup:** Wire your hardware according to the ESP32 Pinout Mapping table.
2. **Connect:** Connect your ESP32 to your computer via USB.
3. **Open Sketch:** Open the desired sketch (e.g., `Fuzzy_Control_Asymmetric.ino`) in the Arduino IDE.
4. **Configure IDE:** Select board **ESP32 Dev Module** and set the baud rate to `115200`.
5. **Upload:** Compile and upload the firmware to the device.
6. **Monitor & Control:** Open the Serial Monitor or connect via a custom Python Tkinter GUI to log telemetry and command target setpoints.
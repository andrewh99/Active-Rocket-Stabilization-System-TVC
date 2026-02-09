# Active-Rocket-Stabilization-System-TVC
# Teensy 4.1 Thrust Vector Control (TVC) System
**Author:** Andrew Hrisak  
**Project Type:** Aerospace Engineering Senior Capstone (Georgia Tech)

## Overview
This repository contains the flight control software for a custom-built 2-axis TVC gimbal. The system uses a **Teensy 4.1** microcontroller and an **Adafruit BNO055 IMU** to provide real-time stabilization for a model rocket motor.

## System Architecture
### Hardware
* **Controller:** Teensy 4.1 (Cortex-M7 at 600MHz)
* **IMU:** BNO055 (Absolute Orientation via Gravity Vector)
* **Actuators:** High-torque hobby servos
* **Gimbal Mechanics:** * **Lower Axis:** 120mm Gear (5.0:1 Ratio)
    * **Upper Axis:** 166mm Gear (6.91:1 Ratio)

### Control Logic
The current implementation utilizes a **Proportional (P) Feedback Loop** based on gravity vector deflection. 



**Control Law:**
$$u(t) = K_p \cdot (G_{raw} - G_{offset})$$

Where $K_p$ accounts for both the software gain and the mechanical gear reduction of the specific axis.

## Calibration Data
To account for mounting variances, the following offsets are applied to the raw Gravity Vector readings:
* **Lower Axis (Y):** 0.28
* **Upper Axis (Z):** -0.24

## Installation & Use
1. Install the `Adafruit_BNO055` and `Adafruit_Sensor` libraries via Arduino Library Manager.
2. Connect the BNO055 via I2C (Pins 18/19).
3. Connect Servos to Pins 0 and 1.
4. Upload `src/TVC_Main.ino`.
5. The system features a **10-second startup lock** to allow for safe physical inspection before reactive mode begins.

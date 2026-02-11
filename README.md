# Teensy 4.1 Thrust Vector Control (TVC) System
**Author:** Andrew Hrisak
**Project Type:** Aerospace Engineering Project | Georgia Institute of Technology

## Overview
This repository contains the flight control software and mechanical design for a custom 2-axis Thrust Vector Control (TVC) gimbal. Utilizing a **Teensy 4.1** and an **Adafruit BNO055 IMU**, the system provides active stabilization for model rocket motors by reacting to gravity vector deflection in real-time.

## Mechanical Design
![Gimbal Assembly](hardware/renders/final_render2.png)

### Specifications
* **Controller:** Teensy 4.1
* **IMU:** BNO055
* **Actuators:** EMAX ES08MA II
* **Materials:**
   * **Airframe/Structure:** PETG (15% Infill for optimized weight-to-strength ratio)
   * **Power Transmission:** PETG (100% Infill for maximum tooth shear strength and durability)
* **Gimbal Mechanics:**
   * **Axis 1 (Lower):** 120mm Gear / 24mm Pinion (**5.0:1 Ratio**)
   * **Axis 2 (Upper):** 166mm Gear / 24mm Pinion (**6.91:1 Ratio**)

## Control Theory v1.0
The system utilizes a **Proportional (P) Feedback Loop**. By leveraging the IMU's gravity vector rather than Euler angles, the controller avoids gimbal lock during high-dynamic vertical flight.

**Control Law:**
$$u(t) = K_p \cdot (G_{raw} - G_{offset})$$

Where $K_p$ incorporates the software gain (currently 8.0) and the specific mechanical advantage of each axis.

## Calibration & Implementation
To account for mounting variances, specific offsets are applied to the raw telemetry to establish a "True Vertical" baseline:
* **Lower Axis (Y) Offset:** 0.50
* **Upper Axis (Z) Offset:** -0.52

The firmware includes a **1-second startup safety lock**, holding the servos at their neutral trim positions (Low: 82.5°, Up: 95.0°) before engaging reactive mode.

Currently finalizing testing of ** Control Theory v2.0, a PID controller.

**Wiring Schematic**
![Wiring Schematic](hardware/schematics/final_schemattic.png)

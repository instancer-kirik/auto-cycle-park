# Self-Parking Motorcycle Firmware (nRF52840)

This project contains the embedded C firmware for an nRF52840 microcontroller to implement a self-parking system for a small motorcycle or scooter.

## Features

*   **Motor Control:**
    *   Drives a DC motor or BLDC (via PWM) for low-speed forward/backward motion.
    *   Direction control (e.g., via H-bridge GPIO).
*   **Gyroscope/IMU:**
    *   Reads data from an I2C-based MPU6050 IMU.
    *   Uses orientation/tilt feedback for stability.
*   **Steering Control (Selectable):**
    *   **Option A:** Pulley-wire system with a micro servo or DC motor with encoder.
    *   **Option B:** Solenoid-actuated steering lock (fixed left/right).
*   **Control Loop:**
    *   Simple PID or rule-based logic for balance and steering.
*   **Safety Features:**
    *   Emergency stop on excessive tilt.
    *   Brake GPIO output when stationary.
*   **Communication (Optional):**
    *   Bluetooth LE for control/feedback via Nordic SoftDevice.

## Hardware Assumptions

*   **Microcontroller:** nRF52840 (e.g., PCA10056 Dev Board)
*   **Motor Driver:** L298N or DRV8833
*   **IMU:** MPU6050 (I2C)
*   **Steering:**
    *   **Option A:** SG90 servo or geared DC motor with position feedback.
    *   **Option B:** Solenoid (on/off for hard left/right lock).
*   **Power Supply:** Adequate for motors + logic, with 5V regulator.

## Project Structure

```
.
├── Makefile
├── README.md
├── src/
│   ├── main.c
│   ├── motor_control.c
│   ├── motor_control.h
│   ├── imu_mpu6050.c
│   ├── imu_mpu6050.h
│   ├── steering_control.c
│   ├── steering_control.h
│   ├── control_logic.c
│   ├── control_logic.h
│   ├── safety.c
│   ├── safety.h
│   ├── ble_comm.c      // Optional
│   ├── ble_comm.h      // Optional
│   └── config/         // Hardware pin configurations, etc.
│       └── custom_board.h // Example board specific definitions
└── sdk/              // Nordic nRF5 SDK (assumed to be outside, or linked)
```

## Building the Firmware

(Instructions to be added once the build system is set up - typically involves `make` with the ARM GCC toolchain and nRF5 SDK.)

1.  **Install ARM GCC Toolchain:**
    Ensure `arm-none-eabi-gcc` and related tools are in your PATH.
2.  **Download nRF5 SDK:**
    Download the nRF5 SDK (e.g., version 17.x.x) from Nordic Semiconductor's website.
    Place it in a known location. The `Makefile` will need to point to it.
3.  **Configure Makefile:**
    Update `SDK_ROOT` in the `Makefile` to point to your nRF5 SDK installation.
    (Further board-specific configurations might be needed in `custom_board.h` or the `Makefile`.)
4.  **Build:**
    Navigate to the project directory and run:
    ```bash
    make
    ```
5.  **Flash:**
    Use `nrfjprog` (part of nRF Command Line Tools) or a J-Link programmer:
    ```bash
    make flash
    ```

## Development Notes

*   The firmware targets the Nordic nRF5 SDK.
*   Code is organized into modules for readability and maintainability.
*   Start with basic functionality (motor, IMU) and incrementally add features.
*   Thorough testing is required at each stage, especially for safety-critical functions. 
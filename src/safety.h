#ifndef SAFETY_H__
#define SAFETY_H__

#include <stdint.h>
#include <stdbool.h>

// Define safety-related thresholds (examples, to be tuned)
#define MAX_SAFE_LEAN_ANGLE_DEG     30.0f // Maximum allowable lean angle in degrees before emergency stop
#define MIN_SAFE_LEAN_ANGLE_DEG    -30.0f // Minimum allowable lean angle
#define BRAKE_ENGAGE_DELAY_MS       500  // Delay before engaging brake when stationary (if applicable)

/**
 * @brief Initializes the safety module.
 *
 * Sets up GPIO for brake control and any other safety-related hardware.
 * @param brake_pin The nRF52 pin number for brake control output (if any, e.g., to a relay or motor driver brake input).
 *                  Set to an invalid pin number (e.g., 0xFFFFFFFF) if no direct brake GPIO.
 */
void safety_init(uint32_t brake_pin);

/**
 * @brief Checks safety conditions.
 *
 * This function should be called periodically.
 * It monitors IMU tilt angles and other critical parameters.
 * If an unsafe condition is detected (e.g., excessive tilt), it triggers an emergency stop.
 * @param current_lean_angle The current lean angle of the motorcycle in degrees.
 */
void safety_check(float current_lean_angle);

/**
 * @brief Triggers an emergency stop.
 *
 * This function will immediately stop the motor, attempt to stabilize if possible (or just cut power),
 * and engage brakes if available. It should put the system into an error state.
 */
void safety_emergency_stop(void);

/**
 * @brief Engages the brake.
 *
 * Activates the brake GPIO output.
 */
void safety_engage_brake(void);

/**
 * @brief Releases the brake.
 *
 * Deactivates the brake GPIO output.
 */
void safety_release_brake(void);

/**
 * @brief Checks if the system is currently in an emergency stop state.
 *
 * @return true if an emergency stop is active, false otherwise.
 */
bool safety_is_emergency_stop_active(void);


#endif // SAFETY_H__ 
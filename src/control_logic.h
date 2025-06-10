#ifndef CONTROL_LOGIC_H__
#define CONTROL_LOGIC_H__

#include <stdint.h>
#include <stdbool.h>
#include "imu_mpu6050.h" // For imu_orientation_t

// Define system states
typedef enum {
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_BALANCING,       // Actively trying to balance
    SYSTEM_STATE_PARKING_FORWARD,
    SYSTEM_STATE_PARKING_REVERSE,
    SYSTEM_STATE_PARKED,
    SYSTEM_STATE_ERROR
} system_state_t;

// PID controller structure (if PID is used)
typedef struct {
    float kp;               // Proportional gain
    float ki;               // Integral gain
    float kd;               // Derivative gain
    float setpoint;         // Desired value (e.g., 0 degrees tilt)
    float integral;         // Integral accumulator
    float prev_error;       // Previous error for derivative calculation
    float output_min;       // Minimum output value
    float output_max;       // Maximum output value
    float integral_min;     // Minimum integral value (anti-windup)
    float integral_max;     // Maximum integral value (anti-windup)
} pid_controller_t;

/**
 * @brief Initializes the control logic module.
 *
 * Sets up PID controllers or rule-based system parameters.
 * Initializes the system state.
 */
void control_logic_init(void);

/**
 * @brief Processes the control logic.
 *
 * This function should be called periodically in the main loop.
 * It reads sensor data (IMU), applies control algorithms (PID or rules),
 * and commands the motor and steering actuators.
 * Manages system state transitions.
 */
void control_logic_process(void);

/**
 * @brief Sets the target parking parameters (e.g., distance, final orientation).
 *
 * @param target_distance_cm Distance to move (positive for forward, negative for reverse).
 * @param target_final_lean_angle Desired lean angle when parked (usually 0 or slightly leaned on a stand).
 */
void control_logic_set_parking_target(int16_t target_distance_cm, float target_final_lean_angle);

/**
 * @brief Initiates the self-parking sequence.
 */
void control_logic_start_parking(void);

/**
 * @brief Stops the self-parking sequence and attempts to stabilize.
 */
void control_logic_stop_parking(void);

/**
 * @brief Gets the current system state.
 *
 * @return The current system_state_t.
 */
system_state_t control_logic_get_system_state(void);

/**
 * @brief Updates the PID controller with a new measurement.
 *
 * @param pid Pointer to the PID controller structure.
 * @param current_value The current measured value (e.g., tilt angle).
 * @param dt Time delta since the last update (in seconds).
 * @return The calculated PID output (e.g., motor speed adjustment, steering command).
 */
float pid_controller_update(pid_controller_t *pid, float current_value, float dt);

/**
 * @brief Resets the PID controller state (e.g., integral term).
 *
 * @param pid Pointer to the PID controller structure.
 */
void pid_controller_reset(pid_controller_t *pid);

/**
 * @brief Sets new gains for a PID controller.
 *
 * @param pid Pointer to the PID controller structure.
 * @param kp New Proportional gain.
 * @param ki New Integral gain.
 * @param kd New Derivative gain.
 */
void pid_controller_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Sets the setpoint for a PID controller.
 *
 * @param pid Pointer to the PID controller structure.
 * @param setpoint New setpoint.
 */
void pid_controller_set_setpoint(pid_controller_t *pid, float setpoint);


#endif // CONTROL_LOGIC_H__ 
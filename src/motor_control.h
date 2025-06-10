#ifndef MOTOR_CONTROL_H__
#define MOTOR_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>

// Define motor direction states
typedef enum {
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_STOPPED // Or BRAKE, depending on H-bridge capability
} motor_direction_t;

/**
 * @brief Initializes the motor control module.
 *
 * Sets up PWM for speed control and GPIOs for direction control.
 * @param pwm_pin The nRF52 pin number for PWM output to the motor driver (e.g., L298N ENA/ENB).
 * @param dir_pin1 The nRF52 pin number for H-bridge input 1 (e.g., L298N IN1).
 * @param dir_pin2 The nRF52 pin number for H-bridge input 2 (e.g., L298N IN2).
 */
void motor_control_init(uint32_t pwm_pin, uint32_t dir_pin1, uint32_t dir_pin2);

/**
 * @brief Sets the motor speed and direction.
 *
 * @param speed Speed of the motor (0-100, where 0 is stopped and 100 is full speed).
 *              The actual PWM duty cycle will be scaled from this value.
 * @param direction The desired direction of the motor (MOTOR_DIR_FORWARD, MOTOR_DIR_BACKWARD).
 */
void motor_control_set_speed(uint8_t speed, motor_direction_t direction);

/**
 * @brief Stops the motor.
 *
 * This might engage a brake if the H-bridge supports it, or simply stop PWM and set direction pins to low.
 */
void motor_control_stop(void);

/**
 * @brief Engages the motor brake (if supported by hardware).
 *
 * For H-bridges like L298N, this usually involves setting both direction pins to HIGH or LOW simultaneously.
 */
void motor_control_brake(void);

#endif // MOTOR_CONTROL_H__ 
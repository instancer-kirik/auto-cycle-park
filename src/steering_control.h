#ifndef STEERING_CONTROL_H__
#define STEERING_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>

// Steering type selection (can be defined in a config file)
// #define STEERING_TYPE_SERVO
// #define STEERING_TYPE_SOLENOID

#if defined(STEERING_TYPE_SERVO)
// Configuration for Servo Steering (Option A)
#define SERVO_PWM_PIN           0 // Example PWM pin for Servo, to be configured
#define SERVO_MIN_PULSE_US      1000 // Typical min pulse width for servo (e.g., 0 degrees)
#define SERVO_MAX_PULSE_US      2000 // Typical max pulse width for servo (e.g., 180 degrees)
#define SERVO_PWM_PERIOD_US     20000 // Typical PWM period for servo (50Hz)

elif defined(STEERING_TYPE_SOLENOID)
// Configuration for Solenoid Steering (Option B)
#define SOLENOID_LEFT_PIN       0 // Example GPIO pin for Left Solenoid, to be configured
#define SOLENOID_RIGHT_PIN      0 // Example GPIO pin for Right Solenoid, to be configured

typedef enum {
    STEERING_LOCK_CENTER, // (Might not be applicable for simple solenoid, implies both off)
    STEERING_LOCK_LEFT,
    STEERING_LOCK_RIGHT
} solenoid_steering_lock_t;

#else
// Default: No steering or to be defined by user project configuration
// It's good practice to have a default or throw a compile error if no type is selected.
// For now, we'll allow it to compile and functions will be stubs if no type is defined.
#warning "Steering type not defined. Steering control functions will be stubs."
#endif

/**
 * @brief Initializes the steering control module.
 *
 * Based on the STEERING_TYPE defined (SERVO or SOLENOID), this function
 * initializes the appropriate hardware (PWM for servo, GPIOs for solenoids).
 * Pin configurations should be done in a board-specific config file or passed here.
 */
void steering_control_init(void); // Pin parameters might be added based on configuration approach

#if defined(STEERING_TYPE_SERVO)
/**
 * @brief Sets the steering angle using a servo.
 *
 * @param angle_degrees The desired steering angle in degrees. The implementation
 *                      will map this to the appropriate servo pulse width.
 *                      (e.g., -45 to +45 degrees, where 0 is center).
 */
void steering_control_set_angle_servo(int16_t angle_degrees);

/**
 * @brief Gets the current estimated steering angle (if feedback is available).
 *
 * @return The current steering angle in degrees.
 *         Note: For a simple SG90 servo without feedback, this might return the last set angle.
 */
int16_t steering_control_get_angle_servo(void);

#elif defined(STEERING_TYPE_SOLENOID)
/**
 * @brief Actuates the steering lock using solenoids.
 *
 * @param lock_position The desired lock position (STEERING_LOCK_LEFT, STEERING_LOCK_RIGHT, STEERING_LOCK_CENTER).
 */
void steering_control_set_lock_solenoid(solenoid_steering_lock_t lock_position);

/**
 * @brief Gets the current steering lock position.
 *
 * @return The current solenoid_steering_lock_t state.
 */
solenoid_steering_lock_t steering_control_get_lock_solenoid(void);

#else // No steering type defined
// Provide stub functions to avoid compilation errors if no steering type is selected.
static inline void steering_control_set_angle_servo(int16_t angle_degrees) { (void)angle_degrees; /* Stub */ }
static inline int16_t steering_control_get_angle_servo(void) { return 0; /* Stub */ }
static inline void steering_control_set_lock_solenoid(solenoid_steering_lock_t lock_position) { (void)lock_position; /* Stub */ }
static inline solenoid_steering_lock_t steering_control_get_lock_solenoid(void) { return STEERING_LOCK_CENTER; /* Stub */ }
#endif

/**
 * @brief Centers the steering (if applicable).
 * For servo, sets to 0 degrees. For solenoid, might deactivate both.
 */
void steering_control_center(void);


#endif // STEERING_CONTROL_H__ 
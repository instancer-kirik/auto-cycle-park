#ifndef CUSTOM_BOARD_H__
#define CUSTOM_BOARD_H__

// This file should contain board-specific pin definitions and configurations.
// It's included by various modules to get the correct hardware mappings.

// For example, if using PCA10056 development board, you might map functions to its pins.
// If using a custom board, define your specific pinouts here.

// --- Example Pin Definitions (adjust to your actual hardware) ---

// Motor Control Pins (L298N example)
#define MOTOR_PWM_PIN          NRF_GPIO_PIN_MAP(0, 13)  // P0.13 (Example, check nRF52840 datasheet for PWM-capable pins)
#define MOTOR_DIR_PIN1         NRF_GPIO_PIN_MAP(0, 14)  // P0.14
#define MOTOR_DIR_PIN2         NRF_GPIO_PIN_MAP(0, 15)  // P0.15

// IMU MPU6050 Pins (I2C)
#define MPU6050_TWI_INSTANCE   0                        // TWI instance (0 or 1)
#define MPU6050_SDA_PIN        NRF_GPIO_PIN_MAP(0, 26)  // P0.26 (Default TWI0 SDA on PCA10056)
#define MPU6050_SCL_PIN        NRF_GPIO_PIN_MAP(0, 27)  // P0.27 (Default TWI0 SCL on PCA10056)
#define MPU6050_INT_PIN        NRF_GPIO_PIN_MAP(0, 11)  // P0.11 (Example for MPU6050 interrupt)
#define MPU6050_I2C_ADDR       MPU6050_ADDR_AD0_LOW     // Or MPU6050_ADDR_AD0_HIGH if AD0 is high

// Steering Control Pins
// Option A: Servo
#define STEERING_TYPE_SERVO // Define which steering type is used
// #define STEERING_TYPE_SOLENOID

#ifdef STEERING_TYPE_SERVO
    #define SERVO_PWM_PIN          NRF_GPIO_PIN_MAP(0, 16)  // P0.16 (Example, ensure it's PWM capable)
    // Servo parameters are usually in steering_control.h or configured dynamically
#endif

#ifdef STEERING_TYPE_SOLENOID
    #define SOLENOID_LEFT_PIN      NRF_GPIO_PIN_MAP(0, 17)  // P0.17 (Example)
    #define SOLENOID_RIGHT_PIN     NRF_GPIO_PIN_MAP(0, 18)  // P0.18 (Example)
#endif

// Safety Pins
#define SAFETY_BRAKE_PIN       NRF_GPIO_PIN_MAP(0, 19)  // P0.19 (Example for brake control)
                                                    // Use NRF_DRV_PIN_NOT_USED if no brake pin

// LEDs (example from PCA10056, `boards.h` usually defines these based on BOARD_PCA10056)
// #define LED_1                  NRF_GPIO_PIN_MAP(0, 13) // Already used by MOTOR_PWM_PIN in this example
// #define LED_2                  NRF_GPIO_PIN_MAP(0, 14) // Already used by MOTOR_DIR_PIN1 in this example
// Ensure no pin conflicts if you define LEDs here manually.
// It is often better to rely on bsp_board_led_on/off functions which use definitions from boards.h.


// --- nRF SDK Configuration (can be in sdk_config.h or here for simplicity if small) ---
// Example: If you need to override sdk_config.h settings for a specific module.
// #define NRFX_PWM_ENABLED 1
// #define NRFX_PWM0_ENABLED 1
// #define NRFX_TWI_ENABLED 1
// #define NRFX_TWI0_ENABLED 1
// #define APP_TIMER_ENABLED 1

#endif // CUSTOM_BOARD_H__ 
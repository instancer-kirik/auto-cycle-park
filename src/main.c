#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"

#include "boards.h" // Contains board-specific definitions like LED pins

// Module headers (to be created)
#include "motor_control.h"
#include "imu_mpu6050.h"
#include "steering_control.h"
#include "control_logic.h"
#include "safety.h"
// #include "ble_comm.h" // Optional BLE communication

/**
 * @brief Function for initializing logging.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Logging initialized.");
}

/**
 * @brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    power_management_init();

    // Initialize board LEDs (example)
    bsp_board_init(BSP_INIT_LEDS);

    NRF_LOG_INFO("Self-Parking Motorcycle Firmware Started!");

    // Initialize modules
    // motor_control_init();
    // imu_init();
    // steering_control_init();
    // control_logic_init();
    // safety_init();
    // if (BLE_ENABLED) ble_stack_init(); // Example for conditional BLE init

    // Main loop.
    while (true)
    {
        // Process events, sensor readings, control loop iterations
        // control_logic_process(); 
        // safety_check();

        // Example: Blink an LED
        bsp_board_led_invert(0); // Invert LED 0 state
        nrf_delay_ms(500);

        if (NRF_LOG_PROCESS() == false) // Process log entries
        {
            nrf_pwr_mgmt_run(); // Enter sleep mode if no log entries to process
        }
    }
} 
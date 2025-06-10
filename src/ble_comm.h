#ifndef BLE_COMM_H__
#define BLE_COMM_H__

#include <stdint.h>
#include <stdbool.h>

// Forward declaration for system_state_t if ble_comm needs it and it's defined elsewhere
// Or include the relevant header e.g. "control_logic.h"
// For now, assume it might be needed for reporting status.
// #include "control_logic.h"

/**
 * @brief Initializes the Bluetooth LE stack and services.
 *
 * This includes initializing the SoftDevice, GAP parameters, GATT services/characteristics
 * for controlling the parking sequence and receiving feedback (e.g., IMU data, system state).
 */
void ble_comm_init(void);

/**
 * @brief Handles BLE events forwarded from the SoftDevice event handler.
 *
 * @param p_ble_evt Pointer to the BLE event received from the SoftDevice.
 */
void ble_comm_on_ble_evt(void * p_ble_evt); // Type might be ble_evt_t* depending on SDK version

/**
 * @brief Sends system status or sensor data over BLE.
 *
 * This function would be called periodically or when data changes to update
 * a connected BLE central device.
 *
 * @param lean_angle Current lean angle.
 * @param speed Current speed.
 * @param system_state Current system state (enum or uint8_t).
 */
void ble_comm_send_status(float lean_angle, int8_t speed, uint8_t system_state);

/**
 * @brief Function to call in the main loop to process BLE tasks, if any.
 *        For example, handling pending transmissions or updating advertising data.
 */
void ble_comm_process(void);


// Callback types for commands received via BLE (optional)

/**
 * @brief Callback function type for a start parking command received via BLE.
 */
typedef void (*ble_cmd_start_parking_handler_t)(void);

/**
 * @brief Callback function type for a stop parking command received via BLE.
 */
typedef void (*ble_cmd_stop_parking_handler_t)(void);

/**
 * @brief Registers callback functions for BLE commands.
 *
 * @param start_handler Handler for the start parking command.
 * @param stop_handler Handler for the stop parking command.
 */
void ble_comm_register_command_handlers(ble_cmd_start_parking_handler_t start_handler, 
                                        ble_cmd_stop_parking_handler_t stop_handler);


#endif // BLE_COMM_H__ 
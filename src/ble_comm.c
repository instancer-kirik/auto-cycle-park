#include "ble_comm.h"
#include "nrf_log.h"

// Include nRF SDK BLE headers (SoftDevice, GATT, GAP, etc.)
// #include "nrf_sdh.h"
// #include "nrf_sdh_ble.h"
// #include "nrf_ble_gatt.h"
// #include "ble_advdata.h"
// #include "ble_conn_params.h"
// #include "app_error.h"

// Placeholder for module implementation

// Example: Pointers to registered command handlers
// static ble_cmd_start_parking_handler_t m_start_parking_handler = NULL;
// static ble_cmd_stop_parking_handler_t  m_stop_parking_handler = NULL;

void ble_comm_init(void)
{
    // TODO: Initialize SoftDevice
    // TODO: Initialize GAP parameters (device name, appearance, preferred connection parameters)
    // TODO: Initialize GATT module
    // TODO: Initialize Advertising module
    // TODO: Define and add custom BLE Service(s) and Characteristic(s)
    //       - Characteristic for receiving commands (e.g., start/stop parking)
    //       - Characteristic for sending status (lean angle, speed, system state)
    // TODO: Initialize Connection Parameters module
    // TODO: Start advertising
    NRF_LOG_INFO("BLE communication initialized (stub).");
}

void ble_comm_on_ble_evt(void * p_ble_evt_void) // Actual type is ble_evt_t*
{
    // ble_evt_t const * p_ble_evt = (ble_evt_t const *) p_ble_evt_void;
    // TODO: Handle BLE events from the SoftDevice stack
    //       - Connection events (connected, disconnected)
    //       - GATT events (writes to characteristics for commands)
    //       - MTU updates, etc.
    // Example: if a write event occurs on the command characteristic, call the appropriate handler.
    // NRF_LOG_DEBUG("BLE event received: %d (stub).", p_ble_evt->header.evt_id);
}

void ble_comm_send_status(float lean_angle, int8_t speed, uint8_t system_state)
{
    // TODO: Check if connected
    // TODO: Format data into a buffer
    // TODO: Update the value of the status characteristic using sd_ble_gatts_hvx or sd_ble_gatts_value_set
    // NRF_LOG_DEBUG("Sending BLE status: Lean=%.1f, Speed=%d, State=%d (stub)", lean_angle, speed, system_state);
}

void ble_comm_process(void)
{
    // TODO: Any non-event-driven BLE processing (e.g., periodic updates if not done elsewhere)
}

void ble_comm_register_command_handlers(ble_cmd_start_parking_handler_t start_handler, 
                                        ble_cmd_stop_parking_handler_t stop_handler)
{
    // m_start_parking_handler = start_handler;
    // m_stop_parking_handler = stop_handler;
    NRF_LOG_INFO("BLE command handlers registered (stub).");
} 
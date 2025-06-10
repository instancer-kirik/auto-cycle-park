#include "control_logic.h"
#include "nrf_log.h"
#include "motor_control.h"
#include "steering_control.h"
#include "imu_mpu6050.h"
#include "safety.h"
#include <math.h> // For fabs, etc.

// Placeholder for module implementation
static system_state_t m_current_system_state = SYSTEM_STATE_IDLE;
static pid_controller_t m_balance_pid;
// static pid_controller_t m_steering_pid; // If used for steering

// Example PID configuration (to be tuned)
#define BALANCE_PID_KP 1.0f
#define BALANCE_PID_KI 0.1f
#define BALANCE_PID_KD 0.05f
#define BALANCE_PID_SETPOINT 0.0f // Target 0 degrees lean
#define BALANCE_PID_OUTPUT_MIN -100.0f // Corresponds to motor speed/effort
#define BALANCE_PID_OUTPUT_MAX  100.0f
#define BALANCE_PID_INTEGRAL_MIN -50.0f
#define BALANCE_PID_INTEGRAL_MAX  50.0f

// Placeholder for last update time for PID dt calculation
// static uint32_t m_last_control_update_ms = 0; // Requires app_timer or similar

void control_logic_init(void)
{
    // Initialize PID controllers
    m_balance_pid.kp = BALANCE_PID_KP;
    m_balance_pid.ki = BALANCE_PID_KI;
    m_balance_pid.kd = BALANCE_PID_KD;
    m_balance_pid.setpoint = BALANCE_PID_SETPOINT;
    m_balance_pid.integral = 0.0f;
    m_balance_pid.prev_error = 0.0f;
    m_balance_pid.output_min = BALANCE_PID_OUTPUT_MIN;
    m_balance_pid.output_max = BALANCE_PID_OUTPUT_MAX;
    m_balance_pid.integral_min = BALANCE_PID_INTEGRAL_MIN;
    m_balance_pid.integral_max = BALANCE_PID_INTEGRAL_MAX;

    // Initialize other PIDs if used

    m_current_system_state = SYSTEM_STATE_IDLE;
    NRF_LOG_INFO("Control logic initialized (stub). State: IDLE");
}

void control_logic_process(void)
{
    if (safety_is_emergency_stop_active()) {
        m_current_system_state = SYSTEM_STATE_ERROR;
        // Ensure motors are stopped, etc., handled by safety module
        return;
    }

    imu_orientation_t current_orientation;
    // bool imu_ok = imu_mpu6050_get_orientation(&current_orientation);
    // For stub, assume fixed orientation or mock data
    current_orientation.roll = 0.0f; // Example: upright
    current_orientation.pitch = 0.0f;
    bool imu_ok = true; // Assume IMU is OK for stub

    if (!imu_ok) {
        NRF_LOG_WARNING("Control_logic: Failed to get IMU data.");
        m_current_system_state = SYSTEM_STATE_ERROR;
        // Potentially trigger safety stop or enter a failsafe mode
        return;
    }

    // Call safety check with current lean angle (e.g., roll or pitch depending on mounting)
    // safety_check(current_orientation.roll); 

    // float dt = 0.01f; // Example: 10ms loop time. Actual dt should be measured.
    // uint32_t current_time_ms = app_timer_cnt_get(); // Needs app_timer
    // dt = (current_time_ms - m_last_control_update_ms) / 1000.0f;
    // m_last_control_update_ms = current_time_ms;

    switch (m_current_system_state)
    {
        case SYSTEM_STATE_IDLE:
            // Do nothing, wait for commands (e.g., start_parking)
            motor_control_stop();
            break;
        case SYSTEM_STATE_BALANCING:
            // float balance_output = pid_controller_update(&m_balance_pid, current_orientation.roll, dt);
            // motor_control_set_speed(abs(balance_output), balance_output > 0 ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD);
            break;
        case SYSTEM_STATE_PARKING_FORWARD:
        case SYSTEM_STATE_PARKING_REVERSE:
            // Implement parking logic: combine balancing with movement towards target
            // Adjust steering as needed
            break;
        case SYSTEM_STATE_PARKED:
            motor_control_stop();
            safety_engage_brake(); // Engage brake when parked
            break;
        case SYSTEM_STATE_ERROR:
            // Error state, motors should be stopped by safety module
            // Wait for reset or intervention
            break;
        default:
            m_current_system_state = SYSTEM_STATE_IDLE;
            break;
    }
    NRF_LOG_DEBUG("Control logic processed (stub). State: %d", m_current_system_state);
}

void control_logic_set_parking_target(int16_t target_distance_cm, float target_final_lean_angle)
{
    NRF_LOG_INFO("Parking target set: dist %d cm, lean %f deg (stub).", target_distance_cm, target_final_lean_angle);
    // Store these targets for use in parking states
}

void control_logic_start_parking(void)
{
    if (m_current_system_state == SYSTEM_STATE_IDLE || m_current_system_state == SYSTEM_STATE_PARKED) {
        pid_controller_reset(&m_balance_pid);
        // pid_controller_reset(&m_steering_pid);
        safety_release_brake();
        m_current_system_state = SYSTEM_STATE_BALANCING; // Or directly to a parking state
        NRF_LOG_INFO("Parking sequence started (stub). State: BALANCING");
    } else {
        NRF_LOG_WARNING("Start parking command ignored, system not IDLE/PARKED.");
    }
}

void control_logic_stop_parking(void)
{
    m_current_system_state = SYSTEM_STATE_IDLE;
    motor_control_stop();
    safety_engage_brake(); // Engage brake on stop
    NRF_LOG_INFO("Parking sequence stopped (stub). State: IDLE");
}

system_state_t control_logic_get_system_state(void)
{
    return m_current_system_state;
}

float pid_controller_update(pid_controller_t *pid, float current_value, float dt)
{
    if (!pid || dt <= 0) return 0.0f;

    float error = pid->setpoint - current_value;

    // Proportional term
    float p_term = pid->kp * error;

    // Integral term (with anti-windup)
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;
    float i_term = pid->ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    // Update previous error
    pid->prev_error = error;

    // Compute output
    float output = p_term + i_term + d_term;

    // Clamp output to min/max
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    return output;
}

void pid_controller_reset(pid_controller_t *pid)
{
    if (pid) {
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
    }
}

void pid_controller_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    if (pid) {
        pid->kp = kp;
        pid->ki = ki;
        pid->kd = kd;
    }
}

void pid_controller_set_setpoint(pid_controller_t *pid, float setpoint)
{
    if (pid) {
        pid->setpoint = setpoint;
    }
} 
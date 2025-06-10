#ifndef IMU_MPU6050_H__
#define IMU_MPU6050_H__

#include <stdint.h>
#include <stdbool.h>

// MPU6050 I2C Address (can be AD0_LOW or AD0_HIGH)
#define MPU6050_ADDR_AD0_LOW     0x68 // Default address
#define MPU6050_ADDR_AD0_HIGH    0x69

// Structure to hold accelerometer, gyroscope, and temperature data
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} mpu6050_data_t;

// Structure to hold calculated orientation (e.g., roll, pitch)
typedef struct {
    float roll;
    float pitch;
    float yaw; // Yaw might be less reliable without a magnetometer
} imu_orientation_t;

/**
 * @brief Initializes the MPU6050 IMU.
 *
 * Configures I2C communication and sets up the MPU6050 sensor (e.g., sample rate, DLPF).
 * @param i2c_instance_idx The index of the TWI (I2C) peripheral to use (e.g., 0 or 1 for NRF_DRV_TWI_INSTANCE(0) or NRF_DRV_TWI_INSTANCE(1)).
 * @param sda_pin The nRF52 pin number for I2C SDA.
 * @param scl_pin The nRF52 pin number for I2C SCL.
 * @param mpu6050_addr The I2C address of the MPU6050.
 * @return true if initialization was successful, false otherwise.
 */
bool imu_mpu6050_init(uint8_t i2c_instance_idx, uint32_t sda_pin, uint32_t scl_pin, uint8_t mpu6050_addr);

/**
 * @brief Reads raw accelerometer, gyroscope, and temperature data from the MPU6050.
 *
 * @param[out] p_data Pointer to an mpu6050_data_t struct to store the read data.
 * @return true if data was read successfully, false otherwise.
 */
bool imu_mpu6050_read_raw_data(mpu6050_data_t *p_data);

/**
 * @brief Reads and processes IMU data to calculate orientation (roll, pitch).
 *
 * This function might implement a simple complementary filter or just basic angle calculation
 * from accelerometer data for static/low-dynamic tilt.
 * More advanced sensor fusion (Kalman filter, Madgwick/Mahony) can be implemented here or in control_logic.
 *
 * @param[out] p_orientation Pointer to an imu_orientation_t struct to store the calculated orientation.
 * @return true if orientation was calculated successfully, false otherwise.
 */
bool imu_mpu6050_get_orientation(imu_orientation_t *p_orientation);

/**
 * @brief Performs a self-test on the MPU6050.
 *
 * @return true if self-test passed, false otherwise.
 */
bool imu_mpu6050_self_test(void);

/**
 * @brief Calibrates gyroscope offsets.
 *        The device should be stationary during calibration.
 * @param num_samples Number of samples to average for offset calculation.
 * @return true if calibration was successful, false otherwise.
 */
bool imu_mpu6050_calibrate_gyro(uint16_t num_samples);


/** @brief Converts raw accelerometer value to G's.
 *  Assumes a default sensitivity of +/- 2g (FS_SEL=0).
 */
float imu_mpu6050_accel_to_g(int16_t accel_raw);

/** @brief Converts raw gyroscope value to degrees/sec.
 *  Assumes a default sensitivity of +/- 250 deg/s (FS_SEL=0).
 */
float imu_mpu6050_gyro_to_dps(int16_t gyro_raw);


#endif // IMU_MPU6050_H__ 
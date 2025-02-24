/*
 * lsm6dsl.c
 *
 *  Created on: Feb 5, 2025
 *      Author: gisellemendoza
 */

#include "lsm6dsl.h"
#include "i2c.h"
#include <stdio.h>

uint8_t who_am_i = LSM6DSL_WHO_AM_I;

void lsm6dsl_init() {
    uint8_t return_value = 0;
    uint8_t check_value = 0;

    // Send WHO_AM_I register address
    return_value = i2c_transaction(LSM6DSL_I2C_ADDR, 0, &who_am_i, 1);
    if (return_value != 1) {
        printf("WHO_AM_I send failed\n");
        return;
    }

    // Read WHO_AM_I register value
    return_value = i2c_transaction(LSM6DSL_I2C_ADDR, 1, &check_value, 1);
    if (return_value != 1) {
        printf("WHO_AM_I receive failed\n");
        return;
    }

    printf("WHO_AM_I: 0x%X\n", check_value);

    uint8_t config_data[2];  // Buffer to store register address and value
    uint8_t ret;             // Variable to store return status of I2C transactions

    // Configure accelerometer control register (CTRL1_XL)
    config_data[0] = LSM6DSL_CTRL1_XL;  // Register address
    config_data[1] = CTRL1_XL_CONFIG;   // Configuration value

    // Send configuration to the LSM6DSL sensor via I2C
    ret = i2c_transaction(LSM6DSL_I2C_ADDR, 0, config_data, 2);
    if (ret != 1) {
        printf("Accelerometer configuration failed\n");
        return;
    }

    // Configure another register (CTRL3_C)
    config_data[0] = LSM6DSL_CTRL3_C;  // Register address
    config_data[1] = CTRL3_C_CONFIG;   // Configuration value

    // Send second configuration to the sensor
    ret = i2c_transaction(LSM6DSL_I2C_ADDR, 0, config_data, 2);
    if (ret != 1) {
        printf("CTRL3_C configuration failed\n");
        return;
    }
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t reg_address = LSM6DSL_OUTX_L_XL;
    uint8_t data[6];  // Buffer for raw acceleration data

    // Step 1: Write register address and read 6 bytes in one transaction
    if (i2c_transaction(LSM6DSL_I2C_ADDR, 0, &reg_address, 1) == 0) {
        printf("Failed to write register address\n");
        return;
    }

    if (i2c_transaction(LSM6DSL_I2C_ADDR, 1, data, 6) == 0) {
        printf("Failed to read accelerometer data\n");
        return;
    }

    // Step 2: Combine LSB and MSB into signed 16-bit integers
    *x = (int16_t)((data[1] << 8) | data[0]) * SENS2G;
    *y = (int16_t)((data[3] << 8) | data[2]) * SENS2G;
    *z = (int16_t)((data[5] << 8) | data[4]) * SENS2G;
}

/*
 * MIT License
 * 
 * Copyright (c) 2025 Pavlo Romaniuk
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * provided to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "i2c.h"

// BMP280 Error Codes
#define BMP280_OK               0x00  // No error: The operation was successful.
#define BMP280_ERR_NOTFOUND     0x01  // Error: BMP280 sensor not responding on the I2C bus.
#define BMP280_ERR_MEASUREMENT  0x03  // Error: Measurement failure. The sensor failed to obtain data correctly.

#define BMP280_ADDR 0x77  // I2C Address of the BMP280 sensor

// Data types for handling temperature and pressure values
typedef int32_t BMP280_S32_t;  // 32-bit signed integer type for temperature values
typedef uint32_t BMP280_U32_t; // 32-bit unsigned integer type for pressure values
typedef int64_t BMP280_S64_t;  // 64-bit signed integer type for intermediate pressure calculations

// Structure to hold the calibration data of the BMP280 sensor
typedef struct {
    uint16_t dig_T1; // Temperature coefficient 1
    int16_t dig_T2;  // Temperature coefficient 2
    int16_t dig_T3;  // Temperature coefficient 3

    uint16_t dig_P1; // Pressure coefficient 1
    int16_t dig_P2;  // Pressure coefficient 2
    int16_t dig_P3;  // Pressure coefficient 3
    int16_t dig_P4;  // Pressure coefficient 4
    int16_t dig_P5;  // Pressure coefficient 5
    int16_t dig_P6;  // Pressure coefficient 6
    int16_t dig_P7;  // Pressure coefficient 7
    int16_t dig_P8;  // Pressure coefficient 8
    int16_t dig_P9;  // Pressure coefficient 9
} BMP280CalibrationData;

// Structure to store the compensated data from the BMP280 sensor
typedef struct {
    int32_t temperature; // Compensated temperature in hundredths of degrees Celsius (e.g., 2234 means 22.34°C)
    uint32_t pressure;   // Compensated pressure in Pa (Pascal)
} BMP280Data;

// Global variable to store the fine temperature value (t_fine), used for pressure compensation
BMP280_S32_t t_fine;

/**
 * @brief  Compensates the raw temperature reading using the calibration data.
 * 
 * This function applies the compensation formula for temperature based on the raw sensor value.
 * It uses the calibration coefficients stored in `BMP280CalibrationData` to compute the
 * temperature in degrees Celsius.
 *
 * @param adc_T Raw temperature reading from the sensor.
 * @param cd Pointer to the calibration data structure.
 * @return BMP280_S32_t The compensated temperature value in hundredths of degrees Celsius.
 */
BMP280_S32_t BMP280_bmp280_compensate_T_int32(BMP280_S32_t adc_T, BMP280CalibrationData *cd) {
    BMP280_S32_t var1, var2, T;  // Temporary variables for intermediate calculations

    // Apply the first part of the temperature compensation formula (var1)
    var1 = ((((adc_T >> 3) - ((BMP280_S32_t)cd->dig_T1 << 1))) * ((BMP280_S32_t)cd->dig_T2)) >> 11;

    // Apply the second part of the temperature compensation formula (var2)
    var2 = (((((adc_T >> 4) - ((BMP280_S32_t)cd->dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t)cd->dig_T1))) >> 12) * ((BMP280_S32_t)cd->dig_T3)) >> 14;

    // Calculate the fine temperature value (t_fine)
    t_fine = var1 + var2;

    // Compute the final temperature (T) in degrees Celsius and return
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

/**
 * @brief  Compensates the raw pressure reading using the temperature and calibration data.
 * 
 * This function applies the compensation formula for pressure based on the raw sensor value and 
 * the fine temperature value (t_fine) from the temperature compensation.
 * It uses the calibration coefficients stored in `BMP280CalibrationData` to compute the 
 * pressure in Pascal (Pa).
 *
 * @param adc_P Raw pressure reading from the sensor.
 * @param cd Pointer to the calibration data structure.
 * @return BMP280_U32_t The compensated pressure value in Pa.
 */
BMP280_U32_t BMP280_bmp280_compensate_P_int64(BMP280_S32_t adc_P, BMP280CalibrationData *cd) {
    BMP280_S64_t var1, var2, p;  // Temporary variables for intermediate calculations

    // Apply compensation formulas based on the fine temperature value (t_fine)
    var1 = ((BMP280_S64_t)t_fine) - 128000;
    var2 = var1 * var1 * (BMP280_S64_t)cd->dig_P6;
    var2 = var2 + ((var1 * (BMP280_S64_t)cd->dig_P5) << 17);
    var2 = var2 + (((BMP280_S64_t)cd->dig_P4) << 35);

    // Further calculations for var1 based on calibration coefficients
    var1 = ((var1 * var1 * (BMP280_S64_t)cd->dig_P3) >> 8) + ((var1 * (BMP280_S64_t)cd->dig_P2) << 12);
    var1 = (((((BMP280_S64_t)1) << 47) + var1) * (BMP280_S64_t)cd->dig_P1) >> 33;

    // Avoid division by zero if var1 is zero
    if (var1 == 0) {
        return 0;  // Return 0 to prevent division by zero errors
    }

    // Calculate the pressure in Pascal (Pa)
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    // Apply final compensation for pressure
    var1 = (((BMP280_S64_t)cd->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((BMP280_S64_t)cd->dig_P8) * p) >> 19;

    // Final pressure calculation and return in Q24.8 format (Pa)
    p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)cd->dig_P7) << 4);
    return (BMP280_U32_t)p;
}

/**
 * @brief  Reads temperature and pressure data from the BMP280 sensor and applies compensation.
 * 
 * This function communicates with the BMP280 sensor over I2C to fetch the raw temperature and pressure
 * data. It then applies the compensation formulas using the calibration data to obtain the corrected values
 * for temperature (in Celsius) and pressure (in Pascal).
 * 
 * @param data Pointer to the BMP280Data structure where the compensated temperature and pressure will be stored.
 * @return uint8_t Status code indicating success (BMP280_OK) or error (BMP280_ERR_NOTFOUND, BMP280_ERR_MEASUREMENT).
 */
uint8_t BMP280_read(BMP280Data *data) {
    uint8_t buf[8] = {0};  // Buffer to store sensor data
    uint8_t reg;           // Register address for reading data
    uint8_t err = BMP280_OK;  // Error flag (initialized to no error)
    BMP280CalibrationData cd;  // Structure to hold calibration data

    // Prepare control register settings for pressure and temperature measurement (with 8x oversampling)
    buf[0] = 0xf4; // Control register address
    buf[1] = 0x92; // Pressure and temperature measurement mode

    // Start I2C communication and send the control register settings to the sensor
    if(i2c_begin_transmisison(BMP280_ADDR, I2C_MODE_WRITE) != BMP280_OK) {
        i2c_end_transmisison();
        return BMP280_ERR_NOTFOUND;  // Sensor not found on the I2C bus
    }
    err |= i2c_transmit_data(&buf, 2);  // Send control register data
    i2c_end_transmisison();

    // Wait for the sensor to take the measurement
    Delay_Ms(100); // Minimal delay of 100ms for accuracy (25ms is the minimum, but 100ms is safer for oversampling)

    // Read the raw pressure data
    reg = 0xf7;  // Register address for pressure data
    err |= i2c_begin_transmisison(BMP280_ADDR, I2C_MODE_WRITE);
    err |= i2c_transmit_data(&reg, 1);  // Send register address to read
    i2c_end_transmisison();
    err |= i2c_begin_transmisison(BMP280_ADDR, I2C_MODE_READ);
    err |= i2c_receive_data(buf, 8);  // Receive 8 bytes of data (temperature and pressure)
    i2c_end_transmisison();

    if(err != I2C_OK) return err;  // Return if an I2C error occurred

    // Merge the received bytes into 20-bit pressure data
    BMP280_S32_t pressure = 0;
    pressure |= buf[0] << 12;
    pressure |= buf[1] << 4;
    pressure |= (buf[2] >> 4) & 0x0f;

    // Merge the received bytes into 20-bit temperature data
    BMP280_S32_t temperature = 0;
    temperature |= buf[3] << 12;
    temperature |= buf[4] << 4;
    temperature |= (buf[5] >> 4) & 0x0f;

    // Read the calibration data from the sensor
    reg = 0x88;  // Register address for calibration data
    err |= i2c_begin_transmisison(BMP280_ADDR, I2C_MODE_WRITE);
    err |= i2c_transmit_data(&reg, 1);  // Send register address to read calibration data
    i2c_end_transmisison();
    err |= i2c_begin_transmisison(BMP280_ADDR, I2C_MODE_READ);
    err |= i2c_receive_data(&cd, sizeof(cd));  // Receive 24 bytes of calibration data
    i2c_end_transmisison();

    if(err != I2C_OK) return err;  // Return if an I2C error occurred

    // Apply compensation to the raw data and store the results in the `data` structure
    data->temperature = BMP280_bmp280_compensate_T_int32(temperature, &cd);  // Compensate temperature
    data->pressure = BMP280_bmp280_compensate_P_int64(pressure, &cd) >> 8;  // Compensate pressure (in Pa)

    return BMP280_OK;  // Return success
}

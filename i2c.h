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

#ifndef I2C_H
#define I2C_H

// I2C Configuration
// I2C Configuration Defaults

//#define I2C_DEBUG    // uncomment to echo all I2C traffic to printf

// Default AFIO Register Value
#define I2C_AFIO_REG     ((uint32_t)0x00000000)  // Default AFIO configuration (no remapping)

// Default I2C Clock Frequency Configuration
#define I2C_CLK_FREQ     0x08  // 8 MHz I2C module clock frequency setting. This value should be between 001000b and 110000b in MHz.

// Default I2C Clock Rate (frequency in Hz)
#define I2C_CLK_RATE     400000  // 400 KHz I2C clock frequency in Hz. Default is 400 KHz

// Default I2C GPIO Port
#define I2C_PORT         GPIOC  // GPIOC as the default I2C port

// Default RCC (Reset and Clock Control) for I2C GPIO Port
#define I2C_PORT_RCC     RCC_APB2Periph_GPIOC  // GPIOC clock for I2C

// Default SDA Pin (Serial Data Line)
#define I2C_PIN_SDA      1  // Default SDA Pin is 1

// Default SCL Pin (Serial Clock Line)
#define I2C_PIN_SCL      2  // Default SCL Pin is 2

// Default I2C Timeout Value (in cycles)
#define I2C_TIMEOUT      3000  // Timeout in n retry cycler

// I2C Modes
#define I2C_MODE_READ    0
#define I2C_MODE_WRITE   1

// I2C Error Codes
#define I2C_OK           0
#define I2C_ERR_BUSY     1   // I2C bus is busy
#define I2C_ERR_MASTER   2   // Error starting master mode on I2C bus
#define I2C_ERR_ADDR     3   // Error setting address
#define I2C_ERR_TIMEOUT  4   // I2C timeout
#define I2C_ERR_BERR     5   // I2C bus error

// I2C Timeout and Configuration
#define I2C_TIMEOUT      300
#define I2C_CLK_FREQ     0x08 // 8MHz I2C module CLK (set in MHz)

// I2C function prototypes
uint8_t i2c_init(void);
uint8_t i2c_begin_transmisison(uint8_t addr, uint8_t mode);
void i2c_end_transmisison();
uint8_t i2c_transmit_data(uint8_t *data, uint16_t length);
uint8_t i2c_receive_data(uint8_t *data, uint16_t length);
uint8_t i2c_deinit();

#endif // I2C_H
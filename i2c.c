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


#include "ch32fun.h"
#include "i2c.h"


/**
 * @brief  Begins an I2C transmission, either for reading or writing to a specific address.
 * 
 * This function initializes the I2C transmission by starting the master mode and setting up 
 * the address and transmission direction (read or write). It handles timeouts during each step
 * to ensure the operation is completed successfully.
 *
 * @param addr     The I2C address of the slave device (7-bit address).
 * @param transmit Specifies the direction of the transmission:
 *                 - `I2C_MODE_READ`: Read operation.
 *                 - `I2C_MODE_WRITE`: Write operation.
 * 
 * @return uint8_t 
 *   - `I2C_OK` (0): Transmission started successfully.
 *   - `I2C_ERR_BUSY` (1): Timeout waiting for the bus to become available.
 *   - `I2C_ERR_MASTER` (2): Timeout while asserting master mode.
 *   - `I2C_ERR_TIMEOUT` (4): Timeout during the data transmission initialization.
 * 
 * @note   The function handles setting up the master mode, clearing any previous errors, 
 *         and checking flags at each step to ensure proper operation.
 */
uint8_t i2c_begin_transmisison(uint8_t addr, uint8_t mode) {
    // Wait for the I2C bus to be free, with timeout to prevent hanging
    int32_t timeout = I2C_TIMEOUT;
    while (I2C1->STAR2 & I2C_STAR2_BUSY) { 
        if (--timeout < 0) {
            #ifdef I2C_DEBUG
            printf("I2C Timeout waiting bus\n");
            #endif
            return I2C_ERR_BUSY;
        }
    }

    // Clear Acknowledge failure (AF) bit if set from previous operation
    I2C1->STAR1 &= ~I2C_STAR1_AF;

    // Generate a START condition on the I2C bus
    I2C1->CTLR1 |= I2C_CTLR1_START;
    timeout = I2C_TIMEOUT;

    // Wait for the start condition to be acknowledged by checking flags
    while ((I2C1->STAR1 != 0x0001) || (I2C1->STAR2 != 0x0003)) { // BUSY, MSL, SB flags
        if (--timeout < 0) {
            #ifdef I2C_DEBUG
            printf("I2C start timeout\n");
            #endif
            return I2C_ERR_MASTER;
        }
    }

    // Send the 7-bit address with write bit (0) or read bit (1)
    I2C1->DATAR = (addr << 1) & 0xFE;  // Address | Write

    // Wait for transmission to be initialized, checking flags again
    timeout = I2C_TIMEOUT;
    while ((I2C1->STAR1 != 0x0082) || (I2C1->STAR2 != 0x0007)) { // BUSY, MSL, SB flags
        if (--timeout < 0) {
            #ifdef I2C_DEBUG
            printf("I2C transmit init timeout\n");
            #endif
            return I2C_ERR_TIMEOUT;
        }
    }

    // If a read operation is requested, set the read flag and configure accordingly
    if (mode == I2C_MODE_READ) {
        I2C1->CTLR1 |= I2C_CTLR1_ACK;  // Enable ACK for receiving

        // Generate start condition in receiver mode
        I2C1->CTLR1 |= I2C_CTLR1_START;
        timeout = I2C_TIMEOUT;

        // Wait for the master mode and slave address match, indicating readiness to receive
        while ((I2C1->STAR1 != 0x0001) || (I2C1->STAR2 != 0x0003)) { // BUSY, MSL, SB flags
            if (--timeout < 0) {
                #ifdef I2C_DEBUG
                printf("I2C start in receiver mode timeout\n");
                #endif
                return I2C_ERR_TIMEOUT;
            }
        }

        // Send the 7-bit address with read bit set (1)
        I2C1->DATAR = (addr << 1) | 0x01;  // Address | Read

        // Wait for acknowledgment of the address, master mode with receive ready flags
        timeout = I2C_TIMEOUT;
        while ((I2C1->STAR1 != 0x0002) || (I2C1->STAR2 != 0x0003)) { // BUSY, MSL, SB flags
            if (--timeout < 0) {
                #ifdef I2C_DEBUG
                printf("Receive init timeout\n");
                #endif
                return I2C_ERR_TIMEOUT;
            }
        }
    }

    return I2C_OK;  // Success
}

/**
 * @brief  Ends the current I2C transmission by generating a STOP condition.
 * 
 * This function generates a STOP condition on the I2C bus to indicate the end of a transmission.
 * It also clears any pending errors (e.g., acknowledge failure or arbitration loss) to prepare 
 * for the next transmission.
 */
void i2c_end_transmisison() {
    // Generate a STOP condition on the I2C bus
    I2C1->CTLR1 |= I2C_CTLR1_STOP;

    // Clear Acknowledge failure (AF) and Arbitration Loss (ARLO) bits to avoid issues in future operations
    I2C1->STAR1 &= ~I2C_STAR1_AF;
    I2C1->STAR1 &= ~I2C_STAR1_ARLO;
}


/**
 * @brief  Transmits a sequence of data bytes to the I2C bus.
 * 
 * This function writes multiple bytes of data to the I2C bus. Each byte is sent sequentially, 
 * and the function waits for the data register to be empty before sending the next byte. 
 * It supports error checking with a timeout for each byte transmission.
 * 
 * @param data     Pointer to the data buffer to transmit.
 * @param length   The number of bytes to transmit.
 * 
 * @return uint8_t 
 *   - `I2C_OK` (0): All data sent successfully.
 *   - `I2C_ERR_TIMEOUT` (4): Timeout while waiting for data to be transmitted.
 * 
 * @note   Debugging messages will be printed for each byte sent if `I2C_DEBUG` is defined.
 */
uint8_t i2c_transmit_data(uint8_t *data, uint16_t length) {
    int16_t timeout = I2C_TIMEOUT;

    // Transmit each byte in the data buffer
    for (uint16_t cnt = 0; cnt < length; cnt++) {
        I2C1->DATAR = *data;  // Write the byte to the data register

        // Wait for the data to be transmitted (TXE flag)
        while (!(I2C1->STAR1 & I2C_STAR1_TXE)) {
            if (--timeout < 0) {
                #ifdef I2C_DEBUG
                printf("Send timeout\n");
                #endif
                return I2C_ERR_TIMEOUT;
            }
        }

        #ifdef I2C_DEBUG
        printf("Byte sent: 0x%02X\n", *data);  // Debug message
        #endif

        data++;  // Move to the next byte in the buffer
    }

    return I2C_OK;  // Success
}

/**
 * @brief  Receives a sequence of data bytes from the I2C bus.
 * 
 * This function reads multiple bytes of data from the I2C bus. It waits for each byte to be received,
 * and based on the current byte's position, it either sends an ACK (for more bytes) or a NACK (for the last byte).
 * The received data is stored in the provided buffer.
 * 
 * @param data     Pointer to the buffer where received data will be stored.
 * @param length   The number of bytes to receive.
 * 
 * @return uint8_t 
 *   - `I2C_OK` (0): All data received successfully.
 *   - `I2C_ERR_TIMEOUT` (4): Timeout while waiting for data to be received.
 * 
 * @note   Debugging messages will be printed for each byte received if `I2C_DEBUG` is defined.
 */
uint8_t i2c_receive_data(uint8_t *data, uint16_t length) {
    uint16_t timeout = I2C_TIMEOUT;

    // Receive each byte in the data buffer
    for (uint16_t cnt = 0; cnt < length; cnt++) {
        // Set NACK for the last byte, ACK for all others
        if (cnt == (length - 1)) {
            I2C1->CTLR1 &= ~I2C_CTLR1_ACK;  // Disable ACK for the last byte
        } else {
            I2C1->CTLR1 |= I2C_CTLR1_ACK;   // Enable ACK for remaining bytes
        }

        // Wait for data to be received (RXNE flag)
        while (!(I2C1->STAR1 & I2C_STAR1_RXNE)) {
            if (--timeout < 0) {
                #ifdef I2C_DEBUG
                printf("Receive byte timeout\n");
                #endif
                return I2C_ERR_TIMEOUT;
            }
        }

        *data = I2C1->DATAR;  // Read received byte from the data register

        #ifdef I2C_DEBUG
        printf("I2C data received: 0x%02x\n", *data);  // Debug message
        #endif

        data++;  // Move to the next byte in the buffer
    }

    return I2C_OK;  // Success
}

/**
 * @brief  Initializes the I2C peripheral, including resetting registers, configuring the clock,
 *         setting up GPIO pins for SCL and SDA, and enabling the I2C peripheral.
 * 
 * @return uint8_t 
 *   - I2C_OK (0): Initialization successful.
 *   - I2C_ERR_BERR (5): Bus error during initialization.
 * 
 * @note   This function performs several hardware-specific operations:
 *         - Toggles I2C reset to initialize registers.
 *         - Configures the GPIO pins for I2C communication.
 *         - Configures the I2C clock frequency.
 *         - Enables the I2C peripheral.
 *         It is assumed that the system core clock is set correctly.
 */

uint8_t i2c_init()
{
	// Toggle the I2C Reset bit to init Registers
	RCC->APB1PRSTR |=  RCC_APB1Periph_I2C1;
	RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1;

	// Enable the I2C Peripheral Clock
	RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

	// Enable the selected I2C Port, and the Alternate Function enable bit
	RCC->APB2PCENR |= I2C_PORT_RCC | RCC_APB2Periph_AFIO;

	// Reset the AFIO_PCFR1 register, then set it up
	AFIO->PCFR1 &= ~(0x04400002);
	AFIO->PCFR1 |= I2C_AFIO_REG;

	// Clear, then set the GPIO Settings for SCL and SDA, on the selected port
	I2C_PORT->CFGLR &= ~(0x0F << (4 * I2C_PIN_SDA));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * I2C_PIN_SDA);	
	I2C_PORT->CFGLR &= ~(0x0F << (4 * I2C_PIN_SCL));
	I2C_PORT->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * I2C_PIN_SCL);

	
    // Set I2C Control Register 2 (CTLR2) to configure the frequency
    I2C1->CTLR2 &= ~I2C_CTLR2_FREQ;  // Clear previous frequency configuration
    I2C1->CTLR2 |= (I2C_CLK_FREQ & I2C_CTLR2_FREQ);  // Set new frequency

    // Set I2C Clock based on the clock rate
    if (I2C_CLK_RATE <= 100000) {
        // Standard mode (100 KHz or below)
        I2C1->CKCFGR &= ~I2C_CKCFGR_CCR;  // Clear previous CCR settings
        I2C1->CKCFGR |= (FUNCONF_SYSTEM_CORE_CLOCK / (2 * I2C_CLK_RATE)) & I2C_CKCFGR_CCR;  // Set new CCR value for standard mode
    } else {
        // Fast mode (above 100 KHz)
        I2C1->CKCFGR &= ~I2C_CKCFGR_CCR;  // Clear previous CCR settings
        I2C1->CKCFGR |= (FUNCONF_SYSTEM_CORE_CLOCK / (3 * I2C_CLK_RATE)) & I2C_CKCFGR_CCR;  // Set new CCR value for fast mode
        I2C1->CKCFGR |= I2C_CKCFGR_FS;  // Enable fast mode (33% duty cycle)
    }

	// Enable the I2C Peripheral
	I2C1->CTLR1 |= I2C_CTLR1_PE;

	//TODO:
	// Check error states
	if(I2C1->STAR1 & I2C_STAR1_BERR) 
	{
		I2C1->STAR1 &= ~(I2C_STAR1_BERR); 
		return I2C_ERR_BERR;
	}

	return I2C_OK;
}


/**
 * @brief  De-initializes the I2C peripheral, including disabling the I2C peripheral,
 *         resetting the associated GPIO pins, and turning off the clocks.
 * 
 * This function shuts down the I2C interface, releases the I2C pins, and powers down
 * the I2C peripheral and its clock. It also ensures the GPIO pins are reset to a safe state.
 * 
 * @return uint8_t 
 *   - I2C_OK (0): De-initialization successful.
 *   - I2C_ERR_RESET (1): Error during I2C reset process.
 *   - I2C_ERR_CLK_DISABLE (2): Error disabling I2C clock.
 */
uint8_t i2c_deinit() {
    // Disable the I2C peripheral (Set I2C1 PE bit to 0)
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;  // Disable I2C peripheral

    // Reset the I2C1 peripheral to its default state (soft reset)
    RCC->APB1PRSTR |= RCC_APB1Periph_I2C1;  // Set the I2C reset bit
    RCC->APB1PRSTR &= ~RCC_APB1Periph_I2C1; // Clear the reset bit to release reset

    // Disable the I2C clock (turn off I2C peripheral clock)
    RCC->APB1PCENR &= ~RCC_APB1Periph_I2C1;  // Disable I2C clock

    // Reset the AFIO settings for the I2C pins (SDA and SCL)
    AFIO->PCFR1 &= ~(0x04400002); // Reset the alternate function settings for SDA and SCL

    // Reset the GPIO configuration for SDA and SCL pins
    I2C_PORT->CFGLR &= ~(GPIO_CNF_IN_PUPD<< (4 * I2C_PIN_SDA));  // Clear the configuration for SDA pin
    I2C_PORT->CFGLR &= ~(GPIO_CNF_IN_PUPD << (4 * I2C_PIN_SCL));  // Clear the configuration for SCL pin

    return I2C_OK;  // De-initialization was successful
}
/*************************************************************************
*  Title:    I2C simple library for AVR-GCC
*  Author:   Dominik Chalupka
*  File:     I2C.c
*  Software: PlatformIO
*  Hardware: Arduino Uno
*  License:  MIT
**************************************************************************/
#include <avr/io.h>
#include "i2c.h"


void I2C_init() {
    // Set SCL frequency: SCL = F_CPU / (16 + 2 * TWBR * Prescaler)
    TWSR = 0x00;  // Set prescaler to 1
    TWBR = I2C_BAUDRATE;  // Calculate TWBR value
}

void I2C_start() {
    // Send start condition
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));  // Wait for transmission
}

void I2C_stop() {
    // Send stop condition
    TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

uint8_t I2C_read(uint8_t ack) {
    if (ack == I2C_ACK) {
        TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);  // Send ACK
    } else {
        TWCR = (1 << TWEN) | (1 << TWINT);  // Send NACK
    }
    while (!(TWCR & (1 << TWINT))); // Wait for transmission
    return TWDR;
}

uint8_t I2C_write(uint8_t data) {
    uint8_t status;

    TWDR = data;  // Load data
    TWCR = (1 << TWEN) | (1 << TWINT); //enable TWI and clear interrupt flag
    while (!(TWCR & (1 << TWINT)));  // Wait for transmission

    status = TWSR & 0xF8;   // Mask status

    if (status == 0x18 || status == 0x28 || status == 0x40) {
        return 0;  // ACK received
    } else {
        return 1;  // NACK received
    }

}

//
uint8_t I2C_read_ack() {
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);  // Send ACK
    while (!(TWCR & (1 << TWINT))); // Wait for transmission
    return TWDR;
}
//
uint8_t I2C_read_nack() {
    TWCR = (1 << TWEN) | (1 << TWINT);  // Send NACK
    while (!(TWCR & (1 << TWINT))); // Wait for transmission
    return TWDR;
}
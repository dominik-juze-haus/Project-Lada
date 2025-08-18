#ifndef I2C_h
#define I2C_h

#include <avr/io.h>

#ifndef F_CPU
#define F_CPU 16000000
#endif

#define I2C_SCL_CLOCK 100000 //100kHz clock
#define I2C_BAUDRATE ((F_CPU/I2C_SCL_CLOCK)-16)/(2) //baudrate calculation


#define I2C_PORT PORTC
#define I2C_PIN_SDA 4 // PC4 - data pin
#define I2C_PIN_SCL 5 // PC5 - clock pin


#define I2C_WRITE 0 //write mode value
#define I2C_READ 1 //read mode value
#define I2C_ACK 1   //acknowledge value
#define I2C_NACK 0  //no acknowledge value



void I2C_init();
void I2C_start();
void I2C_stop();

uint8_t I2C_read(uint8_t ack);
uint8_t I2C_write(uint8_t data);
uint8_t I2C_read_ack();
uint8_t I2C_read_nack();



#endif
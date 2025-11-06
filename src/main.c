/*
 * Implementation of LFSR-based (Linear Feedback Shift Register) 
 * pseudo-random generator in AVR assembly.
 * (c) 2017-2025 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and Atmel AVR platform.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 * 
 * NOTE:
 *   To see assembly listing, run the following command in Terminal
 *   after the compilation.
 * 
 *   Windows:
 *   C:\Users\YOUR-LOGIN\.platformio\packages\toolchain-atmelavr\bin\avr-objdump -S -d -m avr .pio/build/uno/firmware.elf > firmware.lst
 * 
 *   Linux, Mac:
 *   ~/.platformio/packages/toolchain-atmelavr/bin/avr-objdump -S -d -m avr .pio/build/uno/firmware.elf > firmware.lst
 * 
 * SEE ALSO:
 *   https://five-embeddev.com/baremetal/platformio/
 */

// -- Includes ---------------------------------------------
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <util/delay.h>     // Functions for busy-wait delay loops
#include "gpio.h"          // GPIO library for AVR-GCC
#include <stdio.h>          // C library for `sprintf`


// -- Defines ----------------------------------------------
#define PARKING_SENSOR_trigger_DDR   DDRB // Data Direction Register for Trigger pin
#define PARKING_SENSOR_trigger_PORT  PORTB // Port Register for Trigger pin
#define PARKING_SENSOR_trigger_PIN   PB0 // Trigger pin for both sensors
#define PARKING_SENSOR_echo_PIN1     PB1 // Echo pin for Right sensor
#define PARKING_SENSOR_echo_PIN2     PB2 // Echo pin for Left sensor

// -- Variables -------------------------------------------

volatile uint16_t parking_sensor_data[4]; // Array to store parking sensor data (base: 16us ~ 0.29 cm)
volatile uint8_t sensor_switch_ON_OFF = 0; // Flag to switch sensors ON/OFF
volatile uint8_t sensor_trigger; // State of the sensor trigger, 0 - not trigger, 1 - trigger
volatile uint8_t sensor_index = 0; // Index for parking_sensor_data array
volatile uint8_t newdata_flag = 0; // Flag for display and UART update
volatile uint8_t display_update_flag = 0; // Flag to indicate display update
// -- Function definitions ---------------------------------
/*
 * Function: Main function where the program execution begins
 * Purpose:  
 * Returns:  none
 */
int main(void)
{
    // Initialize USART to asynchronous, 8-N-1, 9600
    // NOTE: Add `monitor_speed = 9600` to `platformio.ini`
    _delay_ms(100); // Wait for UART to stabilize
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    tim0_ovf_16us();  // Set Timer/Counter0 overflow every 16us
    tim0_ovf_enable();  // Enable Timer/Counter0 overflow interrupt

    tim1_ovf_1sec();  // Set Timer/Counter1 overflow every 1s
    tim1_ovf_enable();  // Enable Timer/Counter1 overflow interrupt

    sei(); // Enable global interrupts

    // Switch sensors ON
    sensor_switch_ON_OFF = 1;


    // Initialize parking sensor data
    for (uint8_t i = 0; i < 4; i++)
    {
        parking_sensor_data[i] = 0;
    }
    // Initialize sensor trigger state
    sensor_trigger = 0;


    // Infinite empty loop
    while (1)
    {
      if (sensor_switch_ON_OFF == 1)
      {
          // Trigger parking sensors every 100ms
          if (sensor_trigger == 0)
          {
              sensor_index++;
              // Set Trigger pin as output
              GPIO_mode_output(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_trigger_PIN);
              GPIO_write_low(&PARKING_SENSOR_trigger_PORT, PARKING_SENSOR_trigger_PIN);
              _delay_ms(40);
              GPIO_write_high(&PARKING_SENSOR_trigger_PORT, PARKING_SENSOR_trigger_PIN);
              _delay_us(10);
              GPIO_write_low(&PARKING_SENSOR_trigger_PORT, PARKING_SENSOR_trigger_PIN);
              GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN1); // Set Echo pin1 as input
              GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN2); // Set Echo pin2 as input
              sensor_trigger = 1; // Set sensor trigger state
          }
      }
      if (display_update_flag == 1)
      {
          
          char uart_buffer[50]; // Buffer for UART transmission

          sprintf(uart_buffer, "Sensors: %d cm %d cm %d cm %d cm\r\n",
          parking_sensor_data[0]*16/58,
          parking_sensor_data[1]*16/58,
          parking_sensor_data[2]*16/58,
          parking_sensor_data[3]*16/58);
          uart_puts(uart_buffer);


          display_update_flag = 0; // Reset the flag
      }
      
    }

    // Will never reach this
    return 0; 
}


ISR(TIMER0_OVF_vect)
{
  static uint16_t sensor_data_temp = 0;
  
    // Right sensor
    if (sensor_trigger == 1 && sensor_index == 1) // Right sensor
    {
    if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 1)
    {
      sensor_data_temp++; // Increment
    }
    else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 0)
    {
      if (sensor_data_temp > 0)
      {
        sensor_trigger = 0; // Reset sensor trigger state
        parking_sensor_data[0] = sensor_data_temp; // Store final distance for Right sensor
        sensor_data_temp = 0; // Reset temporary storage
        newdata_flag = 1; // Set flag to update display and UART
        if (sensor_index >= 2)
        {
          sensor_index = 0;
        }
      }
    }
    }

    // Left sensor
    else if (sensor_trigger == 1 && sensor_index == 2) // Left sensor
    {
    if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 1)
    {
      sensor_data_temp++; // Increment
    }
    else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 0)
    {
      if (sensor_data_temp > 0)
      {
        sensor_trigger = 0; // Reset sensor trigger state
        parking_sensor_data[2] = sensor_data_temp; // Store final distance for Left sensor
        sensor_data_temp = 0; // Reset temporary storage
        newdata_flag = 1; // Set flag to update display and UART
        if (sensor_index >= 2)
        {
              sensor_index = 0;
        }
      }
    }
    }

}
    


/*
ISR(TIMER0_OVF_vect)
{
    static uint8_t sensors_processed[4] = {0, 1, 0, 1}; // To track processed sensors in one trigger cycle
    static uint8_t wait_for_data[4] = {1, 1, 1, 1}; // To implement timeout for each sensor
    if (sensor_switch_ON_OFF == 1)
    {
      if (sensor_trigger_state == 1)
      {   // Read Echo pins
          if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 1)
          {
              parking_sensor_data[0]++; // Increment distance for Right sensor
              wait_for_data[0] = 0; // Reset wait time for Right sensor
          }
          else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 0)
          {
            if (wait_for_data[0] == 0) // Timeout after ~6.4ms
              {
              newdata_flag = 1; // Set flag to update display and UART
              sensors_processed[0] = 1; // Mark Right sensor as processed
              wait_for_data[0] = 1; // Reset wait time for Right sensor
              }
          }
          
          if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 1)
          {
              parking_sensor_data[2]++; // Increment distance for Left sensor
              newdata_flag = 1; // Set flag to update display and UART
              sensors_processed[2] = 1; // Mark Left sensor as processed
          }
          else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 0)
          {
              if (wait_for_data[2] == 0) // Timeout after ~6.4ms
              {
              newdata_flag = 1; // Set flag to update display and UART
              sensors_processed[2] = 1; // Mark Left sensor as processed
              wait_for_data[2] = 1; // Reset wait time for Left sensor
              }
          }
      
          
          if ((sensors_processed[0] == 1) && (sensors_processed[2] == 1))
          {
              // Reset for next trigger cycle
              sensor_trigger_state = 0; // Reset trigger state
              sensors_processed[0] = 0;
              sensors_processed[2] = 0;
          }
      }
        
    }
    
}
*/
ISR(TIMER1_OVF_vect)
{

  if (newdata_flag == 1)
  {
      display_update_flag = 1; // Set flag to update display and UART
      newdata_flag = 0; // Reset new data flag
  }
}
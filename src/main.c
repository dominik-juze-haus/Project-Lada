
// -- Includes ---------------------------------------------
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <util/delay.h>     // Functions for busy-wait delay loops
#include "gpio.h"          // GPIO library for AVR-GCC
#include <stdio.h>          // C library for `sprintf`
#include <stdlib.h>         // C library for general utilities
#include <twi.h>            // TWI library for AVR-GCC
#include <adc.h>           // ADC library for AVR-GCC
#include <oled.h>           // OLED library for AVR-GCC



// -- Defines ----------------------------------------------
<<<<<<< Updated upstream
=======
// Parking sensor pins
>>>>>>> Stashed changes
#define PARKING_SENSOR_trigger_DDR   DDRB // Data Direction Register for Trigger pin
#define PARKING_SENSOR_trigger_PORT  PORTB // Port Register for Trigger pin
#define PARKING_SENSOR_trigger_PIN   PB0 // Trigger pin for both sensors
#define PARKING_SENSOR_echo_PIN1     PB1 // Echo pin for Left outer sensor
#define PARKING_SENSOR_echo_PIN2     PB2 // Echo pin for Left inner sensor
#define PARKING_SENSOR_echo_PIN3     PB3 // Echo pin for Right inner sensor
#define PARKING_SENSOR_echo_PIN4     PB4 // Echo pin for Right outer sensor

<<<<<<< Updated upstream
=======
// Rotary encoder pins
#define RTENC_DDR    DDRD // Data Direction Register for Rotary encoder
#define RTENC_PORT   PORTD // Port Register for Rotary encoder
#define RTENC_button_PIN    PD5 // Pin for Rotary encoder button
#define RTENC_DT_PIN    PD6 // Pin DT for Rotary encoder
#define RTENC_CLK_PIN    PD7 // Pin CLK for Rotary encoder


>>>>>>> Stashed changes
/*
#define MAX_MATRIX_DDR      DDRC // Data Direction Register for MAX7219
#define MAX_MATRIX_PORT     PORTC // Port Register for MAX7219
#define MAX_MATRIX_din_pin    PC4 // DIN pin for MAX7219
#define MAX_MATRIX_clk_pin   PC5 // CLK pin for MAX7219
#define MAX_MATRIX_cs_pin    PC3 // CS pin for MAX7219
*/

// -- Variables -------------------------------------------

volatile uint16_t parking_sensor_data[4]; // Array to store parking sensor data (base: 16us ~ 0.29 cm)
volatile uint8_t sensor_switch_ON_OFF = 0; // Flag to switch sensors ON/OFF
volatile uint8_t sensor_trigger; // State of the sensor trigger, 0 - not trigger, 1 - trigger
volatile uint8_t sensor_index = 0; // Index for parking_sensor_data array


volatile uint8_t newdata_flag = 0; // Flag for display and UART update
volatile uint8_t display_update_flag = 0; // Flag to indicate display update


volatile uint8_t oled_switch_page = 0; // OLED display page switch flag
volatile uint8_t oled_current_page = 0; // OLED current page index
<<<<<<< Updated upstream
=======
volatile uint8_t oled_last_page = 0; // OLED last page index

volatile uint8_t rtenc[2]; // Flag to start ADC conversion
>>>>>>> Stashed changes
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
    twi_init();       // Initialize TWI (I2C)

    tim0_ovf_16us();  // Set Timer/Counter0 overflow every 16us
    tim0_ovf_enable();  // Enable Timer/Counter0 overflow interrupt

    tim1_ovf_262ms();  // Set Timer/Counter1 overflow every 262ms
    tim1_ovf_enable();  // Enable Timer/Counter1 overflow interrupt

    sei(); // Enable global interrupts
    
    oled_init(OLED_DISP_ON); // Initialize OLED display
    oled_clrscr(); // Clear the display
    oled_charMode(NORMALSIZE);
<<<<<<< Updated upstream
    // -----------------------------DEMO SETTINGS-----------------------------
    // Switch sensors ON
    sensor_switch_ON_OFF = 1;
    oled_switch_page = 1;
    oled_current_page = 1;
    // -----------------------------------------------------------------------

=======

    // Configure Rotary encoder button pin as input with pull-up
    GPIO_mode_input_pullup(&RTENC_DDR, RTENC_button_PIN);
    GPIO_mode_input_pullup(&RTENC_DDR, RTENC_DT_PIN);
    GPIO_mode_input_pullup(&RTENC_DDR, RTENC_CLK_PIN);

    // Initialize flags and indexes
    // TODO: read from mem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    

    
    // -----------------------------DEMO SETTINGS-----------------------------
    // Switch sensors ON
    //sensor_switch_ON_OFF = 1; // Parking sensor flag,1 - ON, 0 - OFF
    oled_switch_page = 1; // Set flag to switch OLED page
    oled_current_page = 0; // current OLED page index
    // -----------------------------------------------------------------------
    
>>>>>>> Stashed changes

    // Initialize parking sensor data
    for (uint8_t i = 0; i < 4; i++)
    {
        parking_sensor_data[i] = 0;
    }
<<<<<<< Updated upstream
    // Initialize sensor trigger state
    sensor_trigger = 0;
=======
    
>>>>>>> Stashed changes


    // Infinite empty loop
    while (1)
<<<<<<< Updated upstream
    {
=======
    {  rtenc[0] = GPIO_read(&PIND, RTENC_DT_PIN); //// To do: Read Rotary encoder !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       rtenc[1] = GPIO_read(&PIND, RTENC_CLK_PIN);
       itoa(rtenc[0], NULL, 10);
       uart_puts(NULL);
       itoa(rtenc[1], NULL, 10);
       uart_puts(NULL);
       uart_puts("\r\n");

      if (GPIO_read(&PIND, RTENC_button_PIN) == 0) // Read Rotary encoder button state
      {
          sensor_switch_ON_OFF ^= 1; // Toggle parking sensor ON/OFF state
          
          if (oled_current_page == 3)
          {
              oled_current_page = oled_last_page;
              oled_switch_page = 1; // Set flag to switch OLED page
          }
          else
          {
              oled_last_page = oled_current_page; // Toggle OLED current page index
              oled_current_page = 3;
              oled_switch_page = 1; // Set flag to switch OLED page
          }
          _delay_ms(30); // Debounce delay
      }
>>>>>>> Stashed changes
      if (oled_switch_page == 1)
      {
          // Switch OLED display page
          oled_clrscr(); // Clear the display
<<<<<<< Updated upstream
          if (oled_current_page == 1)
          {
              oled_gotoxy(0, 0);
              oled_puts("Parking Sensors");
=======

          // RPM page
          if (oled_current_page == 0)
          {
              oled_gotoxy(0, 0);
              oled_puts("RPM");
              oled_display();
          }
          
          // Speed page
          if (oled_current_page == 1)
          {   // Speed page
              oled_gotoxy(0, 0);
              oled_puts("Speed");
              oled_display();
          }
          
          // Temperature page
          if (oled_current_page == 2)
          {   // Temperature page
              oled_gotoxy(0, 0);
              oled_puts("Temperature");
              oled_display();
          }
          // Parking sensor page
          if (oled_current_page == 3)
          {    
              oled_drawRect(0, 10, 127, 20, WHITE); // Graphic back of the car
              oled_gotoxy(3, 0);
              oled_puts("Parking Sensors");
              oled_display();
              /*
>>>>>>> Stashed changes
              oled_gotoxy(0, 2);
              oled_puts("Left Outer:");
              oled_gotoxy(0, 3);
              oled_puts("Left Inner:");
              oled_gotoxy(0, 4);
              oled_puts("Right Inner:");
              oled_gotoxy(0, 5);
              oled_puts("Right Outer:");
              oled_display();
<<<<<<< Updated upstream
=======
              */
>>>>>>> Stashed changes
          }
          oled_switch_page = 0; // Reset the flag
      }
      if (sensor_switch_ON_OFF == 1)
<<<<<<< Updated upstream
      {
=======
      {   
          
>>>>>>> Stashed changes
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
              GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN3); // Set Echo pin3 as input
              GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN4); // Set Echo pin4 as input
              sensor_trigger = 1; // Set sensor trigger state
          }
      }
      if (display_update_flag == 1)
      {
<<<<<<< Updated upstream
          
          char uart_buffer[50]; // Buffer for UART transmission

          sprintf(uart_buffer, "Sensors: %d cm %d cm %d cm %d cm\r\n",
          parking_sensor_data[0]*16/58,
          parking_sensor_data[1]*16/58,
          parking_sensor_data[2]*16/58,
          parking_sensor_data[3]*16/58);
          uart_puts(uart_buffer);
          // Update OLED display
          oled_gotoxy(12, 2);
          sprintf(uart_buffer, "%d cm   ", parking_sensor_data[0]*16/58);
          oled_puts(uart_buffer);
          oled_gotoxy(12, 3);
          sprintf(uart_buffer, "%d cm   ", parking_sensor_data[1]*16/58);
          oled_puts(uart_buffer);
          oled_gotoxy(12, 4);
          sprintf(uart_buffer, "%d cm   ", parking_sensor_data[2]*16/58);
          oled_puts(uart_buffer);
          oled_gotoxy(12, 5);
          sprintf(uart_buffer, "%d cm   ", parking_sensor_data[3]*16/58);
          oled_puts(uart_buffer);
          oled_display();

          display_update_flag = 0; // Reset the flag
=======
          if (sensor_switch_ON_OFF == 1)
          {
          
            char uart_buffer[50]; // Buffer for UART transmission

            sprintf(uart_buffer, "Sensors: %d cm %d cm %d cm %d cm\r\n",
            parking_sensor_data[0],
            parking_sensor_data[1],
            parking_sensor_data[2],
            parking_sensor_data[3]);
            uart_puts(uart_buffer); //

            //oled_clrscr(); // Clear the display
            //oled_drawRect(0, 10, 127, 20, WHITE); // Graphic back of the car
            oled_fillRect(9, 21, 35, 62, BLACK); // Clear Left outer sensor bar
            oled_fillRect(36, 21, 62, 62, BLACK); // Clear Left inner sensor bar
            oled_fillRect(63, 21, 89, 62, BLACK); // Clear Right inner sensor bar
            oled_fillRect(90, 21, 116, 62, BLACK); // Clear Right outer sensor bar

            uint16_t bar_height;
            if (parking_sensor_data[0]*16/58 < 200)
            {
                bar_height = 20 + ceil(parking_sensor_data[0]/17.26);

                oled_fillRect(9, bar_height, 35, 62, WHITE); // Left outer sensor bar
            }
            if (parking_sensor_data[1]*16/58 < 200)
            {
                bar_height = 20 + ceil(parking_sensor_data[1]/17.26);

                oled_fillRect(36, bar_height, 62, 62, WHITE); // Left inner sensor bar
            }
            if (parking_sensor_data[2]*16/58 < 200)
            {   
                bar_height = 20 + ceil(parking_sensor_data[2]/17.26);

                oled_fillRect(63, bar_height, 89, 62, WHITE); // Right inner sensor bar
            }
            if (parking_sensor_data[3]*16/58 < 200)
            {   
                bar_height = 20 + ceil(parking_sensor_data[3]/17.26);

                oled_fillRect(90, bar_height, 116, 62, WHITE); // Right outer sensor bar
            }
            
            // Update OLED display
            /*oled_gotoxy(12, 2);
            sprintf(uart_buffer, "%d cm   ", parking_sensor_data[0]*16/58);
            oled_puts(uart_buffer);
            oled_gotoxy(12, 3);
            sprintf(uart_buffer, "%d cm   ", parking_sensor_data[1]*16/58);
            oled_puts(uart_buffer);
            oled_gotoxy(12, 4);
            sprintf(uart_buffer, "%d cm   ", parking_sensor_data[2]*16/58);
            oled_puts(uart_buffer);
            oled_gotoxy(12, 5);
            sprintf(uart_buffer, "%d cm   ", parking_sensor_data[3]*16/58);
            oled_puts(uart_buffer);*/
            oled_display();

            display_update_flag = 0; // Reset the flag
        }
>>>>>>> Stashed changes
      }
      
    }

    // Will never reach this
    return 0; 
}

// -- Interrupt Service Routines --------------------------
ISR(TIMER0_OVF_vect)
{
  static uint16_t sensor_data_temp = 0;
  
    // Left outer sensor
    if (sensor_trigger == 1 && sensor_index == 1) // Left outer sensor
    {
      if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 1)
      {
        sensor_data_temp++; // Increment
      }
      else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 0)
      {
        if (sensor_data_temp > 0) // Avoid the waiting time for echo
        {
          sensor_trigger = 0; // Reset sensor trigger state
          parking_sensor_data[0] = sensor_data_temp; // Store final distance for Right sensor
          sensor_data_temp = 0; // Reset temporary storage
          newdata_flag = 1; // Set flag to update display and UART
          if (sensor_index >= 4)
          {
            sensor_index = 0;
          }
        }
      }
    }

    // Left inner sensor
    else if (sensor_trigger == 1 && sensor_index == 2) // Left inner sensor
    {
      if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 1)
      {
        sensor_data_temp++; // Increment
      }
      else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 0)
      {
        if (sensor_data_temp > 0) // Avoid the waiting time for echo
        {
          sensor_trigger = 0; // Reset sensor trigger state
          parking_sensor_data[1] = sensor_data_temp; // Store final distance for Left inner sensor
          sensor_data_temp = 0; // Reset temporary storage
          newdata_flag = 1; // Set flag to update display and UART
          if (sensor_index >= 4) 
          {
                sensor_index = 0; // Reset sensor index
          }
        }
      }
    }
    // Right inner sensor
    else if (sensor_trigger == 1 && sensor_index == 3) // Right inner sensor
    {
    if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN3) == 1)
      {
        sensor_data_temp++; // Increment
      }
      else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN3) == 0)
      {
        if (sensor_data_temp > 0) // Avoid the waiting time for echo
        {
          sensor_trigger = 0; // Reset sensor trigger state
          parking_sensor_data[2] = sensor_data_temp; // Store final distance for Right inner sensor
          sensor_data_temp = 0; // Reset temporary storage
          newdata_flag = 1; // Set flag to update display and UART
          if (sensor_index >= 4)
          {
                sensor_index = 0; // Reset sensor index
          }
        }
      }
    }
    // Right outer sensor
    else if (sensor_trigger == 1 && sensor_index == 4) // Right outer sensor
    {
    if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN4) == 1)
      {
        sensor_data_temp++; // Increment
      }
      else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN4) == 0)
      {
        if (sensor_data_temp > 0) // Avoid the waiting time for echo
        {
          sensor_trigger = 0; // Reset sensor trigger state
          parking_sensor_data[3] = sensor_data_temp; // Store final distance for Right outer sensor
          sensor_data_temp = 0; // Reset temporary storage
          newdata_flag = 1; // Set flag to update display and UART
          if (sensor_index >= 4)
          {
                sensor_index = 0; // Reset sensor index
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
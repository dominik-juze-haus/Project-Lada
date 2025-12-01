
// -- Includes ---------------------------------------------
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <util/delay.h>     // Functions for busy-wait delay loops
#include "gpio.h"           // GPIO library for AVR-GCC
#include <stdio.h>          // C library for `sprintf`
#include <stdlib.h>         // C library for general utilities
#include <twi.h>            // TWI library for AVR-GCC by Tomas Fryza
#include <adc.h>            // ADC library for AVR-GCC by EX4 (ekapujiw2002@gmail.com)
#include <oled.h>           // OLED library for AVR-GCC 



// -- Defines ----------------------------------------------
// Parking sensor pins
#define PARKING_SENSOR_trigger_DDR   DDRB // Data Direction Register for Trigger pin
#define PARKING_SENSOR_trigger_PORT  PORTB // Port Register for Trigger pin
#define PARKING_SENSOR_trigger_PIN   PB0 // Trigger pin for both sensors
#define PARKING_SENSOR_echo_PIN1     PB1 // Echo pin for Left outer sensor
#define PARKING_SENSOR_echo_PIN2     PB2 // Echo pin for Left inner sensor
#define PARKING_SENSOR_echo_PIN3     PB3 // Echo pin for Right inner sensor
#define PARKING_SENSOR_echo_PIN4     PB4 // Echo pin for Right outer sensor

// Speed sensor optical input pin
//#define SPEED_SENSOR_optical_DDR     DDRB // Data Direction Register for Speed sensor optical input
//#define SPEED_SENSOR_optical_PORT    PORTB // Port Register for Speed sensor optical input
//#define SPEED_SENSOR_optical_PIN     PB5 // Pin for Speed sensor optical input

// Speed sensor magnetic Hall's effect input pin
#define SPEED_SENSOR_magnetic_DDR     DDRB // Data Direction Register for Speed sensor magnetic input
#define SPEED_SENSOR_magnetic_PORT    PORTB // Port Register for Speed sensor magnetic input
#define SPEED_SENSOR_magnetic_PIN     PB5  // Pin for Speed sensor magnetic input


// Rotary encoder pins
#define RTENC_DDR    DDRD // Data Direction Register for Rotary encoder
#define RTENC_PORT   PORTD // Port Register for Rotary encoder
#define RTENC_button_PIN    PD5 // Pin for Rotary encoder button
#define RTENC_DT_PIN    PD6 // Pin DT for Rotary encoder
#define RTENC_CLK_PIN    PD7 // Pin CLK for Rotary encoder

// UART Pins
// (Using default UART pins: PD0 - RX, PD1 - TX)

// ADC channels
#define ADC_channel_RPM        0 // ADC channel for RPM sensor
#define ADC_channel_TemperatureOil 1 // ADC channel for Oil Temperature sensor
#define ADC_channel_TemperatureCoolant 2 // ADC channel for Coolant Temperature sensor



/*
#define MAX_MATRIX_DDR      DDRC // Data Direction Register for MAX7219
#define MAX_MATRIX_PORT     PORTC // Port Register for MAX7219
#define MAX_MATRIX_din_pin    PC4 // DIN pin for MAX7219
#define MAX_MATRIX_clk_pin   PC5 // CLK pin for MAX7219
#define MAX_MATRIX_cs_pin    PC3 // CS pin for MAX7219
*/

// -- Variables -------------------------------------------
// -- Data variables ------------------------
volatile uint16_t parking_sensor_data[4]; // Array to store raw parking sensor data (base: 16us ~ 0.29 cm) from all 4 HC-SR04 sensors
volatile uint16_t rpm_value = 0; // Variable to store RPM value from ADC
volatile uint16_t optical_speed_value = 0; // Variable to store raw speed value from optical switch
volatile uint8_t magnetic_speed_value[5]; // Variable to store raw speed value from magnetic Hall's sensor (pulse count in time frame) [0] - current, [1], [2], [3] - last 3 readings for averaging, 4 - calculated speed in km/h
volatile uint16_t temperature_sensor_data[4]; // Array to store raw temperature sensor data (base: ADC voltage) [0] - Oil temp voltage, [1] - Coolant temp voltage, [2] - Oil temp Celsius, [3] - Coolant temp Celsius
volatile uint8_t rtenc[3]; // Rotary encoder states variable, rtenc  [0] - DT pin state, rtenc [1] - CLK pin state, rtenc [2] - previous state

// -- Switches ------------------------------
volatile uint8_t sensor_switches[4] = {0, 0, 0, 0}; // Array of flags for sensors: [0] - Parking, [1] - RPM, [2] - Temperature, [3] - Speed 
volatile uint8_t parking_sensor_trigger; // State of the sensor trigger, 0 - not trigger, 1 - trigger


// -- Flags --------------------------------
volatile uint8_t newdata_flag = 0; // Flag for display and UART update
volatile uint8_t data_process_flag = 0; // Flag to indicate data processing
volatile uint8_t display_update_flag = 0; // Flag to indicate display update
volatile uint8_t speed_sensor_pulse_flag = 0; // Flag for speed sensor pulse detection
volatile uint8_t oled_switch_page = 0; // OLED display page switch flag

// -- Indexes ------------------------------
volatile uint8_t current_systems = 0; // index for the current used sensor system/page on OLED display
volatile uint8_t oled_last_page = 0; // OLED last page index
volatile uint8_t sensor_index = 0; // Index for parking_sensor_data array



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
    ADC_Init(ADC_VREF_AVCC, ADC_DATA_10BIT, ADC_PSC16); // Initialize ADC

    tim0_ovf_16us();  // Set Timer/Counter0 overflow every 16us
    tim0_ovf_enable();  // Enable Timer/Counter0 overflow interrupt

    tim1_ovf_262ms();  // Set Timer/Counter1 overflow every 262ms
    tim1_ovf_enable();  // Enable Timer/Counter1 overflow interrupt

    sei(); // Enable global interrupts
    
    oled_init(OLED_DISP_ON); // Initialize OLED display
    oled_clrscr(); // Clear the display
    oled_charMode(NORMALSIZE);

    
    GPIO_mode_input_pullup(&RTENC_DDR, RTENC_button_PIN); // Configure Rotary encoder button pin as input with pull-up
    GPIO_mode_input_pullup(&RTENC_DDR, RTENC_DT_PIN); // Configure Rotary encoder DT pin as input with pull-up
    GPIO_mode_input_pullup(&RTENC_DDR, RTENC_CLK_PIN); // Configure Rotary encoder CLK pin as input with pull-up

    GPIO_mode_input_nopull(&SPEED_SENSOR_magnetic_DDR, SPEED_SENSOR_magnetic_PIN); // Configure Speed sensor magnetic pin as input without pull-up
    

    

    
    // -----------------------------DEMO SETTINGS-----------------------------
    // Switch sensors ON
    //sensor_switch_ON_OFF = 1; // Parking sensor flag,1 - ON, 0 - OFF
    oled_switch_page = 1; // Set flag to switch OLED page
    current_systems = 0; // current OLED page index
    // -----------------------------------------------------------------------
    

    // Initialize parking sensor data
    for (uint8_t i = 0; i < 4; i++)
    {
        parking_sensor_data[i] = 0;
    }
    


    // Infinite empty loop
    while (1)
      { 
        
        rtenc[0] = GPIO_read(&PIND, RTENC_DT_PIN); 
        rtenc[1] = GPIO_read(&PIND, RTENC_CLK_PIN);
        //itoa(rtenc[0], NULL, 10); // Debugging rotary encoder - uart output
        //uart_puts(NULL);
        //itoa(rtenc[1], NULL, 10);
        //uart_puts(NULL);
        //uart_puts("\r\n");
        
        // Rotary encoder increment/decrement handling
        if (rtenc[0] == rtenc[1]) // If DT and CLK pins are the same
        {
            rtenc[2] = rtenc[0]; // Store previous state
        }
        
        if (rtenc[0] != rtenc[1] && current_systems != 3) // If DT and CLK pins are different and not on parking sensor page
        { 
            if (rtenc[2] != rtenc[0]) // Counter-clockwise rotation
            { 
                
                if (current_systems == 0)
                {
                    current_systems = 2;
                }
                else
                {
                    current_systems--;
                }
                oled_switch_page = 1; // Set flag to switch OLED page
            }
            if (rtenc[2] != rtenc[1])  // Clockwise rotation
            {
                current_systems++;
                if (current_systems > 2)
                {
                    current_systems = 0;
                }
                oled_switch_page = 1; // Set flag to switch OLED page
            }
        }


      
        // Rotary encoder button handling
        if (GPIO_read(&PIND, RTENC_button_PIN) == 0) // Read Rotary encoder button state - pressed - switches to/from parking sensor page
        {
            sensor_switches[0] ^= 1; // Toggle parking sensor ON/OFF state
            
            if (current_systems == 3) // If currently on parking sensor page
            {
                current_systems = oled_last_page; // Return to last page
                oled_switch_page = 1; // Set flag to switch OLED page
            }
            else // If not on parking sensor page
            {
                oled_last_page = current_systems; // Store current page as last page
                current_systems = 3; // Switch to parking sensor page
                oled_switch_page = 1; // Set flag to switch OLED page
            }
            _delay_ms(30); // Debounce delay
        }

        // --------------------------------------------------------------------------- Systems switching -----------------------------------
        if (oled_switch_page == 1)
        {
            // Switch OLED display page
            oled_clrscr(); // Clear the display

            // ------------------------------------------------------------RPM page
            if (current_systems == 0)
            { 
                sensor_switches[1] = 1; // Enable RPM measurement
                oled_gotoxy(0, 0);
                oled_puts("RPM");
                oled_display();
            }
            else
            {
                sensor_switches[1] = 0; // Disable RPM measurement
            }
            
            
            // -----------------------------------------------------------Temperature page
            if (current_systems == 1)
            {   
                sensor_switches[2] = 1; // Enable Temperature measurement
                oled_gotoxy(0, 0);
                oled_puts("Temperature");
                oled_display();
            }
            else
            {
                sensor_switches[2] = 0; // Disable Temperature measurement
            }

            // -----------------------------------------------------------Speed page
            if (current_systems == 2)
            {   // Speed page
                sensor_switches[3] = 1; // Enable Speed measurement
                oled_gotoxy(0, 0);
                oled_puts("Speed");
                oled_display();
            }
            else
            {
                sensor_switches[3] = 0; // Disable Speed measurement
            }

            // -----------------------------------------------------------Parking sensor page
            if (current_systems == 3)
            {    
                oled_drawRect(0, 10, 127, 20, WHITE); // Graphic back of the car
                oled_gotoxy(3, 0);
                oled_puts("Parking Sensors");
                oled_display();
                /*
                oled_gotoxy(0, 2);
                oled_puts("Left Outer:");
                oled_gotoxy(0, 3);
                oled_puts("Left Inner:");
                oled_gotoxy(0, 4);
                oled_puts("Right Inner:");
                oled_gotoxy(0, 5);
                oled_puts("Right Outer:");
                oled_display();
                */
            }
            oled_switch_page = 0; // Reset the flag
        }



        // --------------------------------------------------------------------------- Sensor calculations -----------------------------------
        
        if (data_process_flag == 1)
        {
          uart_puts("Processing");
          // --------------------------------------------------------------------------------------------------------Temperature data processing   
          if (current_systems == 1)
          {   
            uart_puts(" temperature data...\r\n");                    
            float temp_oil_V = ((float)temperature_sensor_data[0] * 5.0 / 1023.0); // Read Oil Temperature raw voltage value
            float temp_coolant_V = ((float)temperature_sensor_data[1] * 5.0 / 1023.0); // Read Coolant Temperature raw voltage value
            
            /* // Debugging temperature calculations - uart output
            itoa((uint16_t)temperature_sensor_data[0], buffer, 10);
            uart_puts("Oil Temp Voltage ADC: ");
            uart_puts(buffer);
            uart_puts("\r\n");
            itoa((uint16_t)temperature_sensor_data[1], buffer, 10);
            uart_puts("Coolant Temp Voltage ADC: ");
            uart_puts(buffer);
            uart_puts("\r\n");*/
            

            // Equation to get resistance: R = (R1*Vout) / (Vin - Vout)
            float temp_oil_R = (1000*temp_oil_V) / (5-temp_oil_V); // Convert Oil temperature voltage to Celsius
            float temp_coolant_R = (1000*temp_coolant_V) / (5-temp_coolant_V); // Convert Coolant temperature voltage to Celsius

            // Steinhart-Hart equation to convert resistance to temperature in Kelvin
            float temp_oil_C = (1 / (0.001129148 + (0.000234125 * log(temp_oil_R)) + (0.0000000876741 * pow(log(temp_oil_R), 3)))) - 273.15;
            float temp_coolant_C = (1 / (0.001129148 + (0.000234125 * log(temp_coolant_R)) + (0.0000000876741 * pow(log(temp_coolant_R), 3)))) - 273.15;
            
            /* // Debugging temperature calculations - uart output
            itoa((uint16_t)temp_oil_C, buffer, 10);
            uart_puts("Oil Temp Celsius: ");
            uart_puts(buffer);
            uart_puts("\r\n");

            itoa((uint16_t)temp_coolant_C, buffer, 10);
            uart_puts("Coolant Temp Celsius: ");
            uart_puts(buffer);
            uart_puts("\r\n");*/

            temperature_sensor_data[3] = ceil((uint16_t)temp_oil_C); // Store Oil temperature in Celsius
            temperature_sensor_data[4] = ceil((uint16_t)temp_coolant_C); // Store Coolant temperature in Celsius
            data_process_flag = 0; // Reset data processing flag
            newdata_flag = 1; // Set new data flag for display and UART update
          }
        
          // --------------------------------------------------------------------------------------------------------Speed data processing
          if (current_systems == 2)
          {
            uart_puts(" speed data...\r\n");
                // Calculate average speed from last 3 readings
            //uint16_t speed_average = (magnetic_speed_value[1] + magnetic_speed_value[2] + magnetic_speed_value[3]) / 3;
            uint16_t speed_average = magnetic_speed_value[1]; // Use only last reading for speed calculation
            // Convert pulse count to speed in km/h
            // Assuming 1 pulse per wheel revolution, wheel circumference = 2.0 meters, time frame = 262ms FOR DEMO PURPOSES ONLY
            // Speed (km/h) = (pulse_count * wheel_circumference (m) / time_frame (s)) * 3.6
            float speed_kmh = ((float)speed_average * 1) * 3.6; // UPDATE EQUATION BASED ON ACTUAL PARAMETERS 
            
            
            magnetic_speed_value[4] = ceil((uint16_t)speed_kmh); // Store speed value as integer
            data_process_flag = 0; // Reset data processing flag
            newdata_flag = 1; // Set new data flag for display and UART update
          }
        }
        
        // --------------------------------------------------------------------------- Sensor triggering ------------------------------------
        if (sensor_switches[0] == 1) // if parking sensor switch is ON
        {   
            
            // Trigger parking sensors every 100ms
            if (parking_sensor_trigger == 0)
            {
                sensor_index++;
                // Set Trigger pin as output
                GPIO_mode_output(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_trigger_PIN); // Set Trigger pin as output
                GPIO_write_low(&PARKING_SENSOR_trigger_PORT, PARKING_SENSOR_trigger_PIN); // Set Trigger pin LOW
                _delay_ms(40); // Wait 40ms between triggers
                GPIO_write_high(&PARKING_SENSOR_trigger_PORT, PARKING_SENSOR_trigger_PIN); // Set Trigger pin HIGH
                _delay_us(10); // Trigger pulse width 10us
                GPIO_write_low(&PARKING_SENSOR_trigger_PORT, PARKING_SENSOR_trigger_PIN); // Set Trigger pin LOW
                GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN1); // Set Echo pin1 as input
                GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN2); // Set Echo pin2 as input
                GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN3); // Set Echo pin3 as input
                GPIO_mode_input_nopull(&PARKING_SENSOR_trigger_DDR, PARKING_SENSOR_echo_PIN4); // Set Echo pin4 as input
                parking_sensor_trigger = 1; // Set sensor trigger state
            }
        }
        
        if (sensor_switches[3] == 1) // if speed measurement switch is ON
        {
          
          if (GPIO_read(&PINB, SPEED_SENSOR_magnetic_PIN) == 0 && speed_sensor_pulse_flag == 0)
              { 
                  speed_sensor_pulse_flag = 1; // Pulse detected
              }
          if (GPIO_read(&PINB, SPEED_SENSOR_magnetic_PIN) == 1 && speed_sensor_pulse_flag == 1)
              {   
                  speed_sensor_pulse_flag = 0; // Reset pulse flag
                  magnetic_speed_value[0]++; // Increment pulse count
                  uart_puts("PULSE            \r\n");
              }
        }

        // --------------------------------------------------------------------------- OLED display and UART update -----------------------------------
        if (display_update_flag == 1) 
        {   
            // --------------------------------------------------------------------------- Update OLED display and UART with parking sensor data
            if (sensor_switches[0] == 1) // if parking sensor switch is ON
            {
            
              char uart_buffer[50]; // Buffer for UART transmission

              sprintf(uart_buffer, "Sensors: %d cm %d cm %d cm %d cm\r\n",
              parking_sensor_data[0],
              parking_sensor_data[1],
              parking_sensor_data[2],
              parking_sensor_data[3]);
              uart_puts(uart_buffer); //

              // Update OLED display with parking sensor data
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
              
              oled_display(); // Update OLED display

              display_update_flag = 0; // Reset the flag
          }
            // --------------------------------------------------------------------------- Update OLED display and UART with RPM
            if (sensor_switches[1] == 1) // if RPM measurement switch is ON
            {
                //ADC_Init(ADC_VREF_AVCC, ADC_DATA_10BIT, ADC_PSC16); // Re-initialize ADC
                
                char uart_buffer[30]; // Buffer for UART transmission

                sprintf(uart_buffer, "RPM: %d rpm\r\n", rpm_value);
                uart_puts(uart_buffer); //

                oled_gotoxy(0, 2);
                sprintf(uart_buffer, "Value: %d rpm   ", rpm_value);
                oled_puts(uart_buffer);
                oled_display();
                display_update_flag = 0; // Reset the flag
            }

            // --------------------------------------------------------------------------- Update OLED display and UART with Temperature
            if (sensor_switches[2] == 1) // if Temperature measurement switch is ON
            {
                char uart_buffer[50]; // Buffer for UART transmission

                itoa(temperature_sensor_data[3], uart_buffer, 10);
                uart_puts("Oil Temperature: ");
                uart_puts(uart_buffer);
                uart_puts(" ");
                itoa(temperature_sensor_data[4], uart_buffer, 10);
                uart_puts("Coolant Temperature: ");
                uart_puts(uart_buffer);
                uart_puts("\r\n");


                oled_gotoxy(0, 2);
                oled_puts("Oil: ");
                itoa(temperature_sensor_data[3], uart_buffer, 10);
                oled_puts(uart_buffer);
                oled_puts(" C   ");
                oled_gotoxy(0, 3);                                                       
                oled_puts("Coolant: ");
                itoa(temperature_sensor_data[4], uart_buffer, 10);
                oled_puts(uart_buffer);
                oled_puts(" C   ");
                oled_display();
                display_update_flag = 0; // Reset the flag
            }
            
            // --------------------------------------------------------------------------- Update OLED display and UART with Speed
            if (sensor_switches[3] == 1) // if Speed measurement switch is ON
            {
              char uart_buffer[50]; // Buffer for UART transmission
              itoa(magnetic_speed_value[4], uart_buffer, 10);
              uart_puts("Speed: ");
              uart_puts(uart_buffer);
              uart_puts(" km/h\r\n");

              oled_gotoxy(0, 2);
              oled_puts("Speed: ");
              oled_puts(uart_buffer);
              oled_puts(" km/h   ");
              oled_display();
              display_update_flag = 0; // Reset the flag
            }    
        }
      
    }

    // Will never reach this
    return 0; 
}

// -------------------------------------------------------------- I N T E R R U P T   S E R V I C E   R O U T I N E S --------------------------------------------
// -------------------------------------- Timer/Counter0 overflow interrupt service routine (16us)
// Parking sensor echo signal handler
ISR(TIMER0_OVF_vect)
{
  static uint16_t parking_sensor_temp = 0;
    
  // -------------------------------- Parking sensor echo signal handling --------------------------------
  // Left outer sensor
  if (parking_sensor_trigger == 1 && sensor_index == 1) // Left outer sensor
  {
    if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 1)
    {
      parking_sensor_temp++; // Increment
    }
    else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN1) == 0)
    {
      if (parking_sensor_temp > 0) // Avoid the waiting time for echo
      {
        parking_sensor_trigger = 0; // Reset sensor trigger state
        parking_sensor_data[0] = parking_sensor_temp; // Store final distance for Right sensor
        parking_sensor_temp = 0; // Reset temporary storage
        newdata_flag = 1; // Set flag to update display and UART
        if (sensor_index >= 4)
        {
          sensor_index = 0;
        }
      }
    }
  }

  // Left inner sensor
  else if (parking_sensor_trigger == 1 && sensor_index == 2) // Left inner sensor
  {
    if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 1)
    {
      parking_sensor_temp++; // Increment
    }
    else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN2) == 0)
    {
      if (parking_sensor_temp > 0) // Avoid the waiting time for echo
      
      {
        parking_sensor_trigger = 0; // Reset sensor trigger state
        parking_sensor_data[1] = parking_sensor_temp; // Store final distance for Left inner sensor
        parking_sensor_temp = 0; // Reset temporary storage
        newdata_flag = 1; // Set flag to update display and UART
        if (sensor_index >= 4) 
        {
              sensor_index = 0; // Reset sensor index
        }
      }
    }
  }
  // Right inner sensor
  else if (parking_sensor_trigger == 1 && sensor_index == 3) // Right inner sensor
  {
  if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN3) == 1)
    {
      parking_sensor_temp++; // Increment
    }
    else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN3) == 0)
    {
      if (parking_sensor_temp > 0) // Avoid the waiting time for echo
      {
        parking_sensor_trigger = 0; // Reset sensor trigger state
        parking_sensor_data[2] = parking_sensor_temp; // Store final distance for Right inner sensor
        parking_sensor_temp = 0; // Reset temporary storage
        newdata_flag = 1; // Set flag to update display and UART
        if (sensor_index >= 4)
        {
              sensor_index = 0; // Reset sensor index
        }
      }
    }
  }
  // Right outer sensor
  else if (parking_sensor_trigger == 1 && sensor_index == 4) // Right outer sensor
  {
  if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN4) == 1)
    {
      parking_sensor_temp++; // Increment
    }
    else if (GPIO_read(&PINB, PARKING_SENSOR_echo_PIN4) == 0)
    {
      if (parking_sensor_temp > 0) // Avoid the waiting time for echo
      {
        parking_sensor_trigger = 0; // Reset sensor trigger state
        parking_sensor_data[3] = parking_sensor_temp; // Store final distance for Right outer sensor
        parking_sensor_temp = 0; // Reset temporary storage
        newdata_flag = 1; // Set flag to update display and UART
        if (sensor_index >= 4)
        {
              sensor_index = 0; // Reset sensor index
        }
      }
    }
  }

}

// -------------------------------------- Timer/Counter1 overflow interrupt service routine (262ms)
// Display and UART update handler, ADC read handler
ISR(TIMER1_OVF_vect)
{
  if (newdata_flag == 1) // if new data is available
    {
        display_update_flag = 1; // Set flag to update display and UART
        newdata_flag = 0; // Reset new data flag
    }

  // -------------------------------------- RPM sensor ADC reading -------------------------------------------
  if (sensor_switches[1] == 1) // if RPM measurement switch is ON
  {
      rpm_value = ADC_ReadData(ADC_channel_RPM); // Read RPM value from ADC
      display_update_flag = 1; // Set flag to update display and UART
  }
  
  // -------------------------------------- Temperature sensor ADC reading -----------------------------------
  if (sensor_switches[2] == 1 && data_process_flag == 0) // if Temperature measurement switch is ON
  {
      uint16_t temp_oil = ADC_ReadData(ADC_channel_TemperatureOil); // Read Oil Temperature value from ADC
      uint16_t temp_coolant = ADC_ReadData(ADC_channel_TemperatureCoolant); // Read Coolant Temperature value from ADC
      temperature_sensor_data[0] = temp_oil;
      temperature_sensor_data[1] = temp_coolant;
      data_process_flag = 1; // Set flag to process new temperature data
  }
  
  // ---------------------------------- Speed sensor magnetic signal handling --------------------------------
  if (sensor_switches[3] == 1) // if Speed sensor switch is ON
  {
      static uint8_t i;
      i++;
      if (i >= 8) // Process speed data every ~2.1s (8 * 262ms)
      {
        i = 0;
        magnetic_speed_value[1] = magnetic_speed_value[0];
        magnetic_speed_value[0] = 0;  // Reset current pulse count for next time frame 
        data_process_flag = 1; // Set flag to process new speed data
      }
      
  }
  
  
  
}



// ____________________________________________________________E N D   O F   F I L E____________________________________________________________________________


//_____________________________________________________________________MESS_____________________________________________________________________________________
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
# Project-Lada
 AVR Vehicle System

This is project was created for purposes of Digital Electronics 2 course at Brno University of Technology

The goal is to create a retrofit system for Lada 1300 that will give it more modern driving and assistive capabilities.

The main features contain:
1. Rear (and optionaly front) parking sensor system
2. Water and oil temperature readings
3. RPM and Speed readings

Optional ambitious features are:
1. gear indicator with shifting suggestions
2. Auto-trigger of parking systems when put into reverse
3. Fuel readout with low level alert

List of parts:
- Atmega 328P microcontroller on an UNO R3 board
- HC-SR04 ultrasonic sensor (weather sealed) - 4 pcs
- SSH1106 128 x 64 OLED i2c display
- KY-040 rotary encoder with button
- Siemens QAD21/209 Ni1000 Thermal sensor (passive) - 2 pcs
- Optical switch Groove H2010 driven by LM393
- Wiring (jumpwires, USB cable, breadboard...)


## Modules
### Parking system
Parking system is a welcomed feature in any car. 
This goes particulary handy for vintage cars, because the outside visibility is also often very limited. For example, Lada 1300 has only a left mirror.

Software description
Subsystem is summoned when
- shifter goes to reverse (gear shifter button)
- parking system button is pressed (rotary encoder button)

At first, parking subsystem initializes
- sets all it's flag variables accordingly (sensor triggering, page indexes etc)
- sends camera initializing instruction over uart

Parking subsystem then runs
- periodically using 16 us timer 0 measures distance (4 ultrasonic sensors evenly spaced out) one by one
- distances are periodically drawn on display in a diagram form
- camera feed is displayed on a separate infotainment (instructed via UART)

Subystem turns off when
- speed exceedes 15 km/h not in reverse
- parking system button is pressed again

When subsystem turns off
- sensor reading are disabled
- instruction to disable the camera i sent via uart
- entire system returns to the last used page (before the parking subsystem was summoned)

Hardware solution
Sensors
For demonstration, the project utilizes basic HC-SR04 ultrasonic sensors.
For practical use, a weather sealed IP rated version of the HC ultrasonic sensors would be used.

Camera
Since image decoding and processing uses a lot more processing power than that of an Atmega328p microcontroller, a Raspberry Pi board would be used to decode the image and simultaneously to run the standalone infotainment.
At least an HD resolution camera with decent low light capabilities would be used.
For demonstration purposes, the initializing camera signal is being sent via UART to another Arduino.

### Water and Oil temperature
This particular car model doesn't have a reliable oil temperature readings and water temperature doesn't seem to be precise. 
Old carburetor cars are prone to overheating, especialy those from the eastern block. 
Temperature monitoring is then crucial for the car health and the owners wallets.

Software description
Subsystem is summoned when
- it's corresponding index is dialed with the rotary encoder

At first, the temperature subsystem
- sets all it's flag variables accordingly (indexes, pages flag etc...)

Temperature subsystem runs
- it periodically reads voltage on 2 channels using ADC and 262 ms timer 1
- calculates temperature from the ADC reading
	- it first converts raw ADC result to voltage
	- from the known components of the voltage divider, it calculates resistance
	- using equations corresponding to the specific sensor (Pt1000, Ni1000, 10k Thermistor etc.), it calculates temperature
- shows the temperature readings on the display

Sybsystem turns off when
- it's index is no longer selected (different page is selected)

When subsystem turns off
- sensor readings are disabled

Hardware solution
For both practical and demonstration purposes, the system uses Pt1000 sensors read by the Arduino's AD converter.

### RPM readings
Lada 1300 misses a tachometer and uses only a speed gauge. Retrofiting it with a digital system will ensure optimal shifting timing and prevent overreving the engine.

Software description
Subsystem is summoned when
- it's corresponding index is dialed with the rotary encoder

At first, the RPM subsystem
- sets all it's flag variables accordingly (pages, indexes...)

RPM subsystem runs
- it periodically reads voltage from 1 ADC channel using 262 ms timer 1
- it converts the voltage value to RPM using an equation
	- voltage change corresponds to RPM change
- if the RPMs exceed a redline value, an LED lights up and if the engine is overreved, the LED blinks
- it displays the RPM value

Subsystem turns off when
- it's index is no longer selected (different page is selected)

When subsystem turns off
- ADC readings are disabled

Hardware solution
reading the voltage difference between the alternator and ditributor or halls/optical switch

### Speed measurment

Software description
Subsystem is summoned when
- it's corresponding index is dialed with the rotary encoder

At first, the speed subsystem
- sets all it's flag variables accordingly (pages, indexes...)

Speed subsystem runs
- it counts the switching of the hall's sensor every 2s (using 262 ms timer1 and a simple counter) and, based on that, calculates the speed value
- it displays the speed value

Subsystem turns off when
- it's index is no longer selected (different page is selected)

When subsystem turns off
- the sensor readings are disabled

Hardware solution
Halls switch triggered by a magnet in the integrated speedometer

<img width="1483" height="1102" alt="SimulIdeSchematic" src="https://github.com/user-attachments/assets/c0927a90-1865-4c8d-9ecf-306a224259e3" />

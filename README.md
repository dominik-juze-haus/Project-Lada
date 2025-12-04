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
- Regin TG-AH4/PT1000 Thermal sensor (passive) - 2 pcs
- Hall's sensor module 3144 driven by AZ393M (Digital mode)
- Wiring (jumpwires, USB cable, breadboard...)


## Modules
### Parking system
Parking system is a welcomed feature in any car. 
This goes particulary handy for vintage cars, because the outside visibility is also often very limited. For example, Lada 1300 has only a left mirror.

At the core concept, when the system is triggered, it starts a camera and graphicaly displays distances from obstacles in a clever and understandable manner.
Distances from the car are measured by 4 ultrasonic sensors evenly spaced out in a line to ensure optimal coverage. 
Camera hardware would be operated by a standalone infotainment, which will display the camera output on a screen when instructed by the arduino via I2C. 

Hardware solution
Sensors
For demonstration, the project utilizes basic HC-SR04 ultrasonic sensors.
For practical use, a weather sealed IP rated version of the HC ultrasonic sensors would be used.

Camera
Since image decoding and processing uses a lot more processing power than that of an Atmega328p microcontroller, a Raspberry Pi board would be used to decode the image and simultaneously to run the standalone infotainment.
At least an HD resolution camera with decent low light capabilities would be used.
We find solutions like arducam unsatisfactory due to their low performance. 
For demonstration purposes, the turning on camera signal is being sent via I2C to another Arduino in place of the Raspberry.

### Water and Oil temperature
This particular car model doesn't have a reliable oil temperature readings and water temperature doesn't seem to be precise. 
Old carburetor cars are prone to overheating, especialy those from the eastern block. 
Temperature monitoring is then crucial for the car health and the owners wallets.

The readings will be obtained by utilizing passive thermal sensors placed onto the water and oil reservoir.

Hardware solution
For both practical and demonstration purposes, the system uses Pt1000 sensors read by the Arduino's AD converter.

### RPM and Speed readings

Lada 1300 misses a tachometer and uses only a speed gauge. Retrofiting it with a digital system will ensure optimal shifting timing and give the driver more options to monitor the speed, set custom alerts or display statistics.

The module

Hardware solution
RPM measuring - reading the voltage difference between the alternator and ditributor/halls sensor
Speed measuring - halls sensor/optical switch

<img width="1483" height="1102" alt="SimulIdeSchematic" src="https://github.com/user-attachments/assets/c0927a90-1865-4c8d-9ecf-306a224259e3" />

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
Parking system is nowadays a standard feature in most new cars. It improves precision and awareness to the driver while navigating the car into parking spots or tight spaces. This goes particulary handy for vintage cars, because not only they carry a substantial value to the owners, but the outside visibility is also often very limited. For example, Lada 1300 has only a left mirror.

At the core concept, when the system is triggered, it starts a camera and graphicaly displays distances from obstacles in a clever and understandable manner.
Distances from the car are measured by 4 ultrasonic sensors evenly spaced out in a line to ensure optimal coverage. Camera hardware would be operated by a standalone infotainment, which will display the camera output on a screen when instructed by the arduino via I2C. A wide angle camera is placed at an angle to capture the entire area behind the car.

Hardware solution
Sensors
For demonstration, the project utilizes basic HC-SR04 ultrasonic sensors. Though those would be unsuitable for practical use due to their lack of any IP rating and therefore water and dust resistance that is a common occurence on the road. 
For practical use, a weather sealed IP rated version of the HC ultrasonic sensors would be used.

Camera
Since image decoding and processing uses a lot more processing power than that of an Atmega328p microcontroller, a Raspberry Pi board would be use to decode the image and simultaneously to run the standalone infotainment. At least an HD resolution camera with decent low light capabilities would be used. We find solutions like arducam unsatisfactory due to their low performance. For demonstration purposes, the turning on camera signal is being sent via I2C to another Arduino in place of the Raspberry.

### Water and Oil temperature
This particular car model doesn't have a reliable oil temperature readings and water temperature doesn't seem to be precise. Old carburetor cars are prone to overheating, especialy those from the eastern block. Temperature monitoring is then crucial for the car health and the owners wallets.

The readings will be obtained by utilizing an analog thermal sensors placed onto the water and oil reservoir, measuring their resistance and calculating the temperatures from those values.

Hardware solution
For both practical and demonstration purposes, the system uses PT1000 sensors read by the Arduino's AD converter.

### RPM and Speed readings
Lada 1300 misses a tachometer and uses only a speed gauge. Retrofiting it with a digital system will ensure optimal shifting timing and give the driver more options to monitor the speed, set custom alerts or display statistics.

The module

Hardware solution
RPM measuring - reading the voltage difference between the alternator and ditributor/halls sensor
Speed measuring - halls sensor/optical switch

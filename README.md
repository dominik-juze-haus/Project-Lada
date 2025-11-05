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


## Modules
### Parking system
Parking system is nowadays a standard feature in most new cars. It improves precision and awareness to the driver while navigating the car into parking spots or tight spaces. This goes particulary handy for vintage cars, because not only they carry a substantial value to the owners, but the outside visibility is also often very limited. For example, Lada 1300 has only a left mirror.

At the core concept, when the system is triggered, it starts a camera and graphicaly displays distances from obstacles in a clever and understandable manner.
Distances from the car are measured by 4 ultrasonic sensors evenly spaced out in a line to ensure optimal coverage. A wide angle camera is placed at an angle to capture the entire area behind the car.

For demonstration, the project utilizes 
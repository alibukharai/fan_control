# Fan Data Collection

This project involves controlling a fan using PWM (Pulse Width Modulation) and collecting data for RPM (Revolutions Per Minute) and voltage.

## Hardware

The following hardware components are required:

1. ESP32 microcontroller board.
2. 4-wire fan with a 5V operating voltage.
3. Resistor with a resistance of 3.9 ohms.

## Pin Configuration

The ESP32 board should be connected to the fan and other components as follows:

- Pin 3: This pin is used for ADC (Analog-to-Digital Conversion) to measure the voltage value.
- Pin 4: This pin is used for PWM to control the fan's speed.
- Pin 5: This pin is used for measuring the RPM of the fan.

By properly connecting the ESP32 board to the fan and configuring the mentioned pins, we can control the fan's speed using PWM and collect data on RPM and current value.


## Circuit 

       +------------------+
       |                  |       +-----------+         3.9 OHM
       |    (ESP32 S3)    |       |           |      +-----------+
       |               5V | +-----|    Fan    |+-----|  Resistor |+-----GND
       |                  |       |           |   +  +-----------+
       |    5   4   3     |       +-----+-----+   |
       +----+---------+---+        |    |         |
            |   |   |              |    |         |
            |   |   |              |    |         | 
            |   |   |              |    |         |
            |   |   +-------------  ----  ------- +                           
            |   |                  |    |      
            |   |                  |    |       
            |   |                  |    |   
            |   +-----------+-----   ---+             
            |                      |
            |                      |
            |                      |
            |     +-----------+    |
            +-----|  Resistor |+---+-----3.3 V
                  +-----------+
                     10 K 

## Inspiration 
https://www.youtube.com/watch?v=v2OxZgyD0Bs
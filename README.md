# ESP32_drone

*This Readme file is only for internal tracking. This project is not finished yet*

## Connections

| MPU9250 | ESP32 |
| --- | --- |
| SDA  | 18  |
| SCL  | 19  |


## Features
* Read Accelerometer, Gyroscope, Magnetometer and Temperature from MPU9250 and print via serial terminal

| Sensor | ODR | bits |
| --- | --- | --- |
| Accelerometer | 1 kHz | 16 |
| Gyroscope | 1 kHz | 16 |
| Magnetometer | 100 Hz | 16 |
| Temperature | 1 kHz | 16 |

* Store data in global variables and pass them as pointers to their respective functions
* Calibrate Accelerometer and Gyroscope using chip offset registers
* Madgwick Filter, obtain roll, pitch and yaw

* Generate 4 PWM signals connected to the same timer (5kHz, 8192 resolution)

| Led color | ESP32 Pin | Corner |
| --- | --- | --- |
| Red | 16 | 0 |
| Blue | 23 | 1 |
| Red | 26 | 2 |
| Blue | 32 | 3 |

## Status
**OK**

## TO DO
* Implement PI/PID controller


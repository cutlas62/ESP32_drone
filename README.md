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
| Magnetometer | 100 Hz | 16 (14 LSB)|
| Temperature | 1 kHz | 16 |

* Store data in global variables and pass them as pointers to their respective functions
* Calibrate Accelerometer and Gyroscope using chip offset registers


## Status
**OK**

## TO DO
* Calibrate magnetometer


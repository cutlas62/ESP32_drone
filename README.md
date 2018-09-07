# ESP32_drone

## Connections

| MPU9250 | ESP32 |
| --- | --- |
| SDA  | 18  |
| SCL  | 19  |


## Features
* Read Accelerometer, Gyroscope, Magnetometer and Temperature from MPU9250 and print via serial terminal
* Store data in global variables and pass them as pointers to their respective functions
* Calibrate Gyroscope using chip offset registers
* Calibrate Accelerometer manually subtracting the offset to measurements (chip registers don't seem to work)


## Status
**IN PROGRESS**

## TO DO
* Increase Magnetometer ODR?
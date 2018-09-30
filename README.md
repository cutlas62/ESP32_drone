# ESP32_drone

*This Readme file is only for internal tracking. This project is not finished yet*

## Status
**TIME MEASURE ACTIVATED**

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
* Magdwick Filter, obtain roll, pitch and yaw

* Generate 4 PWM signals connected to the same timer (5kHz, 8192 resolution)
* PID controller for Roll, Pitch and Yaw

| Led color | ESP32 Pin | Corner |
| --- | --- | --- |
| Red | 32 | 0 |
| Blue | 23 | 1 |
| Red | 26 | 2 |
| Blue | 16 | 3 |

## Time Measurements

| Component | Total time for 1000 iterations| % |
| --- | --- | --- |
| Whole loop | 1438 ms | 100% |
| Read_All function | 1364 ms | 94.85% |
| Read sensors | 1232 ms | 85.67% |
| Filter execution | 29 ms | 2.71% |
| Others | 45 ms | 3.13% |

| Component | Total time for 1000 iterations| % |
| --- | --- | --- |
| Read_All function | 1364 ms | 100% |
| Read sensors | 1232 ms | 90.32% |
| Others | 132 ms | 9.68% |

## TO DO
* Average target roll and pitch instead of one sample
* Increase controller frequency from 730 Hz (current frequency) to 1 kHz


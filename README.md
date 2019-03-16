# ESP32_drone

*This Readme file is only for internal tracking. This project is not finished yet*

## Status
**TIME MEASURE ACTIVATED**
**SOMETIMES IT DOESN'T WORK**

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
| Whole loop | 425.2 ms | 100% |
| Read_All function | 354.4 ms | 83.35% |
| Read sensors | 220.8 ms | 51.93% |
| Filter execution | 27 ms | 6.35% |
| Others | 43.8 ms | 10.3% |

| Component | Total time for 1000 iterations| % |
| --- | --- | --- |
| Read_All function | 354.4 ms | 100% |
| Read sensors | 220.8 ms | 62.3% |
| Others | 133.6 ms | 37.7% |

Maximum controller frequency = 2820 Hz

## Power Consumption Measurements

Maximum motor current = 850 mA @ 3.7 V

## TO DO
* Average target roll and pitch instead of one sample
* Read VL53LOX
* Select pressure sensor


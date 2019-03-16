#ifndef MPU9250_MAHONY_FILTER_
#define MPU9250_MAHONY_FILTER_

#include <math.h>

#define MAHONY_KP 		10.0f
#define MAHONY_KI 		0.0f

void Mahony(float * acceData, float * gyroData, float * magData, float * q, float * deltat, float * eInt);

#endif

#ifndef MPU9250_MADGWICK_FILTER_
#define MPU9250_MADGWICK_FILTER_

#include <math.h>

void Madgwick(float * acceData, float * gyroData, float * magData, float * q, float * deltat, float * beta);

#endif

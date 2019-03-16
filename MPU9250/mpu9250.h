/*
 * mpu9250.h
 *
 *  Created on: 15 mar. 2019
 *      Author: Carlos santos
 */

#ifndef MPU9250_MPU9250_H_
#define MPU9250_MPU9250_H_

#define PI 3.141592653589793238462643383279502884f
#define GyroMeasError  PI*(40.0f/180.0f)

#define MPU9250_ADDR			0x68
#define MPU9250_WHO_AM_I		0x75
#define MPU9250_FIFO_EN 		0x23
#define MPU9250_INT_ENABLE		0x38
#define MPU9250_I2C_MST_CTRL	0x24
#define MPU9250_USER_CTRL		0x6A
#define MPU9250_FIFO_COUNTH		0x72
#define MPU9250_FIFO_R_W		0x74
#define MPU9250_INT_PIN_CFG 	0x37
#define MPU9250_ACCEL_XOUT_H	0x3B
#define MPU9250_TEMP_OUT_H 		0x41
#define MPU9250_GYRO_XOUT_H 	0x43
#define MPU9250_PWR_MGMT_1		0x6B
#define MPU9250_PWR_MGMT_2 		0x6C
#define MPU9250_CONFIG			0x1A
#define MPU9250_SMPLRT_DIV		0x19
#define MPU9250_GYRO_CONFIG		0x1B
#define MPU9250_ACCE_CONFIG		0x1C
#define MPU9250_ACCE_CONFIG2	0x1D
#define MPU9250_XG_OFFSET_H     0x13
#define MPU9250_XG_OFFSET_L     0x14
#define MPU9250_YG_OFFSET_H     0x15
#define MPU9250_YG_OFFSET_L     0x16
#define MPU9250_ZG_OFFSET_H     0x17
#define MPU9250_ZG_OFFSET_L 	0x18
#define MPU9250_XA_OFFSET_H     0x77
#define MPU9250_XA_OFFSET_L     0x78
#define MPU9250_YA_OFFSET_H     0x7A
#define MPU9250_YA_OFFSET_L     0x7B
#define MPU9250_ZA_OFFSET_H     0x7D
#define MPU9250_ZA_OFFSET_L 	0x7E

#define AK8963_ADDR			0x0C
#define AK8963_CNTL 		0x0A
#define AK8963_ASAX 		0x10
#define AK8963_WHO_AM_I 	0x00
#define AK8963_ST1			0x02
#define AK8963_XOUT_L		0x03


enum Ascale {
	AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
};

enum Gscale {
	GFS_250DPS = 0, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

void MPU9250_Init();
void MPU9250_Calibrate(float * dest1, float * dest2);
void MPU9250_Read_Acce(int16_t * dest);
void MPU9250_Read_Gyro(int16_t * dest);
void MPU9250_Read_Mag(int16_t * dest);
void MPU9250_Read_Temp(float * dest);
void MPU9250_Read_ATG(void);
void MPU9250_Read_All(void);
void AK8963_Init(float * destination);
void MPU9250_GetMres();
void MPU9250_GetGres();
void MPU9250_GetAres();


#endif /* MPU9250_MPU9250_H_ */

/*
 * mpu9250.c
 *
 *  Created on: 15 mar. 2019
 *      Author: Carlos Santos
 */
#include "mpu9250.h"

#include "freertos/FreeRTOS.h"
#include <stddef.h>
#include "sdkconfig.h"


void MPU9250_Init() {
#ifdef DEBUG
	uint8_t whoami;
	I2C_Read_Byte(MPU9250_ADDR, MPU9250_WHO_AM_I, &whoami);
	printf("MPU9250: I am 0x%x and I should be 0x71\n", whoami);
#endif

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00);
	vTaskDelay(100 / portTICK_RATE_MS);

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);
	vTaskDelay(200 / portTICK_RATE_MS);

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_CONFIG, 0x03);

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_SMPLRT_DIV, 0x00);	//SMPLRT_DIV = 0, ODR = 1kHz

	uint8_t temp;
	I2C_Read_Byte(MPU9250_ADDR, MPU9250_GYRO_CONFIG, &temp);
	temp = temp & ~0x03;		//Clear GYRO_FS_SEL
	temp = temp & ~0x18;		//Clear F_CHOICE
	temp = temp | Gscale << 3;	//Set GYRO_FS_SEL
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_GYRO_CONFIG, temp);

	I2C_Read_Byte(MPU9250_ADDR, MPU9250_ACCE_CONFIG, &temp);
	temp = temp & ~0x18;		//Clear ACCEL_FS_SEL
	temp = temp | Ascale << 3;	//Set ACCEL_FS_SEL
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ACCE_CONFIG, temp);

	I2C_Read_Byte(MPU9250_ADDR, MPU9250_ACCE_CONFIG2, &temp);
	temp = temp & ~0x0F;		//Clear accel_fchoice_b and A_DLPFCFG
	temp = temp | 0x03;		//Set accel_fchoice_b to '0' and A_DLPFCFG to "011"
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ACCE_CONFIG2, temp);

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x22);
}

void MPU9250_Calibrate(float * dest1, float * dest2) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	vTaskDelay(100 / portTICK_RATE_MS);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x01);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_PWR_MGMT_2, 0x00);
	vTaskDelay(200 / portTICK_RATE_MS);

// Configure device for bias calculation
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_INT_ENABLE, 0x00); // Disable all interrupts
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_PWR_MGMT_1, 0x00); // Turn on internal clock source
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_USER_CTRL, 0x00); // Disable FIFO and I2C master modes
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_USER_CTRL, 0x0C); // Reset FIFO and DMP
	vTaskDelay(15 / portTICK_RATE_MS);

// Configure MPU6050 gyro and accelerometer for bias calculation
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_CONFIG, 0x01); // Set low-pass filter to 188 Hz
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_SMPLRT_DIV, 0x00); // Set sample rate to 1 kHz
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ACCE_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_USER_CTRL, 0x40);   // Enable FIFO
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	vTaskDelay(40 / portTICK_RATE_MS); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0] += (int32_t) gyro_temp[0];
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0] /= (int32_t) packet_count;
	gyro_bias[1] /= (int32_t) packet_count;
	gyro_bias[2] /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

// Push gyro biases to hardware registers

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_XG_OFFSET_H, data[0]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_XG_OFFSET_L, data[1]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_YG_OFFSET_H, data[2]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_YG_OFFSET_L, data[3]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ZG_OFFSET_H, data[4]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ZG_OFFSET_L, data[5]);

// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

	int16_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	int16_t mask_bit[3] = { 1, 1, 1 }; // Define array to hold mask bit for each accelerometer bias axis

	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = ((int16_t) data[0] << 8) | data[1];
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = ((int16_t) data[0] << 8) | data[1];
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = ((int16_t) data[0] << 8) | data[1];

	for (int i = 0; i < 3; i++) {
		if (accel_bias_reg[i] % 2) {
			mask_bit[i] = 0;
		}
		accel_bias_reg[i] -= accel_bias[i] >> 3; // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
		if (mask_bit[i]) {
			accel_bias_reg[i] = accel_bias_reg[i] & ~mask_bit[i]; // Preserve temperature compensation bit
		} else {
			accel_bias_reg[i] = accel_bias_reg[i] | 0x0001; // Preserve temperature compensation bit
		}
	}

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;

	// Push accelerometer biases to hardware registers

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_XA_OFFSET_H, data[0]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_XA_OFFSET_L, data[1]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_YA_OFFSET_H, data[2]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_YA_OFFSET_L, data[3]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ZA_OFFSET_H, data[4]);
	I2C_Write_Byte(MPU9250_ADDR, MPU9250_ZA_OFFSET_L, data[5]);

// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
	dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
	dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

void MPU9250_Read_Acce(int16_t * dest) {
	uint8_t raw_data[6];
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 6, &raw_data[0]);
	dest[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	dest[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	dest[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];
}

void MPU9250_Read_Gyro(int16_t * dest) {
	uint8_t raw_data[6];
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_GYRO_XOUT_H, 6, &raw_data[0]);
	dest[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	dest[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	dest[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];
}

void MPU9250_Read_Mag(int16_t * dest) {
	uint8_t raw_data[7];
	//uint8_t ST1;
	//I2C_Read_Byte(AK8963_ADDR, AK8963_ST1, &ST1);
	//if (ST1 == 0x01) {
	I2C_Read_N_Bytes(AK8963_ADDR, AK8963_XOUT_L, 7, &raw_data[0]);
	if (raw_data[6] == 0x10) { //16 bits without overflow
		dest[0] = ((int16_t) raw_data[1] << 8) | raw_data[0];
		dest[1] = ((int16_t) raw_data[3] << 8) | raw_data[2];
		dest[2] = ((int16_t) raw_data[5] << 8) | raw_data[4];
	} else {
#ifdef DEBUG
		printf("WARNING: Overflow in the magnetometer measurement.\n");
#endif
	}
	//}
}

void MPU9250_Read_Temp(float * dest) {
	uint8_t raw_data[2];
	uint16_t temp;
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_TEMP_OUT_H, 2, &raw_data[0]);
	temp = ((int16_t) raw_data[0] << 8) | raw_data[1];
	*dest = ((float) temp) / 333.87 + 21.0;
}

void MPU9250_Read_ATG(void) {
	uint8_t raw_data[14];
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_ACCEL_XOUT_H, 14, &raw_data[0]);
	acceData[0] = ((int16_t) raw_data[0] << 8) | raw_data[1];
	acceData[1] = ((int16_t) raw_data[2] << 8) | raw_data[3];
	acceData[2] = ((int16_t) raw_data[4] << 8) | raw_data[5];

	uint16_t temp = ((int16_t) raw_data[6] << 8) | raw_data[7];
	tempRealData = ((float) temp) / 333.87 + 21.0;

	gyroData[0] = ((int16_t) raw_data[8] << 8) | raw_data[9];
	gyroData[1] = ((int16_t) raw_data[10] << 8) | raw_data[11];
	gyroData[2] = ((int16_t) raw_data[12] << 8) | raw_data[13];
}

void MPU9250_Read_All(void) {

	//++++++++++++++++++++++
	uint32_t read_all_start;
	gettimeofday(&tv, NULL);
	read_all_start = 1000000 * tv.tv_sec + tv.tv_usec;
	//++++++++++++++++++++++

	/*
	 MPU9250_Read_Acce(&acceData[0]);
	 MPU9250_Read_Temp(&tempRealData);
	 MPU9250_Read_Gyro(&gyroData[0]);
	 */
	MPU9250_Read_ATG();
	MPU9250_Read_Mag(&magData[0]);

#ifdef TIME_MEASURE
	gettimeofday(&tv, NULL);
	sensor_read_time += (1000000 * tv.tv_sec + tv.tv_usec) - read_all_start;
#endif

	for (int i = 0; i < 3; i++) {
		acceRealData[i] = acceData[i] * aRes;
		gyroRealData[i] = gyroData[i] * gRes * PI / 180.0;
		//gyroRealData[i] = gyroData[i] * gRes;
		magRealData[i] = magData[i] * magCalibration[i] * mRes;
	}
	/*Magnetometer absolute ratings
	 xMax = 730.02
	 xmin = -400.62
	 yMax = 669.48
	 ymin =-494.99
	 zMax = 344.94
	 zmin = -741.36
	 */
	magRealData[0] -= 165;
	magRealData[1] -= 105;
	magRealData[2] += 220;

	Now = getMicros();
	deltat = ((Now - lastTime) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastTime = Now;

#ifdef TIME_MEASURE
	gettimeofday(&tv, NULL);
	read_all_start = 1000000 * tv.tv_sec + tv.tv_usec;
#endif

	//Mahony(&acceRealData[0], &gyroRealData[0], &magRealData[0], &q[0], &deltat, &eInt[0]);
	Madgwick(&acceRealData[0], &gyroRealData[0], &magRealData[0], &q[0],
			&deltat, &beta);

#ifdef TIME_MEASURE
	gettimeofday(&tv, NULL);
	filter_time += (1000000 * tv.tv_sec + tv.tv_usec) - read_all_start;
#endif
	//printf("q[0] = %f\nq[1] = %f\nq[2] = %f\nq[3] = %f\n\n", q[0], q[1],q[2], q[3]);

	yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
			q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	roll = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	pitch = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
			q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	roll *= 180.0f / PI;
	yaw *= 180.0f / PI;
	yaw -= 4; // Declination at Chicago
	pitch *= 180.0f / PI;

	//printf("yaw = %.2f\npitch = %.2f\nroll = %.2f\n\n", yaw, pitch, roll);

}

void AK8963_Init(float * destination) {
#ifdef DEBUG
	uint8_t whoami;
	I2C_Read_Byte(AK8963_ADDR, AK8963_WHO_AM_I, &whoami);
	printf("AK8963: I am 0x%x and I should be 0x48\n", whoami);
#endif

	uint8_t rawData[3];
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x00);		//Power-down mode
	vTaskDelay(10 / portTICK_RATE_MS);
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x0F);		//Fuse ROM access mode
	vTaskDelay(10 / portTICK_RATE_MS);
	I2C_Read_N_Bytes(AK8963_ADDR, AK8963_ASAX, 3, &rawData[0]);
	destination[0] = (float) (rawData[0] - 128) / 256. + 1.;
	destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
	destination[2] = (float) (rawData[2] - 128) / 256. + 1.;

#ifdef DEBUG
	printf("magCalibration[0] = %f\n", magCalibration[0]);
	printf("magCalibration[1] = %f\n", magCalibration[1]);
	printf("magCalibration[2] = %f\n", magCalibration[2]);
#endif

	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x00);		//Power-down mode
	vTaskDelay(10 / portTICK_RATE_MS);
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, Mscale << 4 | Mmode);
	vTaskDelay(10 / portTICK_RATE_MS);
}

void MPU9250_GetMres() {
	switch (Mscale) {
	case MFS_14BITS:
		mRes = 10.0 * 4912. / 8190.0;
		break;
	case MFS_16BITS:
		mRes = 10.0 * 4912. / 32760.0;
		break;
	}
}

void MPU9250_GetGres() {
	switch (Gscale) {
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void MPU9250_GetAres() {
	switch (Ascale) {
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}



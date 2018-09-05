//**********************************************************************
//Includes
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

//**********************************************************************
//Defines
#define I2C_NUM 			1
#define I2C_SDA_IO			18
#define I2C_SCL_IO			19
#define I2C_FREQ_HZ			100000
#define I2C_RX_BUF_DISABLE	0
#define I2C_TX_BUF_DISABLE	0
#define WRITE_BIT  			I2C_MASTER_WRITE
#define READ_BIT 			I2C_MASTER_READ
#define ACK_CHECK_EN 		0x1
#define ACK_CHECK_DIS 		0x0
#define ACK_VAL 			0x0
#define NACK_VAL 			0x1

#define MPU9250_ADDR			0x68
#define MPU9250_WHO_AM_I		0x75
#define MPU9250_INT_PIN_CFG 	0x37
#define MPU9250_ACCEL_XOUT_H	0x3B
#define MPU9250_TEMP_OUT_H 		0x41
#define MPU9250_GYRO_XOUT_H 	0x43

#define AK8963_ADDR			0x0C
#define AK8963_CNTL 		0x0A
#define AK8963_ASAX 		0x10
#define AK8963_WHO_AM_I 	0x00
#define AK8963_ST1			0x02
#define AK8963_XOUT_L		0x03

//**********************************************************************
//Global Variables
int16_t acceData[3];	//Store the 16-bit signed accelerometer data
int16_t gyroData[3];	//Store the 16-bit signed gyroscope data
int16_t magData[3];		//Store the 16-bit signed magnetometer data
int16_t tempData;		//Store the 16-bit signed temperature data

//**********************************************************************
//Auxiliary Functions
esp_err_t I2C_Write_Byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t regData) {
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, regData, ACK_CHECK_DIS);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t I2C_Read_Byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t* regData) {
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		return ret;
	}
	vTaskDelay(30 / portTICK_RATE_MS);
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | READ_BIT, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, regData, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

esp_err_t I2C_Read_N_Bytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t nBytes,
		uint8_t* buffer) {
	int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		return ret;
	}
	vTaskDelay(30 / portTICK_RATE_MS);
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | READ_BIT, ACK_CHECK_EN);
	for (int i = 0; i < nBytes - 1; i++) {
		i2c_master_read_byte(cmd, buffer++, ACK_VAL);
	}
	i2c_master_read_byte(cmd, buffer, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}

void I2C_Init() {
	int i2c_master_port = I2C_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_FREQ_HZ;
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode,
	I2C_RX_BUF_DISABLE,
	I2C_TX_BUF_DISABLE, 0);
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
	uint8_t ST1;
	I2C_Read_Byte(AK8963_ADDR, AK8963_ST1, &ST1);
	//if (ST1 == 0x01) {
		I2C_Read_N_Bytes(AK8963_ADDR, AK8963_XOUT_L, 7, &raw_data[0]);
		//if (raw_data[6] != 0x08) {
			dest[0] = ((int16_t) raw_data[1] << 8) | raw_data[0];
			dest[1] = ((int16_t) raw_data[3] << 8) | raw_data[2];
			dest[2] = ((int16_t) raw_data[5] << 8) | raw_data[4];
		//}
	//}
}

void MPU9250_Read_Temp(int16_t * dest) {
	uint8_t raw_data[2];
	I2C_Read_N_Bytes(MPU9250_ADDR, MPU9250_TEMP_OUT_H, 2, &raw_data[0]);
	*dest = ((int16_t) raw_data[0] << 8) | raw_data[1];
}

void MPU9250_Read_All() {
	while (1) {
		MPU9250_Read_Acce(&acceData[0]);
		MPU9250_Read_Gyro(&gyroData[0]);
		MPU9250_Read_Mag(&magData[0]);
		MPU9250_Read_Temp(&tempData);
		printf("AccelX = %d\nAccelY = %d\nAccelZ = %d\n",acceData[0],acceData[1],acceData[2]);
		printf("GyroX = %d\nGyroY = %d\nGyroZ = %d\n",gyroData[0],gyroData[1],gyroData[2]);
		printf("MagX = %d\nMagY = %d\nMagZ = %d\n",magData[0],magData[1],magData[2]);
		printf("Temp = %d\n\n",tempData);
		vTaskDelay(50 / portTICK_RATE_MS);
	}
}

void AK8963_Init(float * destination) {
	uint8_t rawData[3];
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x00);
	vTaskDelay(10 / portTICK_RATE_MS);
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x0F);
	vTaskDelay(10 / portTICK_RATE_MS);
	I2C_Read_N_Bytes(AK8963_ADDR, AK8963_ASAX, 3, &rawData[0]);
	destination[0] = (float) (rawData[0] - 128) / 256. + 1.;
	destination[1] = (float) (rawData[1] - 128) / 256. + 1.;
	destination[2] = (float) (rawData[2] - 128) / 256. + 1.;
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x00);
	vTaskDelay(10 / portTICK_RATE_MS);
	I2C_Write_Byte(AK8963_ADDR, AK8963_CNTL, 0x16);
	vTaskDelay(10 / portTICK_RATE_MS);
}

//**********************************************************************
//Main App
void app_main() {
	float magData[3];
	uint8_t whoami;

	I2C_Init();

	I2C_Write_Byte(MPU9250_ADDR, MPU9250_INT_PIN_CFG, 0x22);

	I2C_Read_Byte(MPU9250_ADDR, MPU9250_WHO_AM_I, &whoami);
	printf("MPU9250: I am 0x%x and I should be 0x71\n", whoami);

	whoami = 0;
	I2C_Read_Byte(AK8963_ADDR, AK8963_WHO_AM_I, &whoami);
	printf("AK8963: I am 0x%x and I should be 0x48\n", whoami);

	AK8963_Init(&magData[0]);
	printf("magData[0] = %f\n", magData[0]);
	printf("magData[1] = %f\n", magData[1]);
	printf("magData[2] = %f\n", magData[2]);

	xTaskCreate(MPU9250_Read_All, "MPU9250_Read_All", 1024 * 2, NULL, 5, NULL);

}

/*
 * m_i2c.h
 *
 *  Created on: 15 mar. 2019
 *      Author: Carlos Santos
 */

#ifndef MAIN_I2C_M_I2C_H_
#define MAIN_I2C_M_I2C_H_

#define I2C_NUM 			1
#define I2C_SDA_IO			18
#define I2C_SCL_IO			19
#define I2C_FREQ_HZ			400000
#define I2C_RX_BUF_DISABLE	0
#define I2C_TX_BUF_DISABLE	0
#define WRITE_BIT  			I2C_MASTER_WRITE
#define READ_BIT 			I2C_MASTER_READ
#define ACK_CHECK_EN 		0x1
#define ACK_CHECK_DIS 		0x0
#define ACK_VAL 			0x0
#define NACK_VAL 			0x1

void I2C_Init(void);
esp_err_t I2C_Write_Byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t regData);
esp_err_t I2C_Read_Byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t* regData);
esp_err_t I2C_Read_N_Bytes(uint8_t deviceAddr, uint8_t regAddr, uint8_t nBytes,
		uint8_t* buffer);

#endif /* MAIN_I2C_M_I2C_H_ */

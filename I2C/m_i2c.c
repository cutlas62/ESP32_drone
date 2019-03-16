/*
 * m_i2c.c
 *
 *  Created on: 15 mar. 2019
 *      Author: Carlos Santos
 */

#include "m_i2c.h"
//#include "freertos/task.h"
#include "driver/i2c.h"



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

#ifdef DEBUG
	printf("I2C%d successfully initialized\n", I2C_NUM);
#endif
}

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
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		return ret;
	}
	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | READ_BIT, ACK_CHECK_EN);
	if (nBytes > 1) {
		i2c_master_read(cmd, buffer, nBytes - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, buffer + nBytes - 1, NACK_VAL);
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_NUM, cmd, 100 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	return ret;
}


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define DATA_LENGTH 					512
#define DELAY_TIME_BETWEEN_ITEMS_MS   	1000

#define I2C_EXAMPLE_MASTER_NUM 				1
#define I2C_EXAMPLE_MASTER_SDA_IO			18
#define I2C_EXAMPLE_MASTER_SCL_IO			19
#define I2C_EXAMPLE_MASTER_FREQ_HZ			100000
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE	0
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE	0

#define MPU9250_ADDR	0x68
#define WRITE_BIT  		I2C_MASTER_WRITE
#define READ_BIT 		I2C_MASTER_READ
#define ACK_CHECK_EN 	0x1
#define ACK_CHECK_DIS 	0x0
#define ACK_VAL 		0x0
#define NACK_VAL 		0x1

#define SI7020_CMD_READ    0xE0

void I2C_Write_Byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t regData) {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, deviceAddr << 1 | WRITE_BIT, ACK_CHECK_DIS);
	i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, regData, NACK_VAL);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM,cmd,1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
}

esp_err_t I2C_Read_Byte(uint8_t deviceAddr, uint8_t regAddr, uint8_t* regData){
	int ret;
	    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	    i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, deviceAddr << 1 | WRITE_BIT, ACK_CHECK_EN);
	    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
	    i2c_master_stop(cmd);
	    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
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
	    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
	    i2c_cmd_link_delete(cmd);
	return ret;
}


void i2c_example_master_init() {
	int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
	i2c_param_config(i2c_master_port, &conf);
	i2c_driver_install(i2c_master_port, conf.mode,
	I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
	I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}



void app_main() {
	i2c_example_master_init();
	uint8_t whoami;
	I2C_Read_Byte(MPU9250_ADDR, 0x75, &whoami);
	printf("Whoami = %d\n", whoami);
}

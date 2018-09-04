#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

//#define BLINK_GPIO 22
//#define BLINK_DELAY 1000

uint8_t redPin = 23;
uint8_t greenPin = 18;
uint8_t yellowPin = 19;

void blink_task(void * pinNumber) {
	gpio_pad_select_gpio(*((uint8_t*)pinNumber));
	gpio_set_direction(*((uint8_t*)pinNumber), GPIO_MODE_OUTPUT);
	while (1) {
		gpio_set_level(*((uint8_t*)pinNumber), 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS / 2);
		gpio_set_level(*((uint8_t*)pinNumber), 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS / 2);
	}
}

void app_main() {
	xTaskCreate(&blink_task, "blink_task_yellow", configMINIMAL_STACK_SIZE, (void*)&yellowPin, 5, NULL);
	vTaskDelay(166 / portTICK_PERIOD_MS);
	xTaskCreate(&blink_task, "blink_task_red", configMINIMAL_STACK_SIZE, (void*)&redPin, 5, NULL);
	vTaskDelay(166 / portTICK_PERIOD_MS);
	xTaskCreate(&blink_task, "blink_task_green", configMINIMAL_STACK_SIZE, (void*)&greenPin, 5, NULL);
}

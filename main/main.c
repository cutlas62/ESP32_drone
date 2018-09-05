#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

struct led {
	uint8_t pinNumber;
	uint16_t tickPeriod;
} led;

struct led redLed = {23,500};
struct led greenLed = {18,500};
struct led yellowLed = {19,500};

void blink_task(void * pvParameters) {
	struct led *_led;
	_led = (struct led*) pvParameters;


	gpio_pad_select_gpio((*_led).pinNumber);
	gpio_set_direction((*_led).pinNumber, GPIO_MODE_OUTPUT);
	while (1) {
		gpio_set_level((*_led).pinNumber, 0);
		vTaskDelay((*_led).tickPeriod / portTICK_PERIOD_MS / 2);
		gpio_set_level((*_led).pinNumber, 1);
		vTaskDelay((*_led).tickPeriod / portTICK_PERIOD_MS / 2);
	}
}


void app_main() {

	xTaskCreate(&blink_task, "blink_task_yellow", configMINIMAL_STACK_SIZE, (void*)&yellowLed, 5, NULL);
	vTaskDelay(84 / portTICK_PERIOD_MS);
	xTaskCreate(&blink_task, "blink_task_red", configMINIMAL_STACK_SIZE, (void*)&redLed, 5, NULL);
	vTaskDelay(84 / portTICK_PERIOD_MS);
	xTaskCreate(&blink_task, "blink_task_green", configMINIMAL_STACK_SIZE, (void*)&greenLed, 5, NULL);
}

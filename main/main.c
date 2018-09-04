#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BLINK_GPIO 22
#define BLINK_DELAY 100

void blink_task(void) {
	gpio_pad_select_gpio(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	while (1) {
		gpio_set_level(BLINK_GPIO, 0);
		vTaskDelay(BLINK_DELAY / portTICK_PERIOD_MS);
		gpio_set_level(BLINK_GPIO, 1);
		vTaskDelay(BLINK_DELAY / portTICK_PERIOD_MS);
	}
}

void app_main() {
	xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}

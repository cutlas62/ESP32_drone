/*
 * m_pwm.c
 *
 *  Created on: 15 mar. 2019
 *      Author: Carlos Santos
 */
#include "m_pwm.h"
#include <stdint.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

ledc_timer_config_t ledc_timer;
ledc_channel_config_t ledc_channel[4];

void PWM_Init() {
	ledc_timer_config_t ledc_timer = { .duty_resolution = LEDC_TIMER_13_BIT,
			.freq_hz = LEDC_HS_FREQ, .speed_mode = LEDC_HS_MODE, .timer_num =
			LEDC_HS_TIMER };
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t ledc_channel[4] = { { .channel = LEDC_HS_CH0_CHANNEL,
			.duty = 500, .gpio_num = LEDC_HS_CH0_GPIO, .speed_mode =
			LEDC_HS_MODE, .timer_sel = LEDC_HS_TIMER }, { .channel =
	LEDC_HS_CH1_CHANNEL, .duty = 500, .gpio_num = LEDC_HS_CH1_GPIO,
			.speed_mode = LEDC_HS_MODE, .timer_sel = LEDC_HS_TIMER }, {
			.channel = LEDC_HS_CH2_CHANNEL, .duty = 500, .gpio_num =
			LEDC_HS_CH2_GPIO, .speed_mode = LEDC_HS_MODE, .timer_sel =
			LEDC_HS_TIMER }, { .channel = LEDC_HS_CH3_CHANNEL, .duty = 500,
			.gpio_num = LEDC_HS_CH3_GPIO, .speed_mode = LEDC_HS_MODE,
			.timer_sel = LEDC_HS_TIMER }, };

	for (int ch = 0; ch < 4; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}
	vTaskDelay(500 / portTICK_RATE_MS);
}

void PWM_Set_Duty(int16_t * duty) {
	ledc_channel_config_t ledc_channel[4] = { { .channel = LEDC_HS_CH0_CHANNEL,
			.duty = duty[0], .gpio_num = LEDC_HS_CH0_GPIO, .speed_mode =
			LEDC_HS_MODE, .timer_sel = LEDC_HS_TIMER }, { .channel =
	LEDC_HS_CH1_CHANNEL, .duty = duty[1], .gpio_num = LEDC_HS_CH1_GPIO,
			.speed_mode = LEDC_HS_MODE, .timer_sel = LEDC_HS_TIMER }, {
			.channel = LEDC_HS_CH2_CHANNEL, .duty = duty[2], .gpio_num =
			LEDC_HS_CH2_GPIO, .speed_mode = LEDC_HS_MODE, .timer_sel =
			LEDC_HS_TIMER }, { .channel = LEDC_HS_CH3_CHANNEL, .duty = duty[3],
			.gpio_num = LEDC_HS_CH3_GPIO, .speed_mode = LEDC_HS_MODE,
			.timer_sel = LEDC_HS_TIMER }, };

	for (int ch = 0; ch < 4; ch++) {
		ledc_channel_config(&ledc_channel[ch]);
	}
}


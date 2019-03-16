/*
 * m_pwm.h
 *
 *  Created on: 15 mar. 2019
 *      Author: Carlos Santos
 */

#ifndef PWM_PWM_H_
#define PWM_PWM_H_

#define LEDC_HS_TIMER          	LEDC_TIMER_0
#define LEDC_HS_MODE           	LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       	32
#define LEDC_HS_CH0_CHANNEL 	LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       	23
#define LEDC_HS_CH1_CHANNEL 	LEDC_CHANNEL_1
#define LEDC_HS_CH2_GPIO       	26
#define LEDC_HS_CH2_CHANNEL 	LEDC_CHANNEL_2
#define LEDC_HS_CH3_GPIO       	16
#define LEDC_HS_CH3_CHANNEL 	LEDC_CHANNEL_3
#define LEDC_HS_FREQ			5000
#define LEDC_HS_MAX_DUTY		8192

void PWM_Init();
void PWM_Set_Duty(int16_t * duty);

#endif /* PWM_PWM_H_ */

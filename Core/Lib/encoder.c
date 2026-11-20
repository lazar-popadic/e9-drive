/*
 * encoder.c
 *
 *  Created on: Nov 18, 2025
 *      Author: Lea
 */

#include "main.h"
#include "tim.h"

static int16_t cnt_difference(TIM_HandleTypeDef *htim);

double enc_velocity(TIM_HandleTypeDef *htim, double dt,
		uint16_t impulse_per_2Pi) {
	double vel, ang;
	int16_t diff = cnt_difference(htim);
	ang = (double) diff / (double) impulse_per_2Pi * 2 * M_PI; // predjeni ugao u radijanima
	vel = ang / dt;
	return vel;
}

static int16_t cnt_difference(TIM_HandleTypeDef *htim) {
	int16_t diff = (int16_t) (htim->Instance->CNT);
	htim->Instance->CNT = 0;
	return diff;
}

void enc_start(TIM_HandleTypeDef *htim) {
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

void enc_init() {
	enc_start(&htim3);
	enc_start(&htim5);
}


/*
 * servo_fxns.c
 *
 *  Created on: Dec 24, 2025
 *      Author: ryansundermeyer
 */

#include "servo_fxns.h" /* PUT ME IN: USER CODE BEGIN Includes */

extern TIM_HandleTypeDef htim2; // variables we need from main

// fully rotate servo counter clockwise
void rotate_ccw(){ /* PUT ME IN: USER CODE BEGIN 3 */
	uint32_t pulse_width = 2500; // 2.5 ms pulse

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width); // set new pulse
}

// fully rotate servo clockwise
void rotate_cw(){ /* PUT ME IN: USER CODE BEGIN 3 */
	uint32_t pulse_width = 500; // 0.5 ms pulse

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width); // set new pulse
}

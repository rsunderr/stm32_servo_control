/*
 * servo_fxns.c
 *
 *  Created on: Dec 24, 2025
 *      Author: ryansundermeyer
 */

#include "servo_fxns.h" /* PUT ME IN: USER CODE BEGIN Includes */

extern TIM_HandleTypeDef htim2; // variables we need from main

// rotate servo counter clockwise
void rotate_ccw(){ /* PUT ME IN: USER CODE BEGIN 3 */
	static uint32_t counter = 0;
	static uint32_t pulse_width = 500;

	if (pulse_width < 2500 && counter % 10 == 0) { // max width 2500, only run every 10 cycles
		pulse_width += 50; // increase pulse width
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width); // set new pulse
	}
	if (pulse_width >= 2500) {
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // turn off PWM when finished
	}
	else {
		counter++;
	}

}

// rotate servo clockwise
void rotate_cw(){ /* PUT ME IN: USER CODE BEGIN 3 */
	static uint32_t counter = 0;
	static uint32_t pulse_width = 2500;

	if (pulse_width > 500 && counter % 10 == 0) { // min width 500, only run every 10 cycles
		pulse_width -= 50; // decrease pulse width
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width); // set new pulse
	}
	if (pulse_width <= 500) {
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // turn off PWM when finished
	}
	else {
		counter++;
	}
}


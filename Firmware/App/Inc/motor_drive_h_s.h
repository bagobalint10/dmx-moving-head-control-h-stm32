/*
 * motor_drive.h
 *
 * Created: 2025. 10. 14. 16:32:55
 *  Author: bagob
 */ 

#ifndef MOTOR_DRIVE_H_
#define MOTOR_DRIVE_H_

	#include "stm32f4xx_hal.h"

	typedef struct
	{
		// motorvezerles
		uint32_t max_speed_level;
		int32_t current_level;
		int32_t input_pos;
		int32_t current_pos;
		int32_t pos_diff;
		uint8_t direction;
		uint8_t motor_enable;

		// sebesseg szamolas
		uint32_t n_max;
		float v_max;
		float v_start;
		float k_mul;
		float r_max;
		uint32_t d_t;

		// időzítés
		uint32_t current_time;
		uint32_t prev_time;
		uint32_t interval;

		// microstep
		int16_t microstep_pos;
		uint8_t pin_1;
		uint8_t pin_2;

	} MOTOR_TypeDef;

	void motor_main(MOTOR_TypeDef *Motor, int16_t dmx_pos_1, float dmx_speed);
	void motor_update_timer(MOTOR_TypeDef *Motor);
	void motor_set_0_pos(MOTOR_TypeDef *Motor);

#endif /* MOTOR_DRIVE_H_ */

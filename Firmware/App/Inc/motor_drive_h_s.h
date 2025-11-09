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
	// globális
	uint32_t max_speed_level; //= 30000;	// ez majd lehet nem kell -- ideiglenesen
	int32_t current_level; //= 0;
	int32_t input_pos; //= 0;				// bemenet --> majd �ttenni 32 bitesre �ket
	int32_t current_pos; //= MAX_POS;		// indulásnál max érték
	int32_t pos_diff; //= 0;
	uint8_t direction; //= 0;
	uint8_t motor_enable; //= 0;

	uint32_t n_max; //= 30000; 				// min 8000 - max 30000
	float v_max; //= 80.0f; 				// min 20 - max 80
	float v_start; //= 1.5f; 				// min 0.2 - max 1.5  --> 10ms
	float k_mul;   //= 0;
	float r_max;   //= 0;  					// hogy a végeredmény 0-1 közé essen

	//motor_1_update_timer()
	uint32_t d_t; //= 0;
	// időzítés
	uint32_t current_time;
	uint32_t prev_time;
	uint32_t interval;
	// microstep cuccok
	int16_t microstep_pos;

} MOTOR_TypeDef;

	void motor_1_main(MOTOR_TypeDef *Motor, uint16_t dmx_pos_1, float dmx_speed);
	void motor_1_update_timer(MOTOR_TypeDef *Motor);
	void motor_1_set_0_pos(MOTOR_TypeDef *Motor);

#endif /* MOTOR_DRIVE_H_ */

/*
 * motor_drive.c
 *
 * Created: 2025. 10. 14. 16:33:07
 *  Author: bagob
 */ 

#include <motor_drive_h_s.h>

#include "main.h"
#include "math.h"

#define ZERO_POS 100
#define MAX_POS 100000UL
#define T_MAX 109.54451f
//#define DT_0 840000UL 			// 10ms
#define DT_0 84000UL 				// 1ms
//#define TIMER_CONST 644095.0f		// 1/16 max speed --> dt = 70us
#define TIMER_CONST 322047.0f		// 1/32 max speed --> dt = 70us
#define SYMMETRY_CONST 0.5f 		//0.1-0.5

static uint16_t microstep_values[32] = {2000, 2390, 2765, 3111, 3414, 3663, 3848, 3962, 4000, 3962, 3848, 3663, 3414, 3111, 2765, 2390, 2000, 1610, 1235, 889, 586, 337, 152, 38, 0, 38, 152, 337, 586, 889, 1235, 1610};

static uint32_t  calculate_time(MOTOR_TypeDef *Motor, uint32_t n)
{
	float v_s;

	if(!n) return 0; 								//0 sebességnál 0
	else if(n > Motor->n_max) v_s = Motor->v_max ;	//max sebesség felett = max
	else
	{
	 float k 	= ((float)(Motor->n_max - n) * Motor->k_mul) + 1.0f;		// korrigáció
	 float r_n 	= sqrtf((float)n) * k / Motor->r_max;    					// gyök n + korrigáció
	 v_s 		= ((3.0f*r_n*r_n)-( 2.0f*r_n*r_n*r_n)) * Motor->v_max;		// s görbe számítás
	}

	v_s += Motor->v_start;
	float dt = 1.0f / v_s;

	dt *= TIMER_CONST;

	return (uint32_t) dt;
}

static void calculate_speed(MOTOR_TypeDef *Motor, float speed)
{
	Motor->v_max = speed;
	Motor->v_start = 0.0f;

	Motor->k_mul   = (SYMMETRY_CONST / (float)Motor->n_max);
	Motor->r_max   = sqrtf((float)Motor->n_max);
	Motor->max_speed_level = Motor->n_max;
}

void motor_update_timer(MOTOR_TypeDef *Motor)
{
	// időzítés
	if ((uint32_t)(Motor->current_time - Motor->prev_time) < Motor->interval)
	{
		return;
	}

	Motor->prev_time = Motor->current_time;

	// motor sebedseg szabalyzas
	if(Motor->motor_enable && Motor->direction)
	{
		Motor->pos_diff = (Motor->input_pos - Motor->current_pos)-1;

		if ((Motor->pos_diff > Motor->current_level) && (Motor->current_level < Motor->max_speed_level))
		{
			Motor->current_level++;
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}
		else if((Motor->pos_diff < Motor->current_level) && (Motor->current_level > 0))
		{
			Motor->current_level--;
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}

		Motor->current_pos++;
		Motor->microstep_pos++;

		if(!Motor->d_t) Motor->d_t = DT_0;

		Motor->interval = (Motor->d_t)/100000;
	}
	else if(Motor->motor_enable && (!Motor->direction))
	{
		Motor->pos_diff = (Motor->current_pos - Motor->input_pos)-1 ;

		if ((Motor->pos_diff > Motor->current_level) && (Motor->current_level < Motor->max_speed_level))
		{
			Motor->current_level++;
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}
		else if((Motor->pos_diff < Motor->current_level) && (Motor->current_level > 0))
		{
			Motor->current_level--;
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}

		Motor->current_pos--;
		Motor->microstep_pos--;

		if(!Motor->d_t) Motor->d_t = DT_0;
		Motor->interval = ((Motor->d_t)/100000);
	}

	// current_pos ovf
	if(Motor->current_pos > 1599) Motor->current_pos = 0;
	else if(Motor->current_pos < 0) Motor->current_pos = 1599;

	// microstep ovf
	if(Motor->microstep_pos > 31) Motor->microstep_pos = 0;
	else if(Motor->microstep_pos < 0) Motor->microstep_pos = 31;

	// set pwm duty
	int16_t microstep_pos_2 = ((Motor->microstep_pos) + 8);
	if(microstep_pos_2 > 31) microstep_pos_2 = microstep_pos_2 - 32;

	set_pwm_duty(Motor->pin_1, microstep_values[Motor->microstep_pos]);
	set_pwm_duty(Motor->pin_2, microstep_values[microstep_pos_2]);

	if ((!Motor->current_level) && Motor->motor_enable)
	{
		Motor->motor_enable = 0;
	}
	HAL_GPIO_WritePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin, 0);
}

void motor_main(MOTOR_TypeDef *Motor, int16_t dmx_pos_1, float dmx_speed)
{
	uint16_t motor_enable_buf;

	Motor->input_pos =((int32_t)dmx_pos_1 + ZERO_POS);
	motor_enable_buf = Motor->motor_enable;

	if((!motor_enable_buf) && (Motor->input_pos != Motor->current_pos))
	{	
		if(Motor->input_pos > Motor->current_pos)	Motor->direction = 1;	// pozitiv  // irany
		else 										Motor->direction = 0;	// negat�v

		Motor->motor_enable = 1;
		calculate_speed(Motor, dmx_speed);
	}
}

void motor_set_0_pos(MOTOR_TypeDef *Motor)
{
	Motor->current_pos = ZERO_POS;
	Motor->current_level = 1;
}


 

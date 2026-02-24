/*
 * my_main.c
 *
 *  Created on: Oct 18, 2025
 *      Author: bagob
 */

#include <my_main.h>

#include "main.h"
#include <motor_drive_h_s.h>
#include <Services/Inc/dmx_usart_s.h>

#define MAX_POS 100000UL
#define FALLING_EDGE(val_cont) (((val_cont) & 0x03) == 0x02)

typedef struct
	{
		int16_t pos;
		float speed;
		uint8_t motor_reset_f;

	} Input_parameters;

typedef struct
	{
		Input_parameters input_parameters;
		MOTOR_TypeDef driver_parameters;

	} Motors;

enum  Motor_ids
	{
		MOTOR_1,
		MOTOR_2,
		MOTOR_3,
		MOTOR_4,
		MOTOR_5,
		MOTOR_6,
		MOTOR_7,
		MOTOR_8,
		MOTOR_COUNT
	};

enum Motor_limit_type {HALL_SENSOR, BLOCKING};

typedef struct
	{
		char name[20];
		uint8_t limit_type;
		uint8_t pin1, pin2;
		int max_speed;
		float start_speed;

	} Motor_Config;

static Motor_Config motor_config[MOTOR_COUNT] =
	{
		[MOTOR_1] = {.name = "color_wheel", 	.limit_type = HALL_SENSOR, 	.pin1 = 0, 	.pin2 = 1, 	.max_speed = 200},
		[MOTOR_2] = {.name = "shutter_1", 		.limit_type = BLOCKING, 	.pin1 = 2, 	.pin2 = 3, 	.max_speed = 25	},
		[MOTOR_3] = {.name = "prism_rotate", 	.limit_type = BLOCKING,		.pin1 = 4, 	.pin2 = 5, 	.max_speed = 500},
		[MOTOR_4] = {.name = "prism_switch", 	.limit_type = HALL_SENSOR, 	.pin1 = 6, 	.pin2 = 7, 	.max_speed = 50	},
		[MOTOR_5] = {.name = "focus", 			.limit_type = BLOCKING, 	.pin1 = 8, 	.pin2 = 9, 	.max_speed = 100},
		[MOTOR_6] = {.name = "shutter_2", 		.limit_type = BLOCKING, 	.pin1 = 11,	.pin2 = 10, .max_speed = 25	},
		[MOTOR_7] = {.name = "gobo_wheel", 		.limit_type = HALL_SENSOR, 	.pin1 = 12, .pin2 = 13, .max_speed = 200},
		[MOTOR_8] = {.name = "gobo_rotate", 	.limit_type = HALL_SENSOR, 	.pin1 = 14, .pin2 = 15, .max_speed = 200},

	};

typedef struct
	{
		uint8_t value_container;
		GPIO_TypeDef* GPIO_Port;
		uint16_t GPIO_pin;

	} Hall_Sensor;

static Motors motors[MOTOR_COUNT] = {0};


static void prism_control(int16_t* pos)
{
	if(*pos > 125) 	*pos = 257;
	else 			*pos = 0;
}

static void focus_control(int16_t* pos)
{
	float tmp;

	tmp = 335.0f/255.0f;
	tmp *= (*pos);
	*pos = (int16_t)tmp;
}

static void dimm_control(int16_t* pos_a, int16_t* pos_b)
{
	float tmp;
	int16_t tmp_pos = *pos_b;

	tmp = tmp_pos;

	tmp = 100.0f/255.0f;
	tmp *= *pos_b;
	*pos_b = (int16_t)tmp;

	tmp = 75.0f/255.0f;
	tmp *= tmp_pos;
	*pos_a = (int16_t)tmp;
}

static void strobe_control(int16_t* pos_a, int16_t* pos_b, uint16_t input_pos)
{
	static uint8_t strobe_f = 0;

	static uint32_t strobe_interval;
	static uint32_t current_time = 0;
	static uint32_t prev_time = 0;

	float tmp;

	if(input_pos > 55)
	{
		tmp = 9.0f / 201.0f;
		tmp *= ((float)input_pos - 55.0f);
		tmp += 1.0f;

		current_time = HAL_GetTick();
		strobe_interval = (uint32_t)(100.0f * (11.0f - tmp));

		if ((uint32_t)(current_time - prev_time)>= strobe_interval)
		{
			prev_time = current_time;
			strobe_f = 1;
		}

		if(strobe_f && (motors[MOTOR_2].driver_parameters.current_pos == ((*pos_b) + 100)))
		{
			strobe_f = 0;
		}

		if(!strobe_f)
		{
			*pos_a = 0;
			*pos_b = 0;
		}
	}
}

static void prism_rotate_control(int16_t* pos, float* speed )
{
	int16_t tmp_pos = *pos;
	float tmp;

	if(*pos > 55)
	{
		if(*pos>155)
		{
			*pos = 3000;

			tmp = 4.5f/100.0f;
			tmp *= ((float)tmp_pos - 155.0f);
			tmp += 0.5f;
			*speed = tmp;
		}
		else
		{
			*pos = -3000;

			tmp = 4.5f/100.0f;
			tmp *=  ((float)tmp_pos - 55.0f);
			tmp += 0.5f;
			*speed = tmp;
		}
	}
	else
	{
		*pos = 0;
		*speed = 5.0f;
	}
}

static void gobo_rotate_control(int16_t* pos, float* speed)
{
	int16_t tmp_pos = *pos;
	float tmp;

	if(*pos > 55)
	{
		if(*pos > 155)
		{
			*pos = 3000;

			tmp = 1.0f/100.0f;
			tmp *= ((float)tmp_pos - 155.0f);
			tmp += 0.5f;
			*speed = tmp;
		}
		else
		{
			*pos = -3000;

			tmp = 1.0f/100.0f;
			tmp *=  ((float)tmp_pos - 55.0f);
			tmp += 0.5f;
			*speed = tmp;
		}
	}
	else
	{
		tmp = 800.0f/55.0f;
		tmp *= *pos;
		*pos = (int16_t)tmp;
		*speed = 1.5f;
	}
}

static void colorwheel_control(int16_t* pos)
{
	static uint16_t colorwheel_array[12] = {128, 261, 394, 527, 660, 793, 926, 1059,1192, 1325, 1458, 1591};
	float tmp;

	if(*pos <= 200)
		{
			tmp = 11.0f/201.0f;
			tmp *= *pos;
			*pos = colorwheel_array[(int8_t)tmp];
		}
		else if(*pos < 225)
		{
			*pos = -3000;
		}
		else
		{
			*pos = 3000;
		}
}

static void gobowheel_control(int16_t* pos)
{
	float tmp;
	uint8_t tmp_2 = 0;

	tmp = 7.0f/256.0f;
	tmp *= *pos;
	tmp_2 = (uint8_t) tmp;

	tmp = tmp_2 * 228.4f;
	tmp += 81.0f;
	*pos = (int16_t)tmp;
}

static void dmx_motor_control(void)
{
	motors[MOTOR_1].input_parameters.speed = 1.5f;
	motors[MOTOR_2].input_parameters.speed = 5.0f;
	motors[MOTOR_3].input_parameters.speed = 5.0f;
	motors[MOTOR_4].input_parameters.speed = 0.5f;

	motors[MOTOR_5].input_parameters.speed = 1.5f;
	motors[MOTOR_6].input_parameters.speed = 5.0f;
	motors[MOTOR_7].input_parameters.speed = 1.5f;
	motors[MOTOR_8].input_parameters.speed = 1.5f;

	for(int i = MOTOR_1; i < MOTOR_COUNT; i++)
	{
		motors[i].input_parameters.pos = dmx_array[i+1];
	}

	uint16_t tmp_pos = motors[MOTOR_6].input_parameters.pos;

	prism_control(&motors[MOTOR_4].input_parameters.pos);
	focus_control(&motors[MOTOR_5].input_parameters.pos);
	dimm_control(&motors[MOTOR_6].input_parameters.pos, &motors[MOTOR_2].input_parameters.pos);
	strobe_control(&motors[MOTOR_6].input_parameters.pos, &motors[MOTOR_2].input_parameters.pos, tmp_pos);
	prism_rotate_control(&motors[MOTOR_3].input_parameters.pos, &motors[MOTOR_3].input_parameters.speed);
	gobo_rotate_control(&motors[MOTOR_8].input_parameters.pos, &motors[MOTOR_8].input_parameters.speed);
	colorwheel_control(&motors[MOTOR_1].input_parameters.pos);
	gobowheel_control(&motors[MOTOR_7].input_parameters.pos);
}

static void hall_read(Hall_Sensor* hall)
{
	uint8_t tmp;

	tmp = HAL_GPIO_ReadPin(hall->GPIO_Port, hall->GPIO_pin);
	hall->value_container = ((hall->value_container<<1) | tmp);
}

static void colorwheel_reset(void)
{
	static Hall_Sensor hall_3 = {.GPIO_Port = HALL_3_GPIO_Port, .GPIO_pin = HALL_3_Pin, .value_container = 0xff};

	hall_read(&hall_3);

	if(FALLING_EDGE(hall_3.value_container) && !motors[MOTOR_1].input_parameters.motor_reset_f)
	{
		motor_set_0_pos(&motors[MOTOR_1].driver_parameters);
		motors[MOTOR_1].input_parameters.pos = 100;
		motors[MOTOR_1].input_parameters.motor_reset_f = 1;
	}
	else if((motors[MOTOR_1].input_parameters.motor_reset_f == 1) && (motors[MOTOR_1].driver_parameters.current_pos == 200))
	{
		motors[MOTOR_1].input_parameters.pos = -3000;
		motors[MOTOR_1].input_parameters.motor_reset_f = 2;
	}
	else if(FALLING_EDGE(hall_3.value_container) && (motors[MOTOR_1].input_parameters.motor_reset_f == 2))
	{
		motor_set_0_pos(&motors[MOTOR_1].driver_parameters);
		motors[MOTOR_1].input_parameters.pos = 0;
		motors[MOTOR_1].input_parameters.motor_reset_f = 3;
	}

}

static void prism_reset(void)
{

	static Hall_Sensor hall_2 = {.GPIO_Port = HALL_2_GPIO_Port, .GPIO_pin = HALL_2_Pin, .value_container = 0xff};

	hall_read(&hall_2);

	if(FALLING_EDGE(hall_2.value_container) && !motors[MOTOR_4].input_parameters.motor_reset_f)
	{
		motor_set_0_pos(&motors[MOTOR_4].driver_parameters);
		motors[MOTOR_4].input_parameters.pos = 100;
		motors[MOTOR_4].input_parameters.motor_reset_f = 1;
	}
	else if((motors[MOTOR_4].input_parameters.motor_reset_f == 1) && (motors[MOTOR_4].driver_parameters.current_pos == 200))
	{
		motors[MOTOR_4].input_parameters.pos = -3000;
		motors[MOTOR_4].input_parameters.motor_reset_f = 2;
	}
	else if(FALLING_EDGE(hall_2.value_container) && (motors[MOTOR_4].input_parameters.motor_reset_f == 2))
	{
		motor_set_0_pos(&motors[MOTOR_4].driver_parameters);
		motors[MOTOR_4].input_parameters.pos = 0;
		motors[MOTOR_4].input_parameters.motor_reset_f = 3;
	}
}

static void gobo_reset(void)
{
	static Hall_Sensor hall_1 = {.GPIO_Port = HALL_1_GPIO_Port, .GPIO_pin = HALL_1_Pin, .value_container = 0xff};

	enum timer_par_names {CURRENT_TIME, PREV_TIME, INTERVAL, PARAMETER_COUNT};
	static uint32_t timer_values[PARAMETER_COUNT] = { [INTERVAL] = 10000 };

	timer_values[CURRENT_TIME] = HAL_GetTick();
	hall_read(&hall_1);

	if(FALLING_EDGE(hall_1.value_container) && !motors[MOTOR_7].input_parameters.motor_reset_f)
	{
		motor_set_0_pos(&motors[MOTOR_7].driver_parameters);
		motors[MOTOR_7].input_parameters.pos = 0;
		motors[MOTOR_7].input_parameters.motor_reset_f = 1;

		timer_values[PREV_TIME] = timer_values[CURRENT_TIME];
	}
	else if(motors[MOTOR_7].input_parameters.motor_reset_f == 1)
	{
		if(hall_1.value_container == 0xff)
		{
			motors[MOTOR_7].input_parameters.pos = -3000;
			motors[MOTOR_7].input_parameters.motor_reset_f = 0;
		}
		else if ((uint32_t)(timer_values[CURRENT_TIME] - timer_values[PREV_TIME]) >= timer_values[INTERVAL])
		{
			motors[MOTOR_7].input_parameters.pos = 100;
			motors[MOTOR_7].input_parameters.motor_reset_f = 10;
		}
	}
	else if((motors[MOTOR_7].input_parameters.motor_reset_f == 10) && (motors[MOTOR_7].driver_parameters.current_pos == 200))
	{
		motors[MOTOR_7].input_parameters.pos = -3000;
		motors[MOTOR_7].input_parameters.motor_reset_f = 2;
	}
	else if(FALLING_EDGE(hall_1.value_container) && (motors[MOTOR_7].input_parameters.motor_reset_f == 2))
	{
		motor_set_0_pos(&motors[MOTOR_7].driver_parameters);
		motors[MOTOR_7].input_parameters.pos = 770;
		motors[MOTOR_7].input_parameters.motor_reset_f = 3;
	}
	else if((motors[MOTOR_7].input_parameters.motor_reset_f == 3) && (hall_1.value_container == 0xff))
	{
		motors[MOTOR_7].input_parameters.motor_reset_f = 4;
	}
	else if((motors[MOTOR_7].input_parameters.motor_reset_f== 4) && (hall_1.value_container == 0x00)&& !motors[MOTOR_8].input_parameters.motor_reset_f && (motors[MOTOR_7].driver_parameters.current_pos == 870))
	{
		motor_set_0_pos(&motors[MOTOR_8].driver_parameters);
		motors[MOTOR_8].input_parameters.pos = 0;
		motors[MOTOR_8].input_parameters.motor_reset_f = 1;
	}
}

static void shutter_corrigation(void)
{
	if((motors[MOTOR_2].driver_parameters.current_pos == 100) && !motors[MOTOR_2].input_parameters.motor_reset_f)
	{
		motors[MOTOR_2].input_parameters.pos = 112;
		motors[MOTOR_2].input_parameters.motor_reset_f = 1;
	}
}

static uint8_t reset_motors(void)
{
	static uint8_t reset = 1;

	colorwheel_reset();
	prism_reset();
	gobo_reset();
	shutter_corrigation();

	if(motors[MOTOR_2].input_parameters.motor_reset_f && motors[MOTOR_8].input_parameters.motor_reset_f)
	{
		if((motors[MOTOR_1].input_parameters.motor_reset_f == 3) && (motors[MOTOR_4].input_parameters.motor_reset_f == 3))
		{
			reset = 0;
		}
	}

	return reset;
}

void my_main_init(void)
{
	for(int i = MOTOR_1; i<MOTOR_COUNT; i++)
	{
		// common
		motors[i].input_parameters.speed = 		0.5f;
		motors[i].driver_parameters.interval = 	10;

		// different
		motors[i].driver_parameters.pin_1 = motor_config[i].pin1;
		motors[i].driver_parameters.pin_2 = motor_config[i].pin2;
		motors[i].driver_parameters.n_max = motor_config[i].max_speed;

		if(motor_config[i].limit_type == HALL_SENSOR)
		{
			motors[i].driver_parameters.current_pos = 100;
			motors[i].input_parameters.pos = -3000;
		}
		else
		{
			motors[i].driver_parameters.current_pos = 1500;
			motors[i].input_parameters.pos = 0;
		}
	}
}

void my_main_loop(void)
{
	static uint8_t reset = 1;

	if(reset)
	{
		reset = reset_motors();
	}
	else
	{
		dmx_motor_control();
	}

	for (int i = MOTOR_1; i < MOTOR_COUNT; i++)
	{
		motor_main(&motors[i].driver_parameters, motors[i].input_parameters.pos, motors[i].input_parameters.speed);
	}
}

void motor_refresh_IT(void)
{
	static uint32_t Motor_Tick = 0;
	Motor_Tick++;

	for (int i = MOTOR_1; i < MOTOR_COUNT; i++)
	{
		motors[i].driver_parameters.current_time = Motor_Tick;
		motor_update_timer(&motors[i].driver_parameters);
	}
}

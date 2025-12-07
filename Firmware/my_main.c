/*
 * my_main.c
 *
 *  Created on: Oct 18, 2025
 *      Author: bagob
 */

#include <my_main.h>

#include "main.h"
#include <dmx_usart_s.h>
#include <motor_drive_h_s.h>

#define MAX_POS 100000UL

MOTOR_TypeDef Motor_1 = {0};
MOTOR_TypeDef Motor_2 = {0};
MOTOR_TypeDef Motor_3 = {0};
MOTOR_TypeDef Motor_4 = {0};
MOTOR_TypeDef Motor_5 = {0};
MOTOR_TypeDef Motor_6 = {0};
MOTOR_TypeDef Motor_7 = {0};
MOTOR_TypeDef Motor_8 = {0};

static int16_t pos_1 = 0;
static int16_t pos_2 = 0;
static int16_t pos_3 = 0;
static int16_t pos_4 = 0;
static int16_t pos_5 = 0;
static int16_t pos_6 = 0;
static int16_t pos_7 = 0;
static int16_t pos_8 = 0;

static float speed_1 = 0;
static float speed_2 = 0;
static float speed_3 = 0;
static float speed_4 = 0;
static float speed_5 = 0;
static float speed_6 = 0;
static float speed_7 = 0;
static float speed_8 = 0;

static uint8_t motor_1_reset_f = 0;
static uint8_t motor_2_reset_f = 0;
static uint8_t motor_4_reset_f = 0;
static uint8_t motor_7_reset_f = 0;
static uint8_t motor_8_reset_f = 0;

static uint8_t reset = 1;

/*	motor mapping
 * 	motor_1		color_w		pin  0 1
 * 	motor_2		shutter_1	pin  2 3
 * 	motor_3		pris_rot	pin  4 5
 * 	motor_4		pris 		pin  6 7
 * 	motor_5		focus 		pin  8 9
 * 	motor_6		shutter_2	pin  10 11
 * 	motor_7		gobo		pin  12 13
 * 	motor_8		gobo_rot	pin  14 15
 */

static void dmx_channel_map(void)
{
	static uint16_t colorwheel_array[12] = {128, 261, 394, 527, 660, 793, 926, 1059,1192, 1325, 1458, 1591};
	float tmp;

	speed_1 = 1.5f;
	speed_2 = 5.0f;
	speed_3 = 5.0f;
	speed_4 = 0.5f;
	speed_5 = 1.5f;
	speed_6 = 5.0f;
	speed_7 = 1.5f;
	speed_8 = 1.5f;

	pos_1 = (int16_t)dmx_array[1];
	pos_2 = (int16_t)dmx_array[2];
	pos_3 = (int16_t)dmx_array[3];
	pos_4 = (int16_t)dmx_array[4];
	pos_5 = (int16_t)dmx_array[5];
	pos_6 = (int16_t)dmx_array[6];
	pos_7 = (int16_t)dmx_array[7];
	pos_8 = (int16_t)dmx_array[8];

	// prism on off
	if(pos_4 > 125) pos_4 = 257;
	else 			pos_4 = 0;

	// focus
	tmp = 335.0f/255.0f;
	tmp *= pos_5;
	pos_5 = (int16_t)tmp;

	// strobe 6 pos alapján --> pos 2
	uint8_t tmp_pos;
	uint8_t tmp_pos_6;
	static uint32_t strobe_interval;
	static uint32_t current_time = 0;
	static uint32_t prev_time = 0;
	static uint8_t strobe_f = 0;

	// dimm pos_2 alapján
	tmp_pos_6 = pos_6;
	tmp_pos = pos_2;
	tmp = 100.0f/255.0f;
	tmp *= pos_2;
	pos_2 = (int16_t)tmp;

	tmp = 75.0f/255.0f;
	tmp *= tmp_pos;
	pos_6 = (int16_t)tmp;

	if(tmp_pos_6 >55)
	{
		tmp = 9.0f/201.0f;
		tmp *= ((float)tmp_pos_6-55.0f);
		tmp += 1.0f;

		current_time = HAL_GetTick();
		strobe_interval = (uint32_t)(100.0f * (11.0f - tmp));

		if ((uint32_t)(current_time - prev_time)>= strobe_interval)
		{
			prev_time = current_time;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			strobe_f = 1;
		}

		if(strobe_f && (Motor_2.current_pos == (pos_2 + 100)))
		{
			strobe_f = 0;
		}

		if(!strobe_f)
		{
			pos_2 = 0;
			pos_6 = 0;
		}
	}
	else
	{

	}

	// prism rotate
	tmp_pos = pos_3;
	if(pos_3 > 55)
	{
		if(pos_3>155)
		{
			pos_3 = 3000;
			// speed szamolas
			tmp = 4.5f/100.0f;
			tmp *= ((float)tmp_pos - 155.0f);
			tmp += 0.5f;
			speed_3 = tmp;
		}
		else
		{
			pos_3 = -3000;
			// speed szamolas
			tmp = 4.5f/100.0f;
			tmp *=  ((float)tmp_pos - 55.0f);
			tmp += 0.5f;
			speed_3 = tmp;
		}
	}
	else
	{
		pos_3 = 0;
		speed_3 = 5.0f;
	}

	// prism on off
	tmp_pos = pos_8;
	if(pos_8 > 55)
	{
		if(pos_8>155)
		{
			pos_8 = 3000;
			// speed szamolas
			tmp = 1.0f/100.0f;
			tmp *= ((float)tmp_pos - 155.0f);
			tmp += 0.5f;
			speed_8 = tmp;
		}
		else
		{
			pos_8 = -3000;
			// speed szamolas
			tmp = 1.0f/100.0f;
			tmp *=  ((float)tmp_pos - 55.0f);
			tmp += 0.5f;
			speed_8 = tmp;
		}
	}
	else
	{
		tmp = 800.0f/55.0f;
		tmp *= pos_8;
		pos_8 = (int16_t)tmp;
		speed_8 = 1.5f;
	}

	// color wheel
	if(pos_1 <= 200)
	{
		tmp = 11.0f/201.0f;
		tmp *= pos_1;
		pos_1 = colorwheel_array[(int8_t)tmp];
	}
	else if(pos_1 < 225)
	{
		pos_1 = -3000;
	}
	else
	{
		pos_1 = 3000;
	}

	uint8_t tmp_2 = 0;
	tmp = 7.0f/256.0f;
	tmp *= pos_7;
	tmp_2 = (uint8_t) tmp;

	tmp = tmp_2 * 228.4f;
	tmp += 81.0f;
	pos_7 = (int16_t)tmp;
}

static void reset_fgv(void)
{
	static uint8_t hall_1_edge = 0xff;
	static uint8_t hall_2_edge = 0xff;
	static uint8_t hall_3_edge = 0xff;

	uint8_t hall;

	hall = HAL_GPIO_ReadPin(HALL_1_GPIO_Port, HALL_1_Pin); // gobo
	hall_1_edge = ((hall_1_edge<<1) | hall);

	hall = HAL_GPIO_ReadPin(HALL_2_GPIO_Port, HALL_2_Pin); // prizma
	hall_2_edge = ((hall_2_edge<<1) | hall);

	hall = HAL_GPIO_ReadPin(HALL_3_GPIO_Port, HALL_3_Pin); // color
	hall_3_edge = ((hall_3_edge<<1) | hall);

	static uint32_t current_time = 0;
	static uint32_t prev_time = 0;
	static uint16_t interval = 10000;

	current_time = HAL_GetTick();

	if(((hall_3_edge & 0x03 ) == 0x02) && !motor_1_reset_f)
	{
		motor_1_set_0_pos(&Motor_1);
		pos_1 = 100;
		motor_1_reset_f = 1;
	}
	else if((motor_1_reset_f == 1) && (Motor_1.current_pos == 200))
	{
		pos_1 = -3000;
		motor_1_reset_f = 2;
	}
	else if(((hall_3_edge & 0x03 ) == 0x02) && (motor_1_reset_f == 2))
	{
		motor_1_set_0_pos(&Motor_1);
		pos_1 = 0;
		motor_1_reset_f = 3;
	}

	if(((hall_2_edge & 0x03 ) == 0x02) && !motor_4_reset_f)
	{
		motor_1_set_0_pos(&Motor_4);
		pos_4 = 100;
		motor_4_reset_f = 1;
	}
	else if((motor_4_reset_f == 1) && (Motor_4.current_pos == 200))
	{
		pos_4 = -3000;
		motor_4_reset_f = 2;
	}
	else if(((hall_2_edge & 0x03 ) == 0x02) && (motor_4_reset_f == 2))
	{
		motor_1_set_0_pos(&Motor_4);
		pos_4 = 0;
		motor_4_reset_f = 3;
	}


	if(((hall_1_edge & 0x03 ) == 0x02) && !motor_7_reset_f)
	{
		motor_1_set_0_pos(&Motor_7);
		pos_7 = 0;
		motor_7_reset_f = 1;

		prev_time = current_time;

	}
	else if(motor_7_reset_f == 1)
	{
		if(hall_1_edge == 0xff)
		{
			motor_7_reset_f = 0;
			pos_7 = -3000;
		}
		else if ((uint32_t)(current_time - prev_time)>= interval)
		{
			pos_7 = 100;
			motor_7_reset_f = 10;
		}
	}
	else if((motor_7_reset_f == 10) && (Motor_7.current_pos == 200))
	{
		pos_7 = -3000;
		motor_7_reset_f = 2;
	}
	else if(((hall_1_edge & 0x03 ) == 0x02)&&(motor_7_reset_f == 2))
	{
		motor_1_set_0_pos(&Motor_7);
		pos_7 = 770;
		motor_7_reset_f = 3;
	}
	else if((motor_7_reset_f == 3)&&(hall_1_edge == 0xff))
	{
		motor_7_reset_f = 4;
	}
	else if((motor_7_reset_f == 4)&&(hall_1_edge == 0x00)&& !motor_8_reset_f && (Motor_7.current_pos == 870))
	{
		motor_1_set_0_pos(&Motor_8);
		pos_8 = 0;
		motor_8_reset_f = 1;
	}

	if((Motor_2.current_pos == 100) && !motor_2_reset_f)
	{
		Motor_2.current_pos = 112;
		motor_2_reset_f = 1;
	}
	if(motor_2_reset_f && motor_8_reset_f && (motor_1_reset_f == 3) &&  (motor_4_reset_f == 3))
	{
		reset = 0;
	}
}

void my_main_init(void)
{
	Motor_1.current_pos = 100;
	Motor_2.current_pos = 1500;
	Motor_3.current_pos = 100;
	Motor_4.current_pos = 1500;
	Motor_5.current_pos = 1500;
	Motor_6.current_pos = 1500;
	Motor_7.current_pos = 100;
	Motor_8.current_pos = 100;

	Motor_1.interval = 10;
	Motor_2.interval = 10;
	Motor_3.interval = 10;
	Motor_4.interval = 10;
	Motor_5.interval = 10;
	Motor_6.interval = 10;
	Motor_7.interval = 10;
	Motor_8.interval = 10;


	Motor_1.n_max = 200;
	Motor_1.pin_1 = 0;
	Motor_1.pin_2 = 1;

	Motor_2.n_max = 25;
	Motor_2.pin_1 = 2;
	Motor_2.pin_2 = 3;

	Motor_3.n_max = 500;
	Motor_3.pin_1 = 4;
	Motor_3.pin_2 = 5;

	Motor_4.n_max = 50;
	Motor_4.pin_1 = 6;
	Motor_4.pin_2 = 7;

	Motor_5.n_max = 100;
	Motor_5.pin_1 = 8;
	Motor_5.pin_2 = 9;

	Motor_6.n_max = 25;
	Motor_6.pin_1 = 11;
	Motor_6.pin_2 = 10;

	Motor_7.n_max = 200;
	Motor_7.pin_1 = 12;
	Motor_7.pin_2 = 13;

	Motor_8.n_max = 200;
	Motor_8.pin_1 = 14;
	Motor_8.pin_2 = 15;

	pos_1 = -3000;
	pos_2 = 0;
	pos_3 = 0;
	pos_4 = 0;
	pos_5 = 0;
	pos_6 = 0;
	pos_7 = -3000;
	pos_8 = -3000;

	speed_1 = 0.5f;
	speed_2 = 0.5f;
	speed_3 = 0.5f;
	speed_4 = 0.5f;
	speed_5 = 0.5f;
	speed_6 = 0.5f;
	speed_7 = 0.5f;
	speed_8 = 0.5f;
}

void my_main_loop(void)
{
	if(reset)
	{
		reset_fgv();
	}
	else
	{
		dmx_channel_map();
	}

	motor_1_main(&Motor_1 , pos_1, speed_1);	// colorw
	motor_1_main(&Motor_2 , pos_2, speed_2);	// shutter_1
	motor_1_main(&Motor_3 , pos_3, speed_3);	// prizrot
	motor_1_main(&Motor_4 , pos_4, speed_4);	// priz
	motor_1_main(&Motor_5 , pos_5, speed_5);	// focus
	motor_1_main(&Motor_6 , pos_6, speed_6);	// shutter2
	motor_1_main(&Motor_7 , pos_7, speed_7);	// gobo
	motor_1_main(&Motor_8 , pos_8, speed_8);	// goborot
}

void motor_refresh_IT(void)
{
	static uint32_t Motor_Tick = 0;
	Motor_Tick++;

	Motor_1.current_time = Motor_Tick;
	Motor_2.current_time = Motor_Tick;
	Motor_3.current_time = Motor_Tick;
	Motor_4.current_time = Motor_Tick;
	Motor_5.current_time = Motor_Tick;
	Motor_6.current_time = Motor_Tick;
	Motor_7.current_time = Motor_Tick;
	Motor_8.current_time = Motor_Tick;

	motor_1_update_timer(&Motor_1);
	motor_1_update_timer(&Motor_2);
	motor_1_update_timer(&Motor_3);
	motor_1_update_timer(&Motor_4);
	motor_1_update_timer(&Motor_5);
	motor_1_update_timer(&Motor_6);
	motor_1_update_timer(&Motor_7);
	motor_1_update_timer(&Motor_8);
}

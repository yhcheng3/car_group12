#ifndef __CONTROL_H
#define __CONTROL_H

#include "include.h"

#define PERIOD 50

typedef struct
{
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t d;
}car_sensor_t;

typedef struct
{
	int16_t B;
	int16_t L;
	int16_t R;
	
	int reverse_cnt;
	int anti_reverse_cnt;
	
	int rotate_cnt;
	int rotate_dir;
}controller_t;

void init_ctrl(controller_t *ctrl);

uint8_t invert(uint8_t val);

void read_sensor(car_sensor_t *carSensor);

void set_PWM(controller_t  *ctrl, int car_V, int E_V, int motor_flag);

void circle_PWM(controller_t *ctrl, int threshold);

void set_control(controller_t *ctrl, car_sensor_t *carSensor, int motor_flag);

#endif

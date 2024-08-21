#ifndef __CONTROL_H
#define __CONTROL_H

#include "include.h"

#define PERIOD 50

//----------set_PWM()--------
#define E_FACTOR 150         
#define E_B_FACTOR 300
#define R_FACTOR
#define MIN_R 800

//----------circle_PWM()--------
#define CRCL_FACTOR 800
#define CRCL_TWIST_FACTOR 3600
#define CRCL_THRESH 1600
#define ROT_THRESH 2 //�����ת�������������
#define TWIST_DELAY 70

//----------set_PWM()--------
//#define REV_CNT_THRESH_C 20
//#define REV_CNT_THRESH 150
//#define REV_CNT_ADD 2
//#define ANTI_REV_CNT_THRESH 0
#define CAR_REV 1200
#define CAR_TURN 800
#define CAR_FWD 1500

#define MAX_VAL 6000

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
	
	int rotate_cnt;
	int rotate_dir; // 0�� 1��
	int rotate_times;

	int work_state;
}controller_t;


typedef struct 
{
	int B;
	int L;
	int R;

}MotoPWM_t;


typedef struct
{
	int B;
	int L;
	int R;

}ENC_t;

typedef enum {
    forward,stop,turn_right,turn_left
} MoveDir;

typedef enum {
    idle , run , barrier , find_R , find_L , delay_R, delay_L , compare_RL
} CarState;


void init_ctrl(controller_t *ctrl);

uint8_t invert(uint8_t val);

void read_sensor(car_sensor_t *carSensor);

void set_PWM(controller_t  *ctrl, int car_V, int E_V);

void circle_PWM(controller_t *ctrl);

void set_control(controller_t *ctrl, car_sensor_t *carSensor, int motor_flag);

void read_ENC_V(ENC_t *enc_v);

void car_move(controller_t *ctrl, MotoPWM_t *motopwm, MoveDir move);

void ultrasonic_avoid(controller_t *ctrl, CarState car_state, MotoPWM_t *motopwm, int dis, ENC_t *enc_v);

#endif

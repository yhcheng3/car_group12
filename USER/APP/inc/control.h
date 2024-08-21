#ifndef __CONTROL_H
#define __CONTROL_H

#include "include.h"

// TODO: period now defined in stm32f1xx_it.c
#define PERIOD 50 

//----------set_PWM()--------
#define E_FACTOR 150         
#define E_B_FACTOR 300
#define R_FACTOR
#define MIN_R 800

//----------circle_PWM()--------
#define CRCL_FACTOR 800
#define CRCL_TWIST_FACTOR 3000
#define CRCL_THRESH 1200
#define ROT_THRESH 1 //�����ת�������������
#define TWIST_DELAY 70

//----------set_PWM()--------
#define CAR_REV 1000
#define CAR_TURN 800
#define CAR_FWD 1500

#define MAX_VAL 6000

//----------rotate()
#define FWD_FACTOR 800
#define ROT_FACTOR 2000
typedef struct
{
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t d;
}photoele_t;

typedef struct
{
	int B;
	int L;
	int R;

}encoder_t;

typedef enum {
    forward, stop, turn_right, turn_big_right, turn_big_left
} MoveDir;

typedef enum {
    idle, run, find_R, find_L, delay_R, delay_L, compare_RL
} CarState;

typedef struct
{
	int16_t B;
	int16_t L;
	int16_t R;
	
	int rotate_cnt; // Cumulative rotation iterations
	int rotate_dir; // Direction: 0 left, 1 right
	int rotate_times; // No. of full rotation cycles

	int work_state; // State: 0: tracking, 1: obstacle
	CarState car_state;
	
	int on_path; // 0: Off path; 1: On path
	
	int mode; // 模式 0: 循迹会后退  1: 循迹会跳进避障状态
}controller_t;

// --------Initialise------
void init_ctrl(controller_t *ctrl);

// --------Tracking--------
uint8_t invert(uint8_t val);

void read_sensor(photoele_t *photoele);

void set_PWM(controller_t  *ctrl, int car_V, int E_V);

void circle_PWM(controller_t *ctrl);

void set_control(controller_t *ctrl, photoele_t *photoele);

// --------Obstacle--------
void read_enc(void);

void car_move(controller_t *ctrl, MoveDir move);

void ultrasonic_avoid(controller_t *ctrl, photoele_t *photoele);

#endif

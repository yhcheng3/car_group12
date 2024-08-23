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
#define CRCL_TWIST_FACTOR 2800
#define CRCL_THRESH 1000
#define ROT_THRESH 1 //�����ת�������������
#define TWIST_DELAY 70

//----------set_PWM()--------
#define CAR_REV 1000
#define CAR_TURN 800
#define CAR_FWD 1500

#define MAX_VAL 6000

//----------rotate()
#define FWD_FACTOR 1000
#define ROT_FACTOR 2500

//----------ultrasonic_avoid()
#define SWITCH_THRESH 20
#define AVOID_THRESH 30

//----------Multiply R motor by:-------
#define RIGHT_FACTOR 1.2

//----------get_distance_filtered()
#define DIS_SAMPLES 5
#define DIS_MAX_ERR 5

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
    forward, turn_right, turn_big_right, turn_big_left, stop
} MoveDir;

typedef enum {
    run, find_R, delay_R, find_L, delay_L, compare_RL
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

void init_photoele(photoele_t *photoele);
	
void init_enc(encoder_t *enc);

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

int get_distance_filtered(void);

void recv_torque(controller_t *ctrl);

void recv_img(controller_t *ctrl);

#endif

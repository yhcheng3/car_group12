/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ���ƽ��Ħ�г�
����    д��chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 
�������¡�2020��4��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Keil5.2�����ϰ汾
��Target �� STM32F103
��Crystal�� 8.000Mhz
��SYS PLL�� 72MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __CONTROL_H
#define __CONTROL_H
#include "include.h"

#define PI 3.14159265

#define DIFFERENCE 160
#define LQSKH   1       //1 ���ģ 

#ifdef LQSKH            //���ģ  		
			#define SERVO_MID 754  //������м�λ�ã���ת930--780--630�ұ�
#else			
			#define SERVO_MID 750
#endif

//int EXTI9_5_IRQHandler(void);
int Balance(float Angle,float Gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
int turn(int encoder_left,int encoder_right,float gyro);
void Key(void);
void Xianfu_Pwm(void);
int myabs(int a);
void OLED_show(void);
extern short Encoder_A,Encoder_B,Encoder_C,Encoder_D;   //���������������
extern int Left ,Right; 
void balance_car(void);
void Infra_Init(void);
void Infra_car(void);
#endif

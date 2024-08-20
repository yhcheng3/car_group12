/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�F3277���İ��ĸ��
����    д������Ƽ�
��E-mail  ��chiusir@163.com
������汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��12��24�գ��������£����ע���°棡
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��IAR7.8 KEIL5.24�����ϰ汾
��Target �� MM32F3277
��SYS PLL�� 120MHz Ƶ��̫�߿����޷�����system_mm32f327x.c
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_MOTOR_H
#define __LQ_MOTOR_H

#include "include.h"

//#define Servo_Center_Mid 730        //���ֱ����ֵ��
//#define MtTargetDuty  3300          //���ռ�ձ�ֵ�����ֵ��ʵ����Ҫ��ȥ��ص�ѹ�����������������ʣ��ﲻ����ֵ

#define	PWM_DUTY_MAX 7200 

#define MTGPIO1p      GPIOB,GPIO_PIN_12        //����������ܽ�
#define MTGPIO2p      GPIOB,GPIO_PIN_13  
#define MTGPIO3p      GPIOB,GPIO_PIN_14 

//extern int16_t ServoDuty;
//extern int16_t MotorDuty1;          // �������ռ�ձ���ֵ
//extern int16_t MotorDuty2;          // �������ռ�ձ���ֵ

//extern uint16_t BatVolt;            // ��ص�ѹ�ɼ�


void MotorInit(void);
void TIM1_PWM_Init(u16 arr,u16 psc);
void MotorCtrl(int16_t motor1, int16_t motor2);
void MotorCtrl3w(int16_t motor1, int16_t motor2, int16_t motor3);
void Brushless_Motor_Ctrl(int16_t motor1, int16_t motor2, int16_t motor3);
void Brushless_Motor_Init(u16 freq);

void ServoInit(u16 duty, u16 psc);
void ServoCtrl(int16_t motor1);
void TIM4_PWM_Init(u16 arr,u16 psc, u16 duty);

void TestMotor(void);
void TestServo(void);
void Test_Brushless_Motor(void);


#endif /* SRC_APPSW_TRICORE_USER_LQ_MOTORCONTROL_H_ */

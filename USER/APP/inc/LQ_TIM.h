#ifndef __TIM_H
#define __TIM_H
#include "include.h"


extern TIM_HandleTypeDef TIM1_Handler;      //��ʱ����� 
extern TIM_HandleTypeDef TIM2_Handler;      //��ʱ����� 
extern TIM_HandleTypeDef TIM3_Handler;      //��ʱ����� 
extern TIM_HandleTypeDef TIM4_Handler;      //��ʱ����� 

void LQ_TIM3_Init(u16 arr,u16 psc);
 #endif

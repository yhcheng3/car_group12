
#ifndef __LQ_ADC_H
#define __LQ_ADC_H

#include "include.h"


void ADC_Init(void); 				//ADCͨ����ʼ��
u16  Get_Adc(u32 ch); 		        //���ĳ��ͨ��ֵ 
u16 Get_Adc_Average(u32 ch,u8 times);//�õ�ĳ��ͨ����������������ƽ��ֵ

void Test_ADC(void);
int Get_Temp(uint32_t val);

#endif 








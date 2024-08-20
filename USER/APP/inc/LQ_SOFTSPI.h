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
#ifndef __LQ_Soft_SPI_H
#define __LQ_Soft_SPI_H
#include "include.h"


//IO����                         
#define Soft_SPI_SCK_PIN      PAout(5)     //SCK
#define Soft_SPI_MISO_PIN     PAout(6)     //��Ƭ��MISO��ģ���--SDO
#define Soft_SPI_MOSI_PIN     PAout(7)     //��Ƭ��MOSI��ģ���--SDI
#define Soft_SPI_CS_PIN       PAout(4)     //CS Ĭ����PE11,���ʹ�ñ�������������л���PE10

#define Soft_SPI_MISO_IN      PAin(6)				//��ȡ�����ϵ�����״̬

#define Soft_SPI_SCK_OUT_H    Soft_SPI_SCK_PIN=1          //��������ߵ�ƽ
#define Soft_SPI_SCK_OUT_L    Soft_SPI_SCK_PIN=0        	//��������͵�ƽ

#define Soft_SPI_MOSI_OUT_H   Soft_SPI_MOSI_PIN=1        	//��������ߵ�ƽ
#define Soft_SPI_MOSI_OUT_L   Soft_SPI_MOSI_PIN=0      		//��������͵�ƽ

#define Soft_SPI_CS_OUT_H     Soft_SPI_CS_PIN=1          	//��������ߵ�ƽ
#define Soft_SPI_CS_OUT_L     Soft_SPI_CS_PIN=0        		//��������͵�ƽ

void Soft_SPI_Init(void);
void Soft_SPI_ReadWriteNbyte(unsigned char *lqbuff, unsigned char len);
void LQ_SPI_Read(unsigned char reg, unsigned short len, unsigned char* buf);
void LQ_SPI_Write(unsigned char reg, unsigned short len, unsigned char* buf);
#endif


















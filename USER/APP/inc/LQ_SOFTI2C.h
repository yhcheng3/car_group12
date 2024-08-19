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
#ifndef __LQ_SOFTIIC_H
#define __LQ_SOFTIIC_H

#include "stdint.h"
#include "include.h"

//IO��������
#define SDA_IN  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}		//PB9����ģʽ
#define SDA_OUT {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;} 	//PB9���ģʽ

//IO����
#define IIC_SCL   PBout(8) //SCL
#define IIC_SDA   PBout(9) //SDA
#define READ_SDA  PBin(9)  //����SDA


#define SOFT_I2C_SCL_PIN   GPIOB,GPIO_PIN_8   /*!< SCCB SCL �ܽ� */
#define SOFT_I2C_SDA_PIN   GPIOB,GPIO_PIN_9   /*!< SCCB SDA �ܽ� */


#define IIC_SCL_H  IIC_SCL=1        //��������ߵ�ƽ
#define IIC_SCL_L  IIC_SCL=0      //��������͵�ƽ

#define IIC_SDA_H  IIC_SDA=1        //��������ߵ�ƽ
#define IIC_SDA_L  IIC_SDA=0      //��������͵�ƽ

#define IIC_SDA_READ  PBin(9)//��ȡ�����ϵ�����״̬

void SOFT_IIC_Start(void);	 		        //����IIC��ʼ�ź�
void SOFT_IIC_Stop(void);	  	                //����IICֹͣ�ź�
void SOFT_IIC_Ack(void);			        //IIC����ACK�ź�
void SOFT_IIC_NAck(void);			        //IIC������ACK�ź�
void SOFT_IIC_Send_Byte(unsigned char data_t);       //IIC����һ���ֽ�
unsigned char SOFT_IIC_Wait_Ack(void); 		//IIC�ȴ�ACK�ź�
unsigned char SOFT_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
int LQ_I2C_Write(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *dat);
int LQ_I2C_Read(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf);


void SOFT_IIC_Init(void);
unsigned char IIC_ReadByteFromSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf);
unsigned char IIC_ReadMultByteFromSlave(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *data_t);
unsigned char IIC_WriteByteToSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char buf);
unsigned char IIC_WriteMultByteToSlave(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* data_t);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char dat);
unsigned char IICwriteBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char* dat);
unsigned char IICwriteBits(unsigned char dev,unsigned char reg,unsigned char bitStart,unsigned char length,unsigned char dat);
unsigned char IICwriteBit(unsigned char dev,unsigned char reg,unsigned char bitNum,unsigned char dat);
unsigned char IICreadBytes(unsigned char dev, unsigned char reg, unsigned char length, unsigned char *dat);

#endif

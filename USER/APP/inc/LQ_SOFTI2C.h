/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技F3277核心板和母板
【编    写】龙邱科技
【E-mail  】chiusir@163.com
【软件版本】V1.0 版权所有，单位使用请先联系授权
【最后更新】2020年12月24日，持续更新，请关注最新版！
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【IDE】IAR7.8 KEIL5.24及以上版本
【Target 】 MM32F3277
【SYS PLL】 120MHz 频率太高可能无法启动system_mm32f327x.c
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_SOFTIIC_H
#define __LQ_SOFTIIC_H

#include "stdint.h"
#include "include.h"

//IO方向设置
#define SDA_IN  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}		//PB9输入模式
#define SDA_OUT {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;} 	//PB9输出模式

//IO操作
#define IIC_SCL   PBout(8) //SCL
#define IIC_SDA   PBout(9) //SDA
#define READ_SDA  PBin(9)  //输入SDA


#define SOFT_I2C_SCL_PIN   GPIOB,GPIO_PIN_8   /*!< SCCB SCL 管脚 */
#define SOFT_I2C_SDA_PIN   GPIOB,GPIO_PIN_9   /*!< SCCB SDA 管脚 */


#define IIC_SCL_H  IIC_SCL=1        //配置输出高电平
#define IIC_SCL_L  IIC_SCL=0      //配置输出低电平

#define IIC_SDA_H  IIC_SDA=1        //配置输出高电平
#define IIC_SDA_L  IIC_SDA=0      //配置输出低电平

#define IIC_SDA_READ  PBin(9)//读取引脚上的引脚状态

void SOFT_IIC_Start(void);	 		        //发送IIC开始信号
void SOFT_IIC_Stop(void);	  	                //发送IIC停止信号
void SOFT_IIC_Ack(void);			        //IIC发送ACK信号
void SOFT_IIC_NAck(void);			        //IIC不发送ACK信号
void SOFT_IIC_Send_Byte(unsigned char data_t);       //IIC发送一个字节
unsigned char SOFT_IIC_Wait_Ack(void); 		//IIC等待ACK信号
unsigned char SOFT_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
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

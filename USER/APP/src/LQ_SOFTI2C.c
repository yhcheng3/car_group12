/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <stdint.h>
#include "LQ_SOFTI2C.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void SOFT_IIC_delay(u16 us)
@����˵����IIC��ʱ
@����˵����us����ʱʱ��
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����SOFT_IIC_delay();  //�򵥵���ʱ
@��    ע���޸�������Ե���IIC����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_delay(u16 us)
{
  delay_us(us);  
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void IIC_Init(void)
@����˵����ģ��IIC��ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_Init();  //ģ��IIC��ʼ��
@��    ע��IIC�ܽ���LQ_SOFTI2C.h�ж���
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOB_CLK_ENABLE();           	//����GPIOBʱ��   
	//PB8/9��ʼ������
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    IIC_SCL_H;
    IIC_SDA_H;
    SOFT_IIC_Stop();
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void SOFT_IIC_Start(void)
@����˵����ģ��IIC��ʼ�ź�
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_Start();
@��    ע���ڲ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Start(void)
{
  IIC_SCL_L;
  SDA_OUT;   //sda����� 
  IIC_SDA_H;	
  IIC_SCL_H;
  SOFT_IIC_delay(1);
  IIC_SDA_L; //START:when CLK is high,DATA change form high to low 
  SOFT_IIC_delay(1);
  IIC_SCL_L; //ǯסI2C���ߣ�׼�����ͻ�������� 
  SOFT_IIC_delay(1);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void SOFT_IIC_Stop(void)
@����˵����ģ��IICֹͣ�ź�
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_Stop();
@��    ע���ڲ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Stop(void)
{  
  IIC_SCL_L;
  SDA_OUT; //sda�����
  IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
  SOFT_IIC_delay(1);
  IIC_SCL_H; 
  SOFT_IIC_delay(1);
  IIC_SDA_H; //����I2C���߽����ź�
  SOFT_IIC_delay(1);							   	
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void IIC_NAck(void)
@����˵����ģ��IIC������ACKӦ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_NAck();
@��    ע���ڲ����� �������������һ���ֽ����ݺ�����������NACK֪ͨ�ӻ����ͽ�����
@           �ͷ�SDA,�Ա���������ֹͣ�ź�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Ack(void)
{
  IIC_SCL_L;
  SDA_OUT;
  IIC_SDA_L;
  SOFT_IIC_delay(1);
  IIC_SCL_H;
  SOFT_IIC_delay(2);
  IIC_SCL_L;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void SOFT_IIC_NAck(void)
@����˵����ģ��IIC����ACKӦ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����SOFT_IIC_NAck();
@��    ע���ڲ����� ����������һ���ֽ����ݺ�����������ACK֪ͨ�ӻ�һ���ֽ���������ȷ����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_NAck(void)
{
  IIC_SCL_L;
  SDA_OUT;
  SOFT_IIC_delay(1);
  IIC_SDA_H;
  SOFT_IIC_delay(1);
  IIC_SCL_H;
  SOFT_IIC_delay(2);
  IIC_SCL_L;
}						     

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char SOFT_IIC_Wait_Ack(void)
@����˵����ģ��IIC�ȴ�Ӧ���ź�
@����˵������
@�������أ�1������Ӧ��ʧ��    0������Ӧ��ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����SOFT_IIC_Wait_Ack();
@��    ע���ڲ����� ��ЧӦ�𣺴ӻ���9�� SCL=0 ʱ SDA ���ӻ�����,���� SCL = 1ʱ SDA��ȻΪ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char SOFT_IIC_Wait_Ack(void)
{
  unsigned char  ucErrTime=0;
  SDA_IN; //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ�� 
  IIC_SCL_L;
  IIC_SDA_H;
  SOFT_IIC_delay(1);	   
  IIC_SCL_H;
  SOFT_IIC_delay(1);	 
  while(IIC_SDA_READ&0x1)
  {
    ucErrTime++;
    if(ucErrTime>200)
    {
      //SOFT_IIC_Stop();
      return 1;
    }
    SOFT_IIC_delay(1);	 
  }
  SOFT_IIC_delay(1);	 
  IIC_SCL_L; //ʱ�����0 	   
  return 0;  
} 

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void IIC_Send_Byte(unsigned char data_t)
@����˵����ģ��IIC����һ���ֽ�
@����˵����data: ���͵��ֽ�
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_SendByte(0x12);
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Send_Byte(unsigned char data_t)
{                        
  unsigned char  t;
      
  IIC_SCL_L; //����ʱ�ӿ�ʼ���ݴ���
  SDA_OUT; 	
  for(t=0;t<8;t++)
  {   
    SOFT_IIC_delay(1);
    if(data_t&0x80)
    {
      IIC_SDA_H;
    }
    else 
    {
      IIC_SDA_L;
    }
    SOFT_IIC_delay(1);
    IIC_SCL_H;   
    data_t<<=1;
    SOFT_IIC_delay(2);   
    IIC_SCL_L;	     	
  }
  IIC_SDA_H;
  SOFT_IIC_delay(1);
} 	 

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char  SOFT_IIC_Read_Byte(unsigned char ack)
@����˵����ģ��IIC��ȡһ���ֽ�
@����˵����ack=1 ʱ���������ݻ�û������ ack=0 ʱ����������ȫ���������
@�������أ����յ����ֽ�
@�޸�ʱ�䣺2022/02/24
@���÷�����SOFT_IC_ReadByte(0x12);
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char SOFT_IIC_Read_Byte(unsigned char ack)
{
  unsigned char  i,tem=0;
  SDA_IN; //SDA����Ϊ����ģʽ �ȴ����մӻ���������
  IIC_SCL_L;
  IIC_SDA_L;
  for(i=0;i<8;i++ )
  {
    SOFT_IIC_delay(2);
    IIC_SCL_H; 
    if(IIC_SDA_READ) tem++; //�ӻ����͵ĵ�ƽ
    if(i < 7) tem <<= 1;
    SOFT_IIC_delay(2);    
    IIC_SCL_L;       
  }					 
  if(ack)
    SOFT_IIC_Ack(); //����ACK 
  else
    SOFT_IIC_NAck(); //����nACK  
  return tem;
}


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char IIC_ReadByteFromSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf)
@����˵����ģ��IIC��ȡָ���豸 ָ���Ĵ�����һ��ֵ
@����˵����I2C_Addr��Ŀ���豸��ַ reg ��Ŀ��Ĵ��� buf ����Ŷ����ֽ�
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_ReadByteFromSlave(0xD0, 0x75, buf);   //�� IIC��ַΪ 0xD0������SOFT_I2C���Ĵ���0x75
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_ReadByteFromSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf)
{
  SOFT_IIC_Start();	
  SOFT_IIC_Send_Byte(I2C_Addr);	 //���ʹӻ���ַ
  if(SOFT_IIC_Wait_Ack()) //����ӻ�δӦ�������ݷ���ʧ��
  {
    SOFT_IIC_Stop();
    return 1;
  }
  SOFT_IIC_Send_Byte(reg); //���ͼĴ�����ַ
  SOFT_IIC_Wait_Ack();	  
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(I2C_Addr+1); //�������ģʽ			   
  SOFT_IIC_Wait_Ack();
  *buf=SOFT_IIC_Read_Byte(0);	   
  SOFT_IIC_Stop(); //����һ��ֹͣ����
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char IIC_WriteByte(unsigned char I2C_Addr,unsigned char reg,unsigned char data_t)
@����˵����ģ��IICдָ���豸 ָ���Ĵ�����һ��ֵ
@����˵����I2C_Addr��Ŀ���豸��ַ reg��Ŀ��Ĵ���data��д�������
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_ReadByte(0xD0, 0X6B, 0X80);   //IIC��ַΪ 0xD0������SOFT_I2C���Ĵ���0x6B д��0x80
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_ReadByte(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf)
{
  SOFT_IIC_Start();	
  SOFT_IIC_Send_Byte(I2C_Addr);	 //���ʹӻ���ַ
  if(SOFT_IIC_Wait_Ack()) //����ӻ�δӦ�������ݷ���ʧ��
  {
    SOFT_IIC_Stop();
    return 1;
  }
  SOFT_IIC_Send_Byte(reg); //���ͼĴ�����ַ
  SOFT_IIC_Wait_Ack();	  
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(I2C_Addr+1); //�������ģʽ			   
  SOFT_IIC_Wait_Ack();
  *buf=SOFT_IIC_Read_Byte(0);	   
  SOFT_IIC_Stop(); //����һ��ֹͣ����
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char IIC_WriteByteToSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char data_t)
@����˵����ģ��IICдָ���豸 ָ���Ĵ�����һ��ֵ
@����˵����I2C_Addr��Ŀ���豸��ַ reg��Ŀ��Ĵ���data��д�������
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_ReadByteFromSlave(0xD0, 0X6B, 0X80);   //IIC��ַΪ 0xD0������SOFT_I2C���Ĵ���0x6B д��0x80
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_WriteByteToSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char data_t)
{
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(I2C_Addr); //���ʹӻ���ַ
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop();
    return 1; //�ӻ���ַд��ʧ��
  }
  SOFT_IIC_Send_Byte(reg); //���ͼĴ�����ַ
  SOFT_IIC_Wait_Ack();	  
  SOFT_IIC_Send_Byte(data_t); 
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop(); 
    return 1; //����д��ʧ��
  }
  SOFT_IIC_Stop(); //����һ��ֹͣ����
  
  //return 1; //status == 0;
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char IIC_ReadMultByteFromSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char* data_t)
@����˵����ģ��IICдָ���豸 ָ���Ĵ�����n��ֵ
@����˵����dev��Ŀ���豸��ַ  reg��Ŀ��Ĵ���  len��д�볤��  data�����д������
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_ReadMultByteFromSlave(0xD0, 0X6B, 1, 0X80);   //��Ĵ���0x6Bд��0x80
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_ReadMultByteFromSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char *data_t)
{
  unsigned char  count = 0;
  unsigned char  temp;
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(dev); //���ʹӻ���ַ
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop(); 
    return 1; //�ӻ���ַд��ʧ��
  }
  SOFT_IIC_Send_Byte(reg); //���ͼĴ�����ַ
  SOFT_IIC_Wait_Ack();	  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(dev+1); //�������ģʽ	
  SOFT_IIC_Wait_Ack();
  for(count=0;count<len;count++)
  {
    if(count!=(len-1))
      temp = SOFT_IIC_Read_Byte(1); //��ACK�Ķ�����
    else  
      temp = SOFT_IIC_Read_Byte(0); //���һ���ֽ�NACK
    
    data_t[count] = temp;
  }
  SOFT_IIC_Stop(); //����һ��ֹͣ����
  //return count;
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char IIC_WriteMultByteToSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char* data_t)
@����˵����ģ��IICдָ���豸 ָ���Ĵ�����n��ֵ
@����˵����dev��Ŀ���豸��ַ  reg��Ŀ��Ĵ���  len��д�볤��  data�����д������
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����IIC_WriteMultByteToSlave(0xD0, 0X6B, 1, 0X80);   //��Ĵ���0x6Bд��0x80
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_WriteMultByteToSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char* data_t)
{
  
  unsigned char  count = 0;
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(dev); //���ʹӻ���ַ
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop();
    return 1; //�ӻ���ַд��ʧ��
  }
  SOFT_IIC_Send_Byte(reg); //���ͼĴ�����ַ
  SOFT_IIC_Wait_Ack();	  
  for(count=0;count<len;count++)
  {
    SOFT_IIC_Send_Byte(data_t[count]); 
    if(SOFT_IIC_Wait_Ack()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
    {
      SOFT_IIC_Stop();
      return 1; //����д��ʧ��
    }
  }
  SOFT_IIC_Stop(); //����һ��ֹͣ����
  
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int LQ_I2C_Write(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf)
@����˵����ģ��SPI��д���ݼ�����
@����˵����addr �����豸��ַ  reg ��Ŀ��Ĵ���  len ��д�볤��  *buf�����д������
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����LQ_I2C_Write(0x68, 0x38, 1, tmp)
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LQ_I2C_Write(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *dat)
{
  unsigned int i = 0;
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(addr << 1);
  SOFT_IIC_Wait_Ack();
 
  SOFT_IIC_Send_Byte(reg);
  SOFT_IIC_Wait_Ack();
  
  for (i = 0; i < len; i++) 
  {
    SOFT_IIC_Send_Byte(dat[i]);
    SOFT_IIC_Wait_Ack();   
  }
  SOFT_IIC_Stop();
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int LQ_I2C_Read(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf)
@����˵����ģ��SPI��д���ݼ�����
@����˵����addr �����豸��ַ  reg ��Ŀ��Ĵ���  len ��д�볤��  *buf�����д������
@�������أ�1ʧ�� 0�ɹ�
@�޸�ʱ�䣺2022/02/24
@���÷�����LQ_I2C_Read(0x68, 0x38, 1, tmp)
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LQ_I2C_Read(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf)
{
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(addr << 1);
  SOFT_IIC_Wait_Ack();
  
  SOFT_IIC_Send_Byte(reg);
  SOFT_IIC_Wait_Ack();
  SOFT_IIC_delay(2);//Ӧ����Ҫһ����ʱ��
  SOFT_IIC_Start();
  
  SOFT_IIC_Send_Byte((addr << 1)+1);
  SOFT_IIC_Wait_Ack();
  while (len) 
  {
    if (len == 1)
      *buf = SOFT_IIC_Read_Byte(0);
    else
      *buf = SOFT_IIC_Read_Byte(1);
    buf++;
    len--;
  }
  SOFT_IIC_Stop();
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@��    д������Ƽ�
@E-mail  ��chiusir@163.com
@����IDE��KEIL5.25.3�����ϰ汾
@ʹ��ƽ̨�������������ܿƼ�ȫ������С��
@�����£�2022��02��19�գ��������£����ע���°棡
@���ܽ��ܣ�
@�����Ϣ�ο����е�ַ
@��    վ��http://www.lqist.cn
@�Ա����̣�http://longqiu.taobao.com
@����汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _KEY_H
#define _KEY_H
#include "sys.h"



//�������ź궨��
#define KEY1_Pin        GPIOB,GPIO_PIN_0  	//KEY0����PB0
#define KEY2_Pin        GPIOB,GPIO_PIN_1  	//KEY1����PB1
#define KEY3_Pin       	GPIOB,GPIO_PIN_2  	//KEY2����PB2
#define KEY4_Pin        GPIOA,GPIO_PIN_11  	//KEY3����PA11
#define KEY5_Pin        GPIOA,GPIO_PIN_12  	//KEY4����PA12

//����ѡ��
typedef enum
{
  KEY1=1,
	KEY2,
	KEY3,
	KEY4,
	KEY5
}key_num;

void KEY_Init(void);				//������ʼ��
u8 Read_key(key_num num);		//��ȡ����״̬
void Test_key(void);				//�������ܲ���

#endif



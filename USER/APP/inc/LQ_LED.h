/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@��    д������Ƽ�
@E-mail  ��chiusir@163.com
@����IDE��KEIL5.25.3�����ϰ汾
@ʹ��ƽ̨�������������ܿƼ�ȫ������С��
@�����£�2022��02��19�գ��������£����ע���°棡
@���ܽ��ܣ�LED
@�����Ϣ�ο����е�ַ
@��    վ��http://www.lqist.cn
@�Ա����̣�http://longqiu.taobao.com
@����汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _LED_H
#define _LED_H
#include "sys.h"

#define led GPIOB,GPIO_PIN_15		//LED�ܽŶ���


//ö��LED״̬
typedef enum
{
  ON=0,  //��
  OFF=1, //��
  RVS=2, //��ת
}LEDs_e;

#define LED0 PBout(15)   	//LED0

void LED_Init(void);
void LED_Ctrl(LEDs_e sta);
void Test_Led(void);

#endif



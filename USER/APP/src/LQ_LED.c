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
#include "include.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�LED_Init
@����˵����LED�Ƴ�ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/2/19
@���÷�����LED_Init()
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOB_CLK_ENABLE();           	//����GPIOBʱ��

	GPIO_Initure.Pin=GPIO_PIN_15; 			  //PB15
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//�������
	GPIO_Initure.Pull=GPIO_PULLUP;          	//����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	HAL_GPIO_WritePin(led,GPIO_PIN_RESET);		//PB15��1��Ĭ�ϳ�ʼ�������
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�LED_Ctrl
@����˵��������LED״̬
@����˵����sta:״̬ ON�� OFF�ر� RVS��ת
@�������أ���
@�޸�ʱ�䣺2022/2/19
@���÷�����LED_Ctrl(ON);
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Ctrl(LEDs_e sta)
{
	if(sta==ON)        HAL_GPIO_WritePin(led, GPIO_PIN_RESET);
	else if(sta==OFF)  HAL_GPIO_WritePin(led, GPIO_PIN_SET);
	else if(sta==RVS)  HAL_GPIO_WritePin(led, (HAL_GPIO_ReadPin(led)?GPIO_PIN_RESET:GPIO_PIN_SET));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�Test_Led
@����˵��������LED
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/2/19
@���÷�����Test_Led()
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_Led(void)
{
  LED_Init();
  OLED_P6x8Str(0,1,"LQ_LED Test");         //�ַ���
  while(1)
  {
    LED_Ctrl(RVS); 
    delay_ms(200);
  }
}



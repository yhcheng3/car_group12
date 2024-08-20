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
@�������ƣ�KEY_Init
@����˵����������ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/2/19
@���÷�����KEY_Init();
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();                       //����GPIOAʱ��
	__HAL_RCC_GPIOB_CLK_ENABLE();                        //����GPIOBʱ��

    GPIO_Initure.Pin=GPIO_PIN_11 | GPIO_PIN_12;  	    //PA11  12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//����
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);    
	
	GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;  //PB0  1  2
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//����
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;            //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�Read_key
@����˵�����ж���ذ����Ƿ���
@����˵����num :�������
@�������أ�״̬ 1������ 0��û����
@�޸�ʱ�䣺2022/2/19
@���÷�����if(Read_key(KEY0)) KEY0�Ƿ���
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
u8 Read_key(key_num num)
{
	switch(num)
	{
		case 1:
			if(HAL_GPIO_ReadPin(KEY1_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY1_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 2:
			if(HAL_GPIO_ReadPin(KEY2_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY2_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 3:
			if(HAL_GPIO_ReadPin(KEY3_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY3_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 4:
			if(HAL_GPIO_ReadPin(KEY4_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY4_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 5:
			if(HAL_GPIO_ReadPin(KEY5_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY5_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
	}
	return 0;
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�Test_key
@����˵�����������ܲ��Ժ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/2/19
@���÷�����Test_key();
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_key(void)
{
	char txt[32];
	uart_init(USART_2, 115200);			//��ʼ������
	LED_Init();							//��ʼ��LED	
	KEY_Init();							//��ʼ������
	OLED_Init();						//��ʼ����Ļ
	
	sprintf(txt, "KEY TEST");
	OLED_P8x16Str(40, 0, txt);
	sprintf(txt, "The KEY is");
	while(1)
	{
		if(Read_key(KEY1))
		{
			sprintf(txt, "The KEY is KEY1");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY2))
		{
			sprintf(txt, "The KEY is KEY2");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY3))
		{
			sprintf(txt, "The KEY is KEY3");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY4))
		{
			sprintf(txt, "The KEY is KEY4");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY5))
		{
			sprintf(txt, "The KEY is KEY5");
			LED_Ctrl(RVS);
		}
		
		OLED_P8x16Str(0, 2, txt);
	}
}







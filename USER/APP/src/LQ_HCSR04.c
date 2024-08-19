#include "LQ_HCSR04.h"
#include "include.h"

/*
1.����:
	  ������ģ���뿪�����öŰ�������������PB9��Trig, PB8��Echo, GND��GND 5V��VCC��
2.ʵ��:
	  ���س���ȫ�����У���װOLED��Ļ����Ļ��ʾ����ֵ��˵��ʵ��ɹ�
*/
/*-----------------------------------------------------------
��ʱ����ʼ�� ��ʼ����ʱ��2 ��ʱʱ��Ϊ10us ��������������ʱ��ϵͳ��Ƶ72MHz

-----------------------------------------------------------*/
/**************************************************
 * ������: TIM2_Init(u16 arr,u16 psc)
 * ��  ��: ��ʱ��2��ʼ����������
 * ����ֵ: װ��ֵarr��Ԥ��Ƶϵ��ֵpsc
 * /�жϷ������� stm32f1xx_it.c��
 ***************************************************/
 
TIM_HandleTypeDef TIM2_Handler;      //��ʱ����� 

void LQ_TIM2_Init(u16 arr,u16 psc)
{  
    __HAL_RCC_TIM2_CLK_ENABLE();                            //ʹ��TIM3ʱ��
    TIM2_Handler.Instance=TIM2;                             //ͨ�ö�ʱ��3
    TIM2_Handler.Init.Prescaler=psc;                        //��Ƶϵ��
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;       //���ϼ�����
    TIM2_Handler.Init.Period=arr;                           //�Զ�װ��ֵ
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; //ʱ�ӷ�Ƶ����
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler);   //ʹ�ܶ�ʱ��3�Ͷ�ʱ��2�����жϣ�TIM_IT_UPDATE   
	HAL_NVIC_SetPriority(TIM2_IRQn,0,1);    //�����ж����ȼ�����ռ���ȼ�1�������ȼ�3
	HAL_NVIC_EnableIRQ(TIM2_IRQn);          //����ITM2�ж�  
}

void Ultrasonic_Init()
{
	//��ʼ�� ������ģ������
    GPIO_InitTypeDef GPIO_Initure;
  
	__HAL_RCC_GPIOB_CLK_ENABLE();                       //����GPIOBʱ��

    GPIO_Initure.Pin=GPIO_PIN_9;  	                    //PB9 // Trig - PB9���
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      	    //���
    GPIO_Initure.Pull=GPIO_PULLUP;        			    //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);    
	
	GPIO_Initure.Pin=GPIO_PIN_8;                       //PB8 // Echo - PB8��
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//����
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;            //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
	LQ_TIM2_Init(719,0);                    // ��ʱ�����ڼ���
}
extern uint32_t TimeCounter;	 //���ڳ�������ʱ,�ñ���λ��stm32f1xx_it.c��
uint32_t HalTime1=0, HalTime2=0;
uint32_t Get_Distance(void) // ����ʱ�䵥λ10us
{
	uint32_t Distance = 0;
	/*��Trig��������ʮ΢��*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	//���Echo�����ǵ͵�ƽ��һֱ�ȴ���ֱ��ΪEcho����Ϊ�ߵ�ƽ��Ϊ�ߵ�ƽ��˵�����ܵ��˷��ص��ź�
    HalTime1= TimeCounter;
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
    {
        if(TimeCounter - HalTime1 > 2200)  break;  //10us *2000      
    }
	//��¼�´�ʱ��ʱ��ֵ��10us��ʱ�����ۼӵı���������ʱ�䵥λΪ10us��
//    printf("TIMEA:%d \r\n", TimeCounter - HalTime1); // ��ӡ������ȡ����ʱ��ֵ��С
	HalTime1 = TimeCounter;
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1)	//�ȴ��ߵ�ƽʱ�����
    {
        if(TimeCounter - HalTime1 > 4500)  break;
    }
//    printf("TIMEB:%d \r\n", TimeCounter - HalTime1); // ��ӡ������ȡ����ʱ��ֵ��С
	//����ʱ�� �ж�ʱ������ǲ��Ǵ�ͷ��ʼ�ۼ��ˣ�����������Χʱ���0��ʼ�ۼӣ�
	if (TimeCounter > HalTime1)
	{
		//����ߵ�ƽ��ʱ�䳤��
		HalTime2 = TimeCounter - HalTime1;
//        Distance=(uint32_t)((float)HalTime2/58*10);//���뵥λcm,����340M/S��ʱ��*�ٶ�/2=����
        Distance = (uint32_t)(((float)HalTime2 *17)/100);

	}
	if( 2.5 < Distance)
        return Distance;
    else 
        return 1;
}

void Test_Ultrasonic()
{
	uint16_t Dis = 0.0;
	char txt[32];
	Ultrasonic_Init();
	OLED_Init();						 // OLED��ʼ��
	OLED_P6x8Str(10, 0, "Test HCSR04 "); // �ַ���
	delay_ms(5);
	while (1)
	{
		Dis = Get_Distance();
		sprintf(txt, "Dis=%3d cm", Dis);
		OLED_P8x16Str(10, 2, txt); // ��ʾ�ַ���
		printf("Distance:%dcm\n", Dis);
		LED_Ctrl(RVS);
		delay_ms(100);
	}
}

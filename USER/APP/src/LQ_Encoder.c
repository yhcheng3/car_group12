#include "include.h"


TIM_HandleTypeDef 	TIM_Handler;
TIM_Encoder_InitTypeDef  Encoder_sConfig;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Encoder_Init_TIM2(void)
@����˵����������1��ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��Encoder_Init_TIM2();
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Encoder_Init_TIM2(void)
{
	GPIO_InitTypeDef 	GPIO_Initure;
	__HAL_RCC_TIM2_CLK_ENABLE();            	//ʹ�ܶ�ʱ��2
	__HAL_RCC_GPIOA_CLK_ENABLE();            	//����GPIOAʱ��
	__HAL_AFIO_REMAP_TIM2_PARTIAL_1();          //���踴��1
	
	GPIO_Initure.Pin=GPIO_PIN_0 |GPIO_PIN_1;    //PA0 1
	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //����ģʽ
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;	 //����
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 

	TIM_Handler.Instance=TIM2;             		//��ʱ��1
	TIM_Handler.Init.Prescaler=0;           	//��ʱ����Ƶ
	TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
	TIM_Handler.Init.Period=ENCODER_TIM_PERIOD;     //�Զ���װ��ֵ
	TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//����Ƶ
	TIM_Handler.Init.RepetitionCounter = 0;//�ظ�������
	TIM_Handler.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
    Encoder_sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    Encoder_sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_sConfig.IC1Filter = 0;
    Encoder_sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	Encoder_sConfig.IC2Filter = 0;
	
	HAL_TIM_Encoder_Init(&TIM_Handler, &Encoder_sConfig);

    HAL_TIM_Encoder_Start(&TIM_Handler, TIM_CHANNEL_ALL);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Encoder_Init_TIM3(void)
@����˵����������2��ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��Encoder_Init_TIM3();
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Encoder_Init_TIM3(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;

	__HAL_RCC_TIM3_CLK_ENABLE();                //ʹ�ܶ�ʱ��3
	__HAL_RCC_GPIOB_CLK_ENABLE();               //����GPIOBʱ��
    __HAL_AFIO_REMAP_TIM3_PARTIAL();            //ʹ�ܸ�����ӳ��,PB4,B5

	
	GPIO_Initure.Pin=GPIO_PIN_4| GPIO_PIN_5;    //PB4 5
	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //����ģʽ
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	
	TIM_Handler.Instance=TIM3;             		//��ʱ��3
	TIM_Handler.Init.Prescaler=0;           	//��ʱ����Ƶ
	TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ���ģʽ
	TIM_Handler.Init.Period=ENCODER_TIM_PERIOD;             //�Զ���װ��ֵ
	TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//����Ƶ
	TIM_Handler.Init.RepetitionCounter = 0;                 //�ظ�������
	TIM_Handler.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
    Encoder_sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    Encoder_sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_sConfig.IC1Filter = 0;
    Encoder_sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	Encoder_sConfig.IC2Filter = 0;
	
	HAL_TIM_Encoder_Init(&TIM_Handler, &Encoder_sConfig);

    HAL_TIM_Encoder_Start(&TIM_Handler, TIM_CHANNEL_ALL);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Encoder_Init_TIM4(void)
@����˵����������3��ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��Encoder_Init_TIM4();
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Encoder_Init_TIM4(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;
	__HAL_RCC_TIM4_CLK_ENABLE();                //ʹ�ܶ�ʱ��4
	__HAL_RCC_GPIOB_CLK_ENABLE();               //����GPIOBʱ��
	
	GPIO_Initure.Pin=GPIO_PIN_6 | GPIO_PIN_7;   //PB6 7
	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //����ģʽ
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	
	TIM_Handler.Instance=TIM4;             		//��ʱ��4
	TIM_Handler.Init.Prescaler=0;           	//��ʱ����Ƶ
	TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //���ϼ���ģʽ
	TIM_Handler.Init.Period=ENCODER_TIM_PERIOD;             //�Զ���װ��ֵ
	TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//����Ƶ
	TIM_Handler.Init.RepetitionCounter = 0;                 //�ظ�������
	TIM_Handler.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
    Encoder_sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    Encoder_sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_sConfig.IC1Filter = 0;
    Encoder_sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	Encoder_sConfig.IC2Filter = 0;
	
	HAL_TIM_Encoder_Init(&TIM_Handler, &Encoder_sConfig);

    HAL_TIM_Encoder_Start(&TIM_Handler, TIM_CHANNEL_ALL);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int Read_Encoder(u8 TIMX)
@����˵������ȡ����������
@����˵����u8 TIMX��ʱ��
@�������أ���������
@�޸�ʱ�䣺2022/02/24
@��    ע��ECPULSE1 = Read_Encoder(3); //���� ĸ���ϱ�����3��С��ǰ��Ϊ��ֵ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
        case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;
         break;
        case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;
         break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;
         break;	
		default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Test_ENC(void)
@����˵�������Գ���TFT1.8��ʾ
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע���жϷ������� stm32f1xx_it.c��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_ENC(void)
{
	int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;
	char txt[30];
	Encoder_Init_TIM2();
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	
	while(1)
	{
//		enc = (uint32_t)(__HAL_TIM_GET_COUNTER(&TIM_Handler));
		
		ECPULSE1=Read_Encoder(2);
		sprintf(txt,"E1:%04d ",ECPULSE1);
	    OLED_P6x8Str(0,2,txt);	
		ECPULSE2=Read_Encoder(3);
		sprintf(txt,"E2:%04d ",ECPULSE2);
	    OLED_P6x8Str(0,3,txt);	
		ECPULSE3=Read_Encoder(4);
		sprintf(txt,"E3:%04d ",ECPULSE3);
	    OLED_P6x8Str(0,4,txt);	
        
        delay_ms(100);   //���� �ۼ�ʱ��
		
	}
}

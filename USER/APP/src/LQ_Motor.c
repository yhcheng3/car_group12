/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"


//uint16_t BatVolt;         // ��ص�ѹ�ɼ�

//int16_t MotorDuty1 = 500;      // �������ռ�ձ���ֵ
//int16_t MotorDuty2 = 500;      // �������ռ�ձ���ֵ
//int32_t NowTime = 0;
//uint16_t BatVolt = 0;           // ��ص�ѹ�ɼ�


//#define MOTOR_FREQUENCY    PWM_DUTY_MAX

//#define USE7843or7971   //USEDRV8701 ʹ������ͬ������ģ�飬ѡ���Ӧ�ĺ궨��
#define USEDRV8701

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void TIM1_PWM_Init(u16 arr,u16 psc)
@����˵����PWM��ʼ��
@����˵����arr����װ��ֵ��psc��Ԥ��Ƶֵ
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����	TIM1_PWM_Init(7199, 0);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM1_PWM_Init(u16 arr,u16 psc)
{
    TIM_HandleTypeDef 	TIM1_Handler;
    TIM_OC_InitTypeDef  TIM1_CH1Handler;    		//��ʱ��1ͨ��1���
    
	TIM1_Handler.Instance=TIM1;             		//��ʱ��1
	TIM1_Handler.Init.Prescaler=psc;           	    //��ʱ����Ƶ
	TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
	TIM1_Handler.Init.Period=arr;              	    //�Զ���װ��ֵ
	TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&TIM1_Handler);           	    //��ʼ��PWM
	HAL_TIM_Base_Start_IT(&TIM1_Handler);           //ʹ�ܶ�ʱ��1�Ͷ�ʱ��1�����жϣ�TIM_IT_UPDATE  

	TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM2; 		//ģʽѡ��PWM1
	TIM1_CH1Handler.Pulse=	0;            			//���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
	TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ��  

	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//����TIM1ͨ��1    
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_2);//����TIM1ͨ��2   
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_3);//����TIM1ͨ��3 
	
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);//����PWMͨ��1
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_2);//����PWMͨ��2
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_3);//����PWMͨ��3

    HAL_TIM_PWM_Start_IT(&TIM1_Handler, TIM_CHANNEL_1);  //��������ʱ��PWM���
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void MotorInit(void)
@����˵���������ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����MotorInit();
@��    ע���������
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void MotorInit(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;
	
	__HAL_RCC_TIM1_CLK_ENABLE();                //ʹ�ܶ�ʱ��1
	__HAL_RCC_GPIOA_CLK_ENABLE();               //����GPIOAʱ��  
	__HAL_RCC_GPIOB_CLK_ENABLE();               //����GPIOBʱ��

	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;      //PA8 9 10
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//����
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 

	GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;    //PB12 13 14
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;          //���
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	TIM1_PWM_Init(7199, 0);
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void TIM1_PWM_Init(u16 arr,u16 psc)
@����˵����PWM��ʼ��
@����˵����arr����װ��ֵ��psc��Ԥ��Ƶֵ��duty��ռ�ձ�
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����	TIM4_PWM_Init(7199, 0,duty);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM4_PWM_Init(u16 arr,u16 psc, u16 duty)
{
    TIM_HandleTypeDef 	TIM4_Handler;
	TIM_OC_InitTypeDef  TIM4_CH1Handler;    		//��ʱ��4ͨ��1���
	TIM4_Handler.Instance=TIM4;             		//��ʱ��1
	TIM4_Handler.Init.Prescaler=psc;           	//��ʱ����Ƶ
	TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//���ϼ���ģʽ
	TIM4_Handler.Init.Period=arr;              	//�Զ���װ��ֵ
	TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&TIM4_Handler);           	//��ʼ��PWM
	HAL_TIM_Base_Start_IT(&TIM4_Handler); 			//ʹ�ܶ�ʱ��4�����жϣ�TIM_IT_UPDATE  

	TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; 			//ģʽѡ��PWM1
	TIM4_CH1Handler.Pulse=	duty;            			//���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
	TIM4_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //����Ƚϼ���Ϊ��  
	
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);//����TIM4ͨ��1 
	HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);//����PWMͨ��1
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void ServoInit(u16 freq)
@����˵���������ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����ServoInit(100);
@��    ע������������
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ServoInit(u16 duty, u16 psc)
{
	GPIO_InitTypeDef  GPIO_Initure;
	__HAL_RCC_TIM4_CLK_ENABLE();            //ʹ�ܶ�ʱ��4
	__HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��

	GPIO_Initure.Pin=GPIO_PIN_6;      			//PB6
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //�����������
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//����
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	if(psc == 100)							 //100Hz
		TIM4_PWM_Init(9865, 72, duty);
	else	                                  //50Hz
		TIM4_PWM_Init(9999, 144, duty);
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Brushless_Motor_Init(u16 freq)
@����˵������ˢ���PWM��ʼ��
@����˵����freq����ʼ��Ƶ��
@�������أ���
@�޸�ʱ�䣺2024/06/11
@���÷�����	Brushless_Motor_Init(50)��
@��    ע��ʹ����ˢ���������ʱ������Ҫʹ����ˢ��غ�������������ܽ���ͬ�������ͻ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Brushless_Motor_Init(u16 freq)
{
    GPIO_InitTypeDef  GPIO_Initure;
	__HAL_RCC_TIM1_CLK_ENABLE();            //ʹ�ܶ�ʱ��
	__HAL_RCC_GPIOA_CLK_ENABLE();           //����GPIOʱ��

	
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;      //PA8 9 10
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //�����������
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//����
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 
    if(freq == 100)
		TIM1_PWM_Init(9865, 72);          //100Hz
	else	                                 
		TIM1_PWM_Init(9999, 144);         //50Hz
}



//-----------------------------------------------------------------------------------

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void MotorCtrl3w(int16_t motor1, int16_t motor2,int16_t motor3)
@����˵����������ƺ���
@����˵����motor1:���1ռ�ձȣ���Χ��-10000��10000��
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����MotorCtrl3w(pid1.out, pid2.out, pid3.out);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void MotorCtrl3w(int16_t motor1, int16_t motor2, int16_t motor3)
{  
//		����TIM1ͨ��1��ռ�ձ�
//		compare:�Ƚ�ֵ
//    TIM1->CCR1=compare; 
  if (motor1 > 0)
  {
    TIM1->CCR1=motor1;	    //C6
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_RESET);   //G5	
  }
  else
  {
    TIM1->CCR1=0-motor1;
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_SET);
  }
  
 if (motor2 > 0)
  {
    TIM1->CCR2=motor2;	 //C7
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_RESET);//G6	
  }
  else
  {
    TIM1->CCR2=0-motor2;
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_SET);
  }
  
	if (motor3 > 0)
  {
    TIM1->CCR3=motor3;	     //C8
    HAL_GPIO_WritePin(MTGPIO3p, GPIO_PIN_RESET);    //G7
  }
  else
  {
    TIM1->CCR3=0-motor3;
    HAL_GPIO_WritePin(MTGPIO3p, GPIO_PIN_SET);
  }
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void MotorCtrl(int16_t motor1, int16_t motor2)
@����˵����˫·�����������
@����˵����motor1:���1ռ�ձȣ���Χ��-10000��10000��
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����MotorCtrl(pid1.out, pid2.out);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

void MotorCtrl(int16_t motor1, int16_t motor2)
{
	if (motor1 > 0)
  {
    TIM1->CCR1=motor1;	    //C6
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_RESET);   //G5	
  }
  else
  {
    TIM1->CCR1=0-motor1;
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_SET);
  }
  
 if (motor2 > 0)
  {
    TIM1->CCR2=motor2;	 //C7
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_RESET);//G6	
  }
  else
  {
    TIM1->CCR2=0-motor2;
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_SET);
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void ServoCtrl (uint16_t duty)
@����˵����������ƺ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����ServoCtrl(750);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ServoCtrl(int16_t motor1)
{
    if (motor1 > 0)
    {
        TIM4->CCR1=motor1;	     //B6
    }
    else
    {
        TIM4->CCR1=0-motor1;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Brushless_Motor_Ctrl(int16_t motor1, int16_t motor2, int16_t motor3)
@����˵������ˢ������ƺ���
@����˵����motor1:���1ռ�ձȣ���Χ
@�������أ���
@�޸�ʱ�䣺2024/06/11
@���÷�����MotorCtrl3w(pid1.out, pid2.out, pid3.out);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Brushless_Motor_Ctrl(int16_t motor1, int16_t motor2, int16_t motor3)
{  
//		����TIM1ͨ��1��ռ�ձ�,�������޷�
//		compare:�Ƚ�ֵ
//      TIM1->CCR1=compare; 

  if (motor1 > 0)
  {
    TIM1->CCR1=motor1;	      //PA8
  }
  
 if (motor2 > 0)
  {
    TIM1->CCR2=motor2;	      //PA9
  }
  
 if (motor3 > 0)
  {
    TIM1->CCR3=motor3;	      //PA10
  }

}


//-----------------------------------------------------------------------------------

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void TestMotor (void)
@����˵����������Ժ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����TestMotor();
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TestMotor (void)
{
    int16_t duty = 1000,flag = 0;
    char txt[16];
	OLED_Init();            	//OLED��ʼ��
	OLED_CLS(); 
    MotorInit();
	sprintf(txt, "Test Motor");
	OLED_P6x8Str(30,0,txt);
  while (1)
  {
    if(Read_key(KEY1) == 1)      //����KEY0����ռ�ձȼ�С
    {
//      if(duty > -PWM_DUTY_MAX)
        duty += 500;
    }
    if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
    {
//      if(duty < PWM_DUTY_MAX)   //��ռ�ձ�Ϊ10000
        duty = 0;
    }
    if(Read_key(KEY3) == 1)      //����KEY1����ռ�ձ���ֵ
    {
      duty -= 500;
    }
    
    
    /*  �Զ��Ӽ��ٲ��� */   //240814
    if(duty >= 3500)//6000
        flag = 1;
    if(duty <= -3500)
        flag = 0;
    
    if(flag == 0)
        duty += 100;
    else
        duty -= 100;
    
    MotorCtrl3w(duty,duty, duty);
//	ServoCtrl(duty);
    sprintf(txt, "PWM: %5d;", duty);
	OLED_P6x8Str(5,7,txt);
    
    LED_Ctrl(RVS);       //��ƽ��ת,LED��˸
    delay_ms(100);       //��ʱ�ȴ�
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void TestServo (void)
@����˵����������Ժ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����TestServo();
@��    ע��
@ע������:ע��!!!!!!!!!!!!!!!!!!һ��Ҫ�Զ����ǽ�������!!!!!!!!!!!!!!
ʹ������ĸ��������̣�
1.��ʹ�����ñ������ص�ѹ����ر�֤��ص�ѹ��7V���ϣ�������������Ӧ��
2.Ȼ��ȷ����������ѹ��SD5�����5V���磬S3010��6-7V���磡����
3.�Ѷ���Ķ���ȥ�����ö����������ת����
4.��д�������У��ö��ת������ֵ���������û��Ӧ�ظ�1-2�������ߵ��������PWMƵ�ʼ�ռ�ձȣ����ܿ�Ϊ׼��
5.����ܿغ�������ת�������֨֨�죬�Կ�ת������ʱ����װ�϶��̣�
6.����K0/K1ȷ�����������ת�����ޣ�������������Ϊ�����޷���ֹ�����ת�ջ٣�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TestServo (void)
{
	int16_t duty = 0;
  char txt[16];
  
	OLED_Init();            	//OLED��ʼ��
	OLED_CLS(); 
    ServoInit(0, 10);
	sprintf(txt, "Test Servo");
	OLED_P6x8Str(30,0,txt);
  while (1)
  {
    if(Read_key(KEY1) == 1)      //����KEY1����ռ�ձȼ�С
    {
        duty += 10;
    }
    if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
    {
      duty = 530;
    }
    if(Read_key(KEY3) == 1)      //����KEY3����ռ�ձ���ֵ
    {
        duty -= 10;
    }
		//�޷�
//		if(duty>100) duty=100;
//		if(duty<-100) duty=-100;
		
//	ServoCtrl(Servo_Center_Mid+duty);
    ServoCtrl(duty);
    sprintf(txt, "PWM: %5d;", duty);
	OLED_P6x8Str(5,3,txt);
    
    LED_Ctrl(RVS);       //��ƽ��ת,LED��˸
    delay_ms(200);              //��ʱ�ȴ�
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Test_Brushless_Motor(void)
@����˵������ˢ������Ժ�����ռ��ԭ������·�����
@����˵������
@�������أ���
@�޸�ʱ�䣺2024/06/11
@���÷�����Test_Brushless_Motor()��
@��    ע��ռ��ԭ����8701������·ֱ����ˢ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_Brushless_Motor(void)
{
    int16_t duty = 0;
    char txt[16];
  
	OLED_Init();            	//OLED��ʼ��
	OLED_CLS(); 
    Brushless_Motor_Init(50);   //50Hz
    Brushless_Motor_Ctrl(duty, duty, duty);
	sprintf(txt, "Test Brushless Motor");
	OLED_P6x8Str(0,0,txt);
  while (1)
  {
    if(Read_key(KEY1) == 1)      //����KEY0����ռ�ձȼ�С
    {
        duty += 20;
    }
    if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
    {
      duty = 530;
    }
    if(Read_key(KEY3) == 1)      //����KEY1����ռ�ձ���ֵ
    {
        duty -= 20;
    }
    Brushless_Motor_Ctrl(duty, duty, duty);
    sprintf(txt, "PWM: %5d;", duty);
	OLED_P6x8Str(2,7,txt);
    
    LED_Ctrl(RVS);       //��ƽ��ת,LED��˸
    delay_ms(200);       //��ʱ�ȴ�
  }
}


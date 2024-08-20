
#include "include.h"

ADC_HandleTypeDef ADC1_Handler;		//ADC���
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void ADC_init(void)
@����˵����ADC��ʼ��
@����˵����ch: ADC_channels 
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����ADC_InitConfig();
@��    ע��ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ADC_Init(void)
{ 
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;
	
	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;		//ADC����ʱ��
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//��Ƶ����6ʱ��Ϊ72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);							//����ADCʱ��
	
	ADC1_Handler.Instance=ADC1;
	ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //�Ҷ���
	ADC1_Handler.Init.ScanConvMode=DISABLE;                      //��ɨ��ģʽ
	ADC1_Handler.Init.ContinuousConvMode=DISABLE;                //�ر�����ת��
	ADC1_Handler.Init.NbrOfConversion=1;                         //1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //��ֹ����������ģʽ
	ADC1_Handler.Init.NbrOfDiscConversion=0;                     //����������ͨ����Ϊ0
	ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //�������
	HAL_ADC_Init(&ADC1_Handler);                                 //��ʼ�� 
	
	HAL_ADCEx_Calibration_Start(&ADC1_Handler);					 //У׼ADC
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
@����˵����ADC�ײ��������������ã�ʱ��ʹ��
@����˵����hadc:ADC���
@�������أ�
@�޸�ʱ�䣺2022/02/24
@���÷�����
@��    ע���˺����ᱻHAL_ADC_Init()����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //ʹ��ADC1ʱ��
    __HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
		__HAL_RCC_GPIOB_CLK_ENABLE();			//����GPIOAʱ��
    GPIO_Initure.Pin=GPIO_PIN_2 | GPIO_PIN_3;            //PA1
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //ģ��
    GPIO_Initure.Pull=GPIO_NOPULL;          //����������
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	  GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1;            //PA1
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //ģ��
    GPIO_Initure.Pull=GPIO_NOPULL;          //����������
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�uint16_t Get_Adc(uint32_t ch)   
@����˵������ȡADCת������
@����˵����ch: ͨ��ֵ 0~16��ȡֵ��ΧΪ��ADC_CHANNEL_0~ADC_CHANNEL_16
@�������أ�ADC������ֵ
@�޸�ʱ�䣺2022/02/24
@���÷�����vbat=ADC_Read(ADC0);
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
uint16_t Get_Adc(uint32_t ch)   
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;   
    ADC1_ChanConf.Channel=ch;                                   //ͨ��
    ADC1_ChanConf.Rank=1;                                       //��1�����У�����1
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;      //����ʱ��               
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //ͨ������
    HAL_ADC_Start(&ADC1_Handler);                               //����ADC
    HAL_ADC_PollForConversion(&ADC1_Handler,10);                //��ѯת��
 
	return (u16)HAL_ADC_GetValue(&ADC1_Handler);	        	//�������һ��ADC1�������ת�����
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned short ADC_Read(ADC_Channel_t  chn)
@����˵����ADC��βɼ�ȡƽ��ֵ
@����˵����chn    : ADCͨ��
@����˵����times  : �ɼ�����
@�������أ�ADC��ֵ
@�޸�ʱ�䣺2020��3��10��
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

uint16_t Get_Adc_Average(uint32_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned short ADC_Read(ADC_Channel_t  chn)
@����˵�����õ��¶�ֵ
@����˵����val   : ADC��ȡֵ
@�������أ��¶�ֵ
@�޸�ʱ�䣺2020��3��10��
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
//
//����ֵ:�¶�ֵ(������100��,��λ:��.)
int Get_Temp(uint32_t val)
{
	int Temp;
 	double temperate;
	
	temperate=(float)val*(3.3/4096);				//��ѹֵ 
	temperate=(1.43-temperate)/0.0043+25;			//ת��Ϊ�¶�ֵ 	 
	Temp=temperate*100;							//����100��.
	return Temp;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Test_ADC(void)
@����˵����ADC���Ժ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2020��3��10��
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_ADC(void)
{
	char txt[32];
	uint32_t val1;
	int Temp;
	uint32_t Val4, Val5, Val6, Val7;
	OLED_Init();   	//OLED��ʼ��
	OLED_CLS(); 		//����
	ADC_Init();			//ADC��ʼ��

	while(1)
	{
		//val1 = Get_Adc(1);
		Val4 = Get_Adc(2);
		Val5 = Get_Adc(3);
		Val6 = Get_Adc(8);
		Val7 = Get_Adc(9);
		Temp = Get_Temp(val1);
		
		printf("Temp:%04d ADC4:%04d ADC5:%04d ADC6:%04d ADC7:%04d\n",val1, Val4, Val5, Val6, Val7);
		sprintf(txt,"ADC4:%04d",Val4);
		OLED_P6x8Str(0,2,txt);
		sprintf(txt,"ADC5:%04d",Val5);
		OLED_P6x8Str(0,3,txt);
		sprintf(txt,"ADC6:%04d",Val6);
		OLED_P6x8Str(0,4,txt);
		sprintf(txt,"ADC7:%04d",Val7);
		OLED_P6x8Str(0,5,txt);
		
		sprintf(txt,"temp:%03d.%02d",Temp/100,Temp%100);
		OLED_P6x8Str(0,7,txt);
		delay_ms(100); 
	}
}




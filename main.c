/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@��    д������Ƽ�
@E-mail  ��chiusir@163.com
@����IDE ��KEIL5.25.3�����ϰ汾
@ʹ��ƽ̨�������������ܿƼ�ȫ������С��
@�����£�2022��02��19�գ��������£����ע���°棡
@���ܽ��ܣ�
@�����Ϣ�ο����е�ַ
@��    վ��http://www.lqist.cn
@�Ա����̣�http://longqiu.taobao.com
@����汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

// ��������ͷ�ļ�
#include "include.h"

int main(void)
{
  int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;
	char txt_pulse[30];
	
	float duty=100;
  char txt[16];

	float output;
	
//----PID----
	pid_param_t pid;
	PidInit(&pid);
	pid.kd = 0;
	pid.ki = 0.1;
	pid.kp = 40;
	
	
//-----------------------ϵͳ��ʼ������----------------------------
	HAL_Init();			  // ��ʼ��HAL��
	SystemClock_Config(); // ����ʱ��9��Ƶ,72M
	delay_init(72);		  // ��ʼ����ʱ����
	JTAG_Set(SWD_ENABLE); // ��SWD�ӿ� �������������SWD�ӿڵ���
//-----------------------------------------------------------------
	LED_Init();			// LED��ʼ��
	OLED_Init();		// OLED��ʼ��    
	Encoder_Init_TIM2();   //������ռ�ö�ʱ��2,3,4��Ӳ��AB�����ȡ
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
    OLED_Show_LQLogo(); // ��ʾLOGO
	delay_ms(500);		// ��ʱ�ȴ�
	OLED_CLS();			// ����
	MotorInit();
	// OLED_P8x16Str(10, 0,"Motor_Enc Test");       //��ʾ�ַ���
	while(1)
	{
		ECPULSE1=Read_Encoder(2);
		sprintf(txt_pulse,"E1:%04f ",(float)ECPULSE1);
	  OLED_P8x16Str(0,0,txt_pulse);	
		
		output = PidLocCtrl(&pid, (float)ECPULSE1-duty);
//		ECPULSE2=Read_Encoder(3);
//		sprintf(txt_pulse,"E2:%04d ",ECPULSE2);
//	    OLED_P8x16Str(0,4,txt_pulse);	
		
//		ECPULSE3=Read_Encoder(4);
//		sprintf(txt_pulse,"E3:%04d ",ECPULSE3);
//	    OLED_P8x16Str(0,6,txt_pulse);	
    
//		if(Read_key(KEY1) == 1)      //����KEY0����ռ�ձȼ�С
//    {
//      if(duty > -PWM_DUTY_MAX)
//        duty += 200;
//    }
//    if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
//    {
//      if(duty < PWM_DUTY_MAX)   //��ռ�ձ�Ϊ10000������7200
//        duty = 0;
//    }
//    if(Read_key(KEY3) == 1)      //����KEY1����ռ�ձ���ֵ
//    {
//      duty -= 200;
//    }
		
    if(Read_key(KEY1) == 1)      //����KEY2����ռ�ձȼӴ�
    {
      pid.kp += 1;
    }
		if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
    {
      pid.kp -= 1;
    }
		if(Read_key(KEY3) == 1)      //����KEY2����ռ�ձȼӴ�
    {
      pid.ki += 0.05;
    }
		if(Read_key(KEY4) == 1)      //����KEY2����ռ�ձȼӴ�
    {
      pid.ki -= 0.05;
    }

		sprintf(txt_pulse,"E2:%04f ",pid.kp);
	    OLED_P8x16Str(0,2,txt_pulse);	
		sprintf(txt_pulse,"E2:%04f ",pid.ki);
	    OLED_P8x16Str(0,4,txt_pulse);	
		sprintf(txt_pulse,"E2:%04f ",duty);
	    OLED_P8x16Str(0,6,txt_pulse);	
		

		output = constrain_float(output, -2000, 2000);
		
		MotorCtrl3w(output, 0, 0);
		// MotorCtrl3w(duty, duty, duty);
    // sprintf(txt, "PWM: %5d;", duty);
		// OLED_P8x16Str(10,0,txt);
		
		LED_Ctrl(RVS);       //��ƽ��ת,LED��˸
    delay_ms(100);   //�����������ۼ�ʱ�䣬ʱ��Խ�� ��ȡֵԽ��
		
	}
}


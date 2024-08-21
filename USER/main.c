/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编    写：龙邱科技
@E-mail  ：chiusir@163.com
@编译IDE ：KEIL5.25.3及以上版本
@使用平台：北京龙邱智能科技全向福来轮小车
@最后更新：2022年02月19日，持续更新，请关注最新版！
@功能介绍：
@相关信息参考下列地址
@网    站：http://www.lqist.cn
@淘宝店铺：http://longqiu.taobao.com
@软件版本：V1.0 版权所有，单位使用请先联系授权
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

// 包含所有头文件
#include "include.h"

int main(void)
{	
//------Basic Params------
	car_sensor_t carSensor;
	controller_t ctrl;
	
	int motor_flag;
	char txt[20];
	// int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;
//------Reverse Params--------
	init_ctrl(&ctrl);
	
	//-----------------------系统初始化配置----------------------------
	HAL_Init();			  // 初始化HAL库
	SystemClock_Config(); // 设置时钟9倍频,72M
	delay_init(72);		  // 初始化延时函数
	JTAG_Set(SWD_ENABLE); // 打开SWD接口 可以利用主板的SWD接口调试
	//-----------------------------------------------------------------
	LED_Init();			// LED初始化
	OLED_Init();		// OLED初始化   
	Encoder_Init_TIM2();   //������ռ�ö�ʱ��2,3,4��Ӳ��AB�����ȡ
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
    OLED_Show_LQLogo(); // ��ʾLOGO
	delay_ms(500);		// ��ʱ�ȴ�
	OLED_CLS();			// ����
	MotorInit();
	// OLED_P8x16Str(10, 0,"Motor_Enc Test");       //��ʾ�ַ���
	
	//-----------------------------------------------------------------
	while(1)
	{
		motor_flag = 1;
		set_control(&ctrl, &carSensor, motor_flag);
		
		//ECPULSE1=Read_Encoder(2);
		//sprintf(txt,"B:%04d ",ECPULSE1);
		sprintf(txt,"L:%04d ",ctrl.L);
	    OLED_P8x16Str(0,0,txt);	
		
		//ECPULSE2=Read_Encoder(3);
		//sprintf(txt,"L:%04d ",ECPULSE2);
		sprintf(txt,"R:%04d ",ctrl.R);
	    OLED_P8x16Str(0,2,txt);	
		
		//ECPULSE3=Read_Encoder(4);
		//sprintf(txt,"R:%04d ",ECPULSE3);
		sprintf(txt,"B:%04d ",ctrl.B);
	    OLED_P8x16Str(0,4,txt);
		
		sprintf(txt, "%d %d %d %d", carSensor.a, carSensor.b, carSensor.c, carSensor.d);
		OLED_P6x8Str(0,6, txt);
		
		//MotorCtrl3w(1000, 1000, 1000);
		MotorCtrl3w(ctrl.B, ctrl.L, ctrl.R);
		delay_ms(PERIOD);
	}
}

//----------PID Testing---------------
//int main(void)
// {
//   int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;
// 	char txt_pulse[30];
	
// 	int16_t duty = 1000,flag = 0;
//   char txt[16];
	
// 	int target = 80;
// 	float output = 0;
	
// //----PID----
// 	pid_param_t pid;
// 	PidInit(&pid);
// 	pid.kd = 10;
// 	pid.ki = 10;
// 	pid.kp = 10;
	
	
// //-----------------------ϵͳ��ʼ������----------------------------
// 	HAL_Init();			  // ��ʼ��HAL��
// 	SystemClock_Config(); // ����ʱ��9��Ƶ,72M
// 	delay_init(72);		  // ��ʼ����ʱ����
// 	JTAG_Set(SWD_ENABLE); // ��SWD�ӿ� �������������SWD�ӿڵ���
// //-----------------------------------------------------------------
// 	LED_Init();			// LED��ʼ��
// 	OLED_Init();		// OLED��ʼ��    
// 	Encoder_Init_TIM2();   //������ռ�ö�ʱ��2,3,4��Ӳ��AB�����ȡ
// 	Encoder_Init_TIM3();
// 	Encoder_Init_TIM4();
//     OLED_Show_LQLogo(); // ��ʾLOGO
// 	delay_ms(500);		// ��ʱ�ȴ�
// 	OLED_CLS();			// ����
// 	MotorInit();
// 	// OLED_P8x16Str(10, 0,"Motor_Enc Test");       //��ʾ�ַ���
// 	while(1)
// 	{
// 		ECPULSE1=Read_Encoder(2);
// 		sprintf(txt_pulse,"E1:%04d ",ECPULSE1);
// 	    OLED_P8x16Str(0,0,txt_pulse);	
		
// 		output = PidLocCtrl(&pid, (ECPULSE1 * 10) - duty);
// //		ECPULSE2=Read_Encoder(3);
// //		sprintf(txt_pulse,"E2:%04d ",ECPULSE2);
// //	    OLED_P8x16Str(0,4,txt_pulse);	
		
// //		ECPULSE3=Read_Encoder(4);
// //		sprintf(txt_pulse,"E3:%04d ",ECPULSE3);
// //	    OLED_P8x16Str(0,6,txt_pulse);	
    
// //		if(Read_key(KEY1) == 1)      //����KEY0����ռ�ձȼ�С
// //    {
// //      if(duty > -PWM_DUTY_MAX)
// //        duty += 200;
// //    }
// //    if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
// //    {
// //      if(duty < PWM_DUTY_MAX)   //��ռ�ձ�Ϊ10000������7200
// //        duty = 0;
// //    }
// //    if(Read_key(KEY3) == 1)      //����KEY1����ռ�ձ���ֵ
// //    {
// //      duty -= 200;
// //    }
		
//     if(Read_key(KEY1) == 1)      //����KEY2����ռ�ձȼӴ�
//     {
//       pid.kp += 1;
//     }
// 		if(Read_key(KEY2) == 1)      //����KEY2����ռ�ձȼӴ�
//     {
//       pid.kp += 1;
//     }
// 		if(Read_key(KEY3) == 1)      //����KEY2����ռ�ձȼӴ�
//     {
//       pid.kp += 1;
//     }
// 		if(Read_key(KEY3) == 1)      //����KEY2����ռ�ձȼӴ�
//     {
//       pid.kp += 1;
//     }
		
// 		if(Read_key(KEY4) == 1)      //����KEY2����ռ�ձȼӴ�
//     {
//       pid.ki += 1;
//     }
		
// 		if(Read_key(KEY5) == 1)      //����KEY2����ռ�ձȼӴ�
//     {
//       pid.kd += 1;
//     }
// 		sprintf(txt_pulse,"E2:%04f ",pid.kp);
// 	    OLED_P8x16Str(0,2,txt_pulse);	
// 		sprintf(txt_pulse,"E2:%04f ",pid.ki);
// 	    OLED_P8x16Str(0,4,txt_pulse);	
// 		sprintf(txt_pulse,"E2:%04f ",pid.kd);
// 	    OLED_P8x16Str(0,6,txt_pulse);	
		
		
// 		output = constrain_float(output, -2000, 2000);
		
// 		MotorCtrl3w(output, 0, 0);
// 		// MotorCtrl3w(duty, duty, duty);
//     // sprintf(txt, "PWM: %5d;", duty);
// 		// OLED_P8x16Str(10,0,txt);
		
// 		LED_Ctrl(RVS);       //��ƽ��ת,LED��˸
//     delay_ms(200);   //�����������ۼ�ʱ�䣬ʱ��Խ�� ��ȡֵԽ��
		
// 	}
// }


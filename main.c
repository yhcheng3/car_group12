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
	
	
//-----------------------系统初始化配置----------------------------
	HAL_Init();			  // 初始化HAL库
	SystemClock_Config(); // 设置时钟9倍频,72M
	delay_init(72);		  // 初始化延时函数
	JTAG_Set(SWD_ENABLE); // 打开SWD接口 可以利用主板的SWD接口调试
//-----------------------------------------------------------------
	LED_Init();			// LED初始化
	OLED_Init();		// OLED初始化    
	Encoder_Init_TIM2();   //编码器占用定时器2,3,4，硬件AB解码读取
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
    OLED_Show_LQLogo(); // 显示LOGO
	delay_ms(500);		// 延时等待
	OLED_CLS();			// 清屏
	MotorInit();
	// OLED_P8x16Str(10, 0,"Motor_Enc Test");       //显示字符串
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
    
//		if(Read_key(KEY1) == 1)      //按下KEY0键，占空比减小
//    {
//      if(duty > -PWM_DUTY_MAX)
//        duty += 200;
//    }
//    if(Read_key(KEY2) == 1)      //按下KEY2键，占空比加大
//    {
//      if(duty < PWM_DUTY_MAX)   //满占空比为10000，限制7200
//        duty = 0;
//    }
//    if(Read_key(KEY3) == 1)      //按下KEY1键，占空比中值
//    {
//      duty -= 200;
//    }
		
    if(Read_key(KEY1) == 1)      //按下KEY2键，占空比加大
    {
      pid.kp += 1;
    }
		if(Read_key(KEY2) == 1)      //按下KEY2键，占空比加大
    {
      pid.kp -= 1;
    }
		if(Read_key(KEY3) == 1)      //按下KEY2键，占空比加大
    {
      pid.ki += 0.05;
    }
		if(Read_key(KEY4) == 1)      //按下KEY2键，占空比加大
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
		
		LED_Ctrl(RVS);       //电平翻转,LED闪烁
    delay_ms(100);   //编码器计数累计时间，时间越大 读取值越大
		
	}
}


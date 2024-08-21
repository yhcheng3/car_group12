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

//-----Global Variables------
photoele_t photoele; // 光电传感器
encoder_t enc;
controller_t ctrl; 

extern u8 ReadBuff[1024];		

int main(void)
{
	//-----------------------系统初始化配置----------------------------
	HAL_Init();			  // 初始化HAL库
	SystemClock_Config(); // 设置时钟9倍频,72M
	delay_init(72);		  // 初始化延时函数
	JTAG_Set(SWD_ENABLE); // 打开SWD接口 可以利用主板的SWD接口调试
	//-----------------------------------------------------------------
	LED_Init();						//初始化LED	
	KEY_Init();						//初始化按键
	OLED_Init();    				//OLED初始化
	OLED_CLS();						//清屏
	uart_init(USART_2,115200);		//初始化串口
	uart_init(USART_3,115200);		//初始化串口
	Encoder_Init_TIM2();			//编码器初始化
	Encoder_Init_TIM3();			//编码器初始化
	Encoder_Init_TIM4();			//编码器初始化
	MotorInit();					//电机初始化
	Ultrasonic_Init();
//===============================================================
	//Car_main();			//超声波避障整车参考程序
	init_ctrl(&ctrl);
	
	while(1)
	{
		OLED_task();
		// delay_ms(PERIOD);
	}
}

void OLED_task(void) {
	char txt[20];
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

	sprintf(txt, "%d %d %d %d", photoele.a, photoele.b, photoele.c, photoele.d);
	OLED_P6x8Str(0,6, txt);
	
	sprintf(txt, "Mode %d", ctrl.mode);
	OLED_P6x8Str(0,7,txt);
}

void control(void) {
	if (Read_key(KEY1)) {
		ctrl.mode = 0;
	}
	if (Read_key(KEY2)) {
		ctrl.mode = 1;
	}

	if (ctrl.work_state == 0) {
		set_control(&ctrl, &photoele);
	} else {
		ultrasonic_avoid(&ctrl, &photoele);
	}
	MotorCtrl3w(ctrl.B, ctrl.L, ctrl.R);
}


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

struct CarSensor
{
	uint8_t a,b,c,d;
};


void Car_sensor(struct CarSensor *car_sensor) 
{
    car_sensor->a = Read_sensor(sensor1);
		car_sensor->b = Read_sensor(sensor2);
    car_sensor->c = Read_sensor(sensor3);
    car_sensor->d = Read_sensor(sensor4);
}


struct MotoPWM
{
	int B,L,R;
};

void moto_PWM(struct MotoPWM  *Moto_PWM, int car_V, int E_V, int MotorFlag) {
    if (MotorFlag)
		{
    Moto_PWM->L = car_V + E_V * 200;
		Moto_PWM->R = -car_V + E_V * 200;
		Moto_PWM->B = E_V * 230;
		}
		else{
		Moto_PWM->L = 0;
		Moto_PWM->R = 0;
		Moto_PWM->B = 0;
		}
}

int main(void)
{	
	struct CarSensor car_sensor;
	struct MotoPWM Moto_PWM;
  int E_V, car_V;
	int MotorFlag;
	char txt[20];
	int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;
//-----------------------系统初始化配置----------------------------
	HAL_Init();			  // 初始化HAL库
	SystemClock_Config(); // 设置时钟9倍频,72M
	delay_init(72);		  // 初始化延时函数
	JTAG_Set(SWD_ENABLE); // 打开SWD接口 可以利用主板的SWD接口调试
//-----------------------------------------------------------------
	LED_Init();			// LED初始化
	OLED_Init();		// OLED初始化   
	OLED_Show_LQLogo(); // 显示LOGO
	sensor_init();
	MotorInit();
	Encoder_Init_TIM2();   //编码器占用定时器2,3,4，硬件AB解码读取
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	
	delay_ms(500);		// 延时等待
	OLED_CLS();			// 清屏	
	
  OLED_P6x8Str(0,0,"trace test");
	while(1)
	{
		MotorFlag=1;
		E_V = (car_sensor.a * 2 + car_sensor.b * 1) - (car_sensor.c * 1 + car_sensor.d * 2);
		if (car_sensor.a == 1 && car_sensor.b == 1 && car_sensor.c == 1 && car_sensor.d == 1)//all black
		{
		car_V = -800;
		}
		else
		{
		if (abs(E_V) > 2) 
		car_V = 600;//turn
		else 
		car_V = 1000;//straight
		}
		moto_PWM(&Moto_PWM, car_V , E_V, MotorFlag);
		Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
		Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
		Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));
		MotorCtrl3w(Moto_PWM.B, Moto_PWM.L, Moto_PWM.R);
		
		
		ECPULSE1=Read_Encoder(2);
		sprintf(txt,"E1:%04d ",ECPULSE1);
	    OLED_P8x16Str(0,2,txt);	
		ECPULSE2=Read_Encoder(3);
		sprintf(txt,"E2:%04d ",ECPULSE2);
	    OLED_P8x16Str(0,4,txt);	
		ECPULSE3=Read_Encoder(4);
		sprintf(txt,"E3:%04d ",ECPULSE3);
	    OLED_P8x16Str(0,6,txt);
		sprintf(txt, "%d %d %d %d", car_sensor.a, car_sensor.b, car_sensor.c, car_sensor.d);
		OLED_P6x8Str(0, 10, txt);
		
		
		
	}
}


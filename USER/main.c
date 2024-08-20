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

struct CarSensor
{
	int a,b,c,d;
};


void Car_sensor(struct CarSensor *car_sensor) 
{
    car_sensor->a = Read_sensor(1);
		car_sensor->b = Read_sensor(2);
    car_sensor->c = Read_sensor(3);
    car_sensor->d = Read_sensor(4);
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
//-----------------------ϵͳ��ʼ������----------------------------
	HAL_Init();			  // ��ʼ��HAL��
	SystemClock_Config(); // ����ʱ��9��Ƶ,72M
	delay_init(72);		  // ��ʼ����ʱ����
	JTAG_Set(SWD_ENABLE); // ��SWD�ӿ� �������������SWD�ӿڵ���
//-----------------------------------------------------------------
	LED_Init();			// LED��ʼ��
	OLED_Init();		// OLED��ʼ��   
	OLED_Show_LQLogo(); // ��ʾLOGO
	sensor_init();
	MotorInit();
	Encoder_Init_TIM2();   //������ռ�ö�ʱ��2,3,4��Ӳ��AB�����ȡ
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	
	delay_ms(500);		// ��ʱ�ȴ�
	OLED_CLS();			// ����	
    
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
	}
}


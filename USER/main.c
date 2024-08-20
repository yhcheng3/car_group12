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
@�����汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

// ��������ͷ�ļ�
#include "include.h"
#define PERIOD 50

struct CarSensor
{
	uint8_t a,b,c,d;
};

uint8_t invert(uint8_t val) // Use if background is white
{
	if (val == 0) val = 1;
	else val = 0;
	return val;
}

void read_sensor(struct CarSensor *carSensor) 
{
    carSensor->a = invert(Read_sensor(sensor1));
		carSensor->b = invert(Read_sensor(sensor2));
    carSensor->c = invert(Read_sensor(sensor3));
    carSensor->d = invert(Read_sensor(sensor4));
}


struct MotoPWM
{
	int16_t B,L,R;
};

void set_PWM(struct MotoPWM  *motoPWM, int car_V, int E_V, int motorFlag) {
    if (motorFlag)
		{
    motoPWM->L = car_V + E_V * 150;
		motoPWM->R = -car_V + E_V * 150;
			if ((E_V * 300) < 800 && E_V > 0) {
				motoPWM->B = 800;
			} else if ((E_V * 300) > -800 && E_V < 0) {
				motoPWM->B = -800;
			} else {
				motoPWM->B = E_V * 300;
			}
		}
		else{
		motoPWM->L = 0;
		motoPWM->R = 0;
		motoPWM->B = 0;
		}

}

void circle_PWM(struct MotoPWM *motoPWM, int *rotate_cnt, int *rotate_dir, int threshold) {
	if (abs(*rotate_cnt * PERIOD) < threshold) { // Around 135 deg
		if (*rotate_dir == 0) { // Right
			motoPWM->L = 800;
			motoPWM->R = 800;
			motoPWM->B = 800;
			(*rotate_cnt)++;
		}
		else { // Left
			motoPWM->L = -800;
			motoPWM->R = -800;
			motoPWM->B = -800;
			(*rotate_cnt)--;
		}
	}
	else if (abs(*rotate_cnt * PERIOD) >= threshold) {
		if (*rotate_dir == 1) { // Right
			motoPWM->L = threshold * 3;
			motoPWM->R = threshold * 3;
			motoPWM->B = threshold * 3;
			*rotate_dir = 0;
			(*rotate_cnt) = 0;
			MotorCtrl3w(motoPWM->B, motoPWM->L, motoPWM->R);
			delay_ms(PERIOD);
			MotorCtrl3w(motoPWM->B, motoPWM->L, motoPWM->R);
			delay_ms(PERIOD);
		}
		else { // Left
			motoPWM->L = -3500;
			motoPWM->R = -3500;
			motoPWM->B = -3500;
			*rotate_dir = 1;
			(*rotate_cnt) = 0;
			MotorCtrl3w(motoPWM->B, motoPWM->L, motoPWM->R);
			delay_ms(PERIOD);
			MotorCtrl3w(motoPWM->B, motoPWM->L, motoPWM->R);
			delay_ms(PERIOD);
		}
	}
}

//void set_PWM(struct MotoPWM *motoPWM, struct CarSensor *carSensor, int prev_dir, int motorFlag) 
//{
//	if (!motorFlag) 
//	{
//		motoPWM->L = 0;
//		motoPWM->R = 0;
//		motoPWM->B = 0;
//	} 
//	else 
//	{	
//		if (carSensor->a == 1 && carSensor->b == 1 && carSensor->c == 1 && carSensor->d == 1) //all black
//		{
//			motoPWM->L = -800;
//			motoPWM->R = 800;
//			motoPWM->B = 0;
//		}
//	}
//}

int main(void)
{	
//------Basic Params------
	struct CarSensor carSensor;
	struct MotoPWM motoPWM;
  int E_V, car_V;
	int motorFlag;
	int prev_E_V;
	char txt[20];
	int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;
//------Reverse Params--------
	int reverse_cnt = 0, anti_reverse_cnt = 0;
	int rotate_cnt = 0, rotate_dir = 0;

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
		read_sensor(&carSensor);
		motorFlag = 1;
		
		E_V = (carSensor.a * 2 + carSensor.b * 1) - (carSensor.c * 1 + carSensor.d * 2);
		if (carSensor.a == 1 && carSensor.b == 1 && carSensor.c == 1 && carSensor.d == 1) { //|| 
				// (carSensor.a == 0 && carSensor.b == 0 && carSensor.c == 0 && carSensor.d == 0))//all black
		
//			if (abs(prev_E_V) >= 1) {
//				car_V = 900;
//				E_V = prev_E_V;
//			}
			if (reverse_cnt < 100) {
				circle_PWM(&motoPWM, &rotate_cnt, &rotate_dir, 700);
				reverse_cnt += 2;
			} else {
				car_V = -800;
				set_PWM(&motoPWM, car_V , E_V, 1);
				reverse_cnt = 100;
			}
		}
		else
		{
			if (reverse_cnt > 0) anti_reverse_cnt++;
			if (anti_reverse_cnt >= 3) {
				reverse_cnt = 0;
				anti_reverse_cnt = 0;
			}
			rotate_cnt = 0;
			
			if (abs(E_V) > 2) 
			car_V = 800;//turn
			else 
			car_V = 2000;//straight
			prev_E_V = E_V;
			set_PWM(&motoPWM, car_V , E_V, 1);
		}
		motoPWM.L = ((motoPWM.L) < (-6000) ? (-6000) : ((motoPWM.L) > (6000) ? (6000) : (motoPWM.L)));
		motoPWM.R = ((motoPWM.R) < (-6000) ? (-6000) : ((motoPWM.R) > (6000) ? (6000) : (motoPWM.R)));
		motoPWM.B = ((motoPWM.B) < (-6000) ? (-6000) : ((motoPWM.B) > (6000) ? (6000) : (motoPWM.B)));
		
		ECPULSE1=Read_Encoder(2);
		//sprintf(txt,"B:%04d ",ECPULSE1);
		sprintf(txt,"L:%04d ",motoPWM.L);
	    OLED_P8x16Str(0,0,txt);	
		ECPULSE2=Read_Encoder(3);
		//sprintf(txt,"L:%04d ",ECPULSE2);
		sprintf(txt,"R:%04d ",motoPWM.R);
	    OLED_P8x16Str(0,2,txt);	
		ECPULSE3=Read_Encoder(4);
		//sprintf(txt,"R:%04d ",ECPULSE3);
		sprintf(txt,"B:%04d ",motoPWM.B);
	    OLED_P8x16Str(0,4,txt);
		sprintf(txt, "%d %d %d %d", carSensor.a, carSensor.b, carSensor.c, carSensor.d);
		OLED_P6x8Str(0,6, txt);
		
		//MotorCtrl3w(1000, 1000, 1000);
		MotorCtrl3w(motoPWM.B, motoPWM.L, motoPWM.R);
		delay_ms(PERIOD);
	}
}

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


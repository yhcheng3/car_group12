/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ���ƽ����ֳ�
����    д��chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 
�������¡�2022��3��10��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��Keil5.2�����ϰ汾
��Target �� STM32F103
��Crystal�� 8.000Mhz
��SYS PLL�� 72MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "balance.h"

extern short   gyro[3], accel[3], sensors;
extern float   Pitch, Roll, Yaw; 

u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=0;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Motor1,Motor2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 Distance;                               //���������
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //��ʱ�͵��εȱ���
float Acceleration_Z;                       //Z����ٶȼ�  
//float Balance_Kp=97 ,Balance_Kd=0.45,Velocity_Kp=3,Velocity_Ki=0.016;//PID����0
float Balance_Kp=105 ,Balance_Kd=0.45,Velocity_Kp=2.8,Velocity_Ki=0.015;//PID����0
//u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������

float Balance_zero =-9.8;

int Left1=0,Left2=0,Right2=0,Right1=0;

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All;
int Start_Flag=1;
float Turn_flag = 0;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void balance_car(void)
@����˵����ƽ�⹦��ʵ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��MPU6050 INT����5�����ж�	
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void balance_car(void)
{
	    LED_Ctrl(RVS); 
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ50ms�ľ�׼��ʱ
			 }				 
		 if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
			{
			  LQ_DMP_Read(); 
				Gyro_Turn=gyro[2];               //===����ת����ٶ�
				Acceleration_Z=accel[2];         //===����Z����ٶȼ�                                              
			}                                                                   //10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
    Encoder_Left = Read_Encoder(3);  //��ȡ��������ֵ 
		Encoder_Right = -Read_Encoder(4);  //��ȡ��������ֵ
    LQ_DMP_Read();                                             //===������̬
		if(Pitch>30)
		{
			Pitch=30;			
		}
		if(Pitch<-10)
		{
			Pitch=-10;			
		}				
		Gyro_Turn=gyro[2];               //===����ת����ٶ�
		Acceleration_Z=accel[2];         //===����Z����ٶȼ�                                              
	 // Key(); 
		Infra_car();
		Balance_Pwm =Balance(-Pitch,-gyro[0]);                   //===ƽ��PID����	
	  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===�ٶȻ�PID����		 								 
	  Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===ת��PID����     
 	  Motor1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                            //===�������ֵ������PWM
 	  Motor2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===�������ֵ������PWM
   	Xianfu_Pwm();    
			
		if(Motor1>0)		Motor1 += 350;
		else if(Motor1<0)		Motor1 -= 350;	
		if(Motor2>0)		Motor2 += 350;
		else if(Motor2<0)		Motor2 -= 350;	
		
		if(Start_Flag==1)
		{
			Motor1 =Motor1*1.1;
			MotorCtrl(Motor1,Motor2);
		}
		else
		{
			MotorCtrl(0,0);	
		}
}
	
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int Balance(float Angle,float Gyro)
@����˵����ֱ��PD����
@����˵�����Ƕȡ����ٶ�
@�������أ�ֱ������PWM
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int Balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Balance_zero;                       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
float Movement =30;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int velocity(int encoder_left,int encoder_right)
@����˵�����ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
@����˵�������ֱ����������ֱ�����
@�������أ��ٶȿ���PWM
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral,Target_Velocixty;
//	  //=============ң��ǰ�����˲���=======================// 
//	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //����������ģʽ,�Զ��������ģʽ
//    else 	                         Target_Velocity=110;                 
//		if(1==Flag_Qian)    	Movement=-Target_Velocity/Flag_sudu;	         //===ǰ����־λ��1 
//		else if(1==Flag_Hou)	Movement=Target_Velocity/Flag_sudu;         //===���˱�־λ��1
//	  else  Movement=0;	
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //���ϱ�־λ��1�ҷ�ң��ת���ʱ�򣬽������ģʽ
//	  Movement=Target_Velocity/Flag_sudu;
   //=============�ٶ�PI������=======================//	
		Encoder_Least =(encoder_left+encoder_right)-Movement;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		//Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral+=(Encoder-Movement);                   //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>10000)  	Encoder_Integral =10000;             //===�����޷�
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===�����޷�	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
		if(Start_Flag==0)   Encoder_Integral=0;      //===����رպ��������
	  return Velocity;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int turn(int encoder_left,int encoder_right,float gyro)//ת�����
@����˵����ת�����  �޸�ת���ٶȣ����޸�Turn_Amplitude����
@����˵�������ֱ����������ֱ�������Z��������
@�������أ�ת�����PWM
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=88/Flag_sudu,Kp=25,Kd=0.3;     
//	  //=============ң��������ת����=======================//
//  	if(1==Left_Flag||1==Right_Flag)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
//		{
//			if(++Turn_Count==1)
//			Encoder_temp=myabs(encoder_left+encoder_right);
//			Turn_Convert=50/Encoder_temp;
//			if(Turn_Convert<0.6)Turn_Convert=0.6;
//			if(Turn_Convert>3)Turn_Convert=3;
//		}	
//	  else
//		{
//			Turn_Convert=0;
//			Turn_Count=0;
//			Encoder_temp=0;
//		}			
//		
//		
//		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
//		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
//		else Turn_Target=0;
	
//    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
//	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
//		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
//		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
  	//=============ת��PD������=======================//
		//Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
	  Turn=-Turn_flag*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
		return Turn;
}


/**************************************************************************
����ָ��
**************************************************************************/
void Key(void)
{	
static u8 flag=0;
//flag=click_N_Double(70);	
if(flag==1&&Start_Flag==0)Start_Flag=1,flag=0; //����һ��������־λ��ת
if(flag==1&&Start_Flag==1)Start_Flag=0,flag=0; //����һ��������־λ��ת
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Xianfu_Pwm(void)
@����˵��������PWM��ֵ
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
//		if(Flag_Qian==1)  Moto1+=DIFFERENCE;  //DIFFERENCE��һ������ƽ��С������ͻ�е��װ�����һ��������ֱ���������������С�����и��õ�һ���ԡ�
//	  if(Flag_Hou==1)   Moto2-=DIFFERENCE;
    if(Motor1<-Amplitude) Motor1=-Amplitude;	
		if(Motor1>Amplitude)  Motor1=Amplitude;	
	  if(Motor2<-Amplitude) Motor2=-Amplitude;	
		if(Motor2>Amplitude)  Motor2=Amplitude;		
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�int myabs(int a)
@����˵��������ֵ����
@����˵����int
@�������أ�unsigned int
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void OLED_show(void)
@����˵����OLED��ʾ����
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void OLED_show(void)
{
	char txt[30];
	sprintf(txt,"L1:%01d L2:%01d",Left1,Left2);
	OLED_P6x8Str(10 ,2,(u8*)txt);
	sprintf(txt,"R2:%01d R1:%01d",Right2,Right1);
	OLED_P6x8Str(10 ,3,(u8*)txt);
	sprintf(txt,"%05d %05d",Encoder_Left,Encoder_Right);
	OLED_P6x8Str(10 ,4,(u8*)txt);
	sprintf(txt,"%05d %05d",Motor1,Motor2);
	OLED_P6x8Str(10 ,5,(u8*)txt);
	sprintf(txt,"%.02f %06d",Pitch,gyro[0]);
	OLED_P6x8Str(10 ,6,(u8*)txt);

}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Infra_Init(void)
@����˵��������ѭ����ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Infra_Init(void)
{
	   GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();           //����GPIOAʱ��
		__HAL_RCC_GPIOB_CLK_ENABLE();           //����GPIOBʱ��

    GPIO_Initure.Pin=GPIO_PIN_2 | GPIO_PIN_3;  	//PA11  12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//����
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);    
	
		GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1;  //PB0  1  2
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      								//����
    GPIO_Initure.Pull=GPIO_PULLDOWN;        								//����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;                //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
}
int left = 0,right = 0;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Infra_car(void)
@����˵��������ѭ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Infra_car(void)
{
	Left1  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);  // A2
	Left2  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);	// A3
	Right2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0); 	// B0
	Right1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1); 	// B1
	
	if(Left2==0&&Right2==0)
	{
		left = 0;
		right=0;	
	}
	else if(Left2==1&&Right2==0)
	{
		left = 1;
		right=0;		
	}
	else if(Left2==0&&Right2==1)
	{
		left = 0;
		right=1;
	}
	
	if(left==0&&right==0)
	{
		Turn_flag=0;
	//	Movement = 50;
	}
	else if(left==1&&right==0)
	{
		Turn_flag += 1;
	//	Movement = 50;
	}
		else if(left==0&&right==1)
	{
		Turn_flag -= 1;
	//	Movement= 50;
	}
	if(Turn_flag>20)Turn_flag=20;
	else if(Turn_flag<-20)Turn_flag=-20;
}
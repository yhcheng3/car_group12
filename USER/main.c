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

//-----Global Variables------
photoele_t photoele; // ��紫����
encoder_t enc;
controller_t ctrl; 
pid_param_t pid;
// int dis = 0; TODO: change to global variable? Timing clashes with ultrasonic_avoid()?

int init_flag = 0;

int main(void)
{
	//-----------------------ϵͳ��ʼ������----------------------------
	HAL_Init();			  // ��ʼ��HAL��
	SystemClock_Config(); // ����ʱ��9��Ƶ,72M
	delay_init(72);		  // ��ʼ����ʱ����
	JTAG_Set(SWD_ENABLE); // ��SWD�ӿ� �������������SWD�ӿڵ���
	//-----------------------------------------------------------------
	LED_Init();						//��ʼ��LED	
	KEY_Init();						//��ʼ������
	OLED_Init();    				//OLED��ʼ��
	OLED_Show_LQLogo(); // ��ʾLOGO
	delay_ms(500);		// ��ʱ�ȴ�
	OLED_CLS();						//����
	// uart_init(USART_2,115200);		//��ʼ������
	uart_init(USART_3,115200);		//��ʼ������
	Encoder_Init_TIM2();			//��������ʼ��
	Encoder_Init_TIM3();			//��������ʼ��
	Encoder_Init_TIM4();			//��������ʼ��
	MotorInit();					//�����ʼ��
	Ultrasonic_Init();
//===============================================================
	//Car_main();			//���������������ο�����
	PidInit(&pid);
	init_ctrl(&ctrl);
	init_photoele(&photoele);
	init_enc(&enc);
	init_flag = 1; // ��ʼ���������stm32f1xx_it.c����read_enc(), control()
	
	while(1)
	{
		OLED_task();
		// delay_ms(PERIOD);
	}
}

void OLED_task(void) {
	char txt[40];
	sprintf(txt, "MtrL: %+05d   Path: %d", ctrl.L, ctrl.on_path);
	OLED_P6x8Str(0,0,txt);	

	sprintf(txt, "MtrR: %+05d ",ctrl.R);
	OLED_P6x8Str(0,1,txt);	

	sprintf(txt, "MtrB: %+05d ",ctrl.B);
	OLED_P6x8Str(0,2,txt);
	
	sprintf(txt, "EncL: %+05d", enc.L);
	OLED_P6x8Str(0,3, txt);
	
	sprintf(txt, "EncR: %+05d", enc.R);
	OLED_P6x8Str(0,4, txt);
	
	sprintf(txt, "EncB: %+05d", enc.B);
	OLED_P6x8Str(0,5, txt);

	sprintf(txt, "Pht: %d %d %d %d Dis %04d", photoele.a, photoele.b, photoele.c, photoele.d, Get_Distance());
	OLED_P6x8Str(0,6, txt);
	
	sprintf(txt, "Mde %d WkSt %d CrSt %d", ctrl.mode, ctrl.work_state, ctrl.car_state);
	OLED_P6x8Str(0,7,txt);
}

void control(void) {
	// ת��ģʽ
	if (Read_key(KEY1)) {
		ctrl.mode = 0;
	}
	if (Read_key(KEY2)) {
		ctrl.mode = 1;
	}
	if (Read_key(KEY3)) {
		ctrl.mode = 2;
	}
	
	if (ctrl.mode == 2) {
		// Receive motor torques directly from UART, pass to ctrl.B, etc.
		recv_torque(&ctrl);
		
		// Use if receiving image from camera
		// recv_img(&ctrl);
	}
	else
	{
		if (ctrl.work_state == 0) {
			set_control(&ctrl, &photoele);
		} else {
			ultrasonic_avoid(&ctrl, &photoele);
		}
	}
//	char txt[40];
//	sprintf(txt, "DL %+05d", ctrl.L);
//	OLED_P6x8Str(70,3, txt);
//	
//	sprintf(txt, "DR %+05d", ctrl.R);
//	OLED_P6x8Str(70,4, txt);
//	
//	sprintf(txt, "DB %+05d", ctrl.B);
//	OLED_P6x8Str(70,5, txt);
	
	// PID��������PID��ע�ͼ��ɣ�
//	ctrl.B = PidLocCtrl(&pid, (enc.B * ENC_FACTOR) - ctrl.B);
//	ctrl.L = PidLocCtrl(&pid, (enc.L * ENC_FACTOR) - ctrl.L);
//	ctrl.R = PidLocCtrl(&pid, (enc.R * ENC_FACTOR) - ctrl.R);
	
	// �޷�
	ctrl.B = constrain_float(ctrl.B, -MAX_VAL, MAX_VAL);
	ctrl.L = constrain_float(ctrl.L, -MAX_VAL, MAX_VAL);
	ctrl.R = constrain_float(ctrl.R, -MAX_VAL, MAX_VAL);
	
	MotorCtrl3w(ctrl.B, ctrl.L, ctrl.R);
}


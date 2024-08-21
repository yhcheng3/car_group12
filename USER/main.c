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

extern u8 ReadBuff[1024];		

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
	OLED_CLS();						//����
	uart_init(USART_2,115200);		//��ʼ������
	uart_init(USART_3,115200);		//��ʼ������
	Encoder_Init_TIM2();			//��������ʼ��
	Encoder_Init_TIM3();			//��������ʼ��
	Encoder_Init_TIM4();			//��������ʼ��
	MotorInit();					//�����ʼ��
	Ultrasonic_Init();
//===============================================================
	//Car_main();			//���������������ο�����
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


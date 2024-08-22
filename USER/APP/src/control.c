#include "include.h"

extern encoder_t enc;

// Not global variables, we want speed not space.
int dis_R = 0, dis_L = 0;
int read_cnt = 0; // No. of reads to be averaged

//------------��ʼ��--------------
void init_ctrl(controller_t *ctrl) {
	ctrl->B = 0;
	ctrl->L = 0;
	ctrl->R = 0;
	
	ctrl->rotate_cnt = 0;
	ctrl->rotate_dir = 0;
	ctrl->rotate_times = 0;
	
	ctrl->work_state = 0;
	ctrl->car_state = run;
	
	ctrl->on_path = 0;
	ctrl->mode = 0;
}

void init_photoele(photoele_t *photoele) {
	photoele->a = 0;
	photoele->b = 0;
	photoele->c = 0;
	photoele->d = 0;
}

void init_enc(encoder_t *enc) {
	enc->B = 0;
	enc->L = 0;
	enc->R = 0;
}


//------------ѭ��--------------
uint8_t invert(uint8_t val) // Use if background is white
{
	if (val == 0) val = 1;
	else val = 0;
	return val;
}

// TODO: move photoele calls to TIM Handler;
// 			 make photoele global variable in main, copy encoder
void read_sensor(photoele_t *photoele) 
{
    photoele->a = invert(Read_sensor(sensor1));
		photoele->b = invert(Read_sensor(sensor2));
    photoele->c = invert(Read_sensor(sensor3));
    photoele->d = invert(Read_sensor(sensor4));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void set_PWM(controller_t  *ctrl, int car_V, int E_V, int motor_flag)
@����˵��������ѭ��ʱ���á�����set_control����ĸ��������������ĵ�����룬������*ctrl��
@����˵����car_V: ��set_control��������ٶȡ���E_V: ��set_control�������ת�������ȡ���
@�������أ���
@�޸�ʱ�䣺2024/08/20
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void set_PWM(controller_t  *ctrl, int car_V, int E_V) {
	ctrl->L = car_V + E_V * E_FACTOR;
	ctrl->R = -car_V + E_V * E_FACTOR;
		if (((E_V * E_B_FACTOR) < MIN_R) && E_V > 0) {
			ctrl->B = MIN_R;
		} else if ((E_V * E_B_FACTOR) > -MIN_R && E_V < 0) {
			ctrl->B = -MIN_R;
		} else {
			ctrl->B = E_V * E_B_FACTOR;
		}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void circle_PWM(controller_t *ctrl, int threshold)
@����˵����Ѱ�ҹ켣ʱ���á�תȦ��
@����˵����ctrl->rotate_dir������Ѱ�ҷ���
@�������أ���
@�޸�ʱ�䣺2024/08/20
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void circle_PWM(controller_t *ctrl) {
	if (abs(ctrl->rotate_cnt * PERIOD) < CRCL_THRESH) { // Around 135 deg
		if ((ctrl->rotate_dir) == 0) { // Right
			ctrl->L = CRCL_FACTOR;
			ctrl->R = CRCL_FACTOR;
			ctrl->B = CRCL_FACTOR;
			(ctrl->rotate_cnt)++;
		}
		else { // Left
			ctrl->L = -CRCL_FACTOR;
			ctrl->R = -CRCL_FACTOR;
			ctrl->B = -CRCL_FACTOR;
			(ctrl->rotate_cnt)--;
		}
	}
	else if (abs(ctrl->rotate_cnt * PERIOD) >= CRCL_THRESH) {
		if ((ctrl->rotate_dir) == 1) { // Left => Right
			ctrl->L = CRCL_TWIST_FACTOR;
			ctrl->R = CRCL_TWIST_FACTOR;
			ctrl->B = CRCL_TWIST_FACTOR;
			(ctrl->rotate_dir) = 0;
			(ctrl->rotate_cnt) = 0;
			MotorCtrl3w(ctrl->B, ctrl->L, ctrl->R);
			delay_ms(PERIOD);
			MotorCtrl3w(ctrl->B, ctrl->L, ctrl->R);
			delay_ms(PERIOD);
			(ctrl->rotate_times)++;
		}
		else { // Right => Left
			ctrl->L = -(CRCL_TWIST_FACTOR);
			ctrl->R = -(CRCL_TWIST_FACTOR);
			ctrl->B = -(CRCL_TWIST_FACTOR);
			(ctrl->rotate_dir) = 1;
			(ctrl->rotate_cnt) = 0;
			MotorCtrl3w(ctrl->B, ctrl->L, ctrl->R);
			delay_ms(PERIOD);
			MotorCtrl3w(ctrl->B, ctrl->L, ctrl->R);
			delay_ms(PERIOD);
		}
	}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void set_control(controller_t *ctrl, photoele_t *photoele, int motor_flag)
@����˵�������ݹ�紫������ֵ��תȦ�����˻�ѭ��
@����˵����*photoele: ���ݹ�紫������ֵ���������ĵ�����룬������*ctrl
@�������أ���
@�޸�ʱ�䣺2024/08/20
@���÷�����
@��    ע��������ģ���⵽�ϰ���ʱ���л�������״̬��work_state = 1��
					 ģʽ0�£���תROT_THRESHȦ��δ�ҵ�·��ʱ������
					 ģʽ1�£���תROT_THRESHȦ��δ�ҵ�·��ʱ���л�������״̬��work_state = 1��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void set_control(controller_t *ctrl, photoele_t *photoele) 
{
	int E_V, car_V, dis;
	read_sensor(photoele);
	
	E_V = (photoele->a * 2 + photoele->b * 1) - (photoele->c * 1 + photoele->d * 2);
	
	//ȫ1����ⲻ���켣
	if (photoele->a == 1 && photoele->b == 1 && photoele->c == 1 && photoele->d == 1) { //|| 
	// (photoele->a == 0 && photoele->b == 0 && photoele->c == 0 && photoele->d == 0))//all black
		
		ctrl->on_path = 0; //���ڹ켣��
		
		// -----------------Ѱ�ҹ켣��תȦ�����ˣ�------------------
		if (ctrl->rotate_times < ROT_THRESH){
			// תȦ
			if ((ctrl->rotate_cnt) == 0) {
				MotorCtrl3w(0, 0, 0); // ��������ת״̬��ɷ��
				delay_ms(PERIOD);
			}
			circle_PWM(ctrl); 
			
		} else if (ctrl->rotate_times == ROT_THRESH){
			// תȦ����ʱ
			delay_ms(TWIST_DELAY); // Wait for twist to finish
			(ctrl->rotate_times)++;
			
			// ģʽ1�´���ģʽת����������
			if (ctrl->mode == 1) {
				ctrl->work_state = 1;
				(ctrl->rotate_cnt) = 0;
				(ctrl->rotate_times) = 0;
				return; // Possible bug
			}
			
		} else { //��ʱrotate_times > ROT_THRESH
			// ģʽ0�º���
			car_V = -CAR_REV;
			set_PWM(ctrl, car_V , E_V);
		}
	}
	
	//����ȫ1����⵽�켣
	else
	{
		ctrl->on_path = 1;

		// --------------------����ѭ��-----------------------
		
		// E_V < 0��ζ�켣����ߣ���켣��ʧʱ������������ߣ�rotate_dir 0�� 1��
		// rotate_dir��circle_PWM����
		ctrl->rotate_dir = (E_V < 0);  
		
		//�һع켣ʱ��Ӧ������Ѱ����ز���
		(ctrl->rotate_cnt) = 0;
		(ctrl->rotate_times) = 0;
		
		if (abs(E_V) > 2) car_V = CAR_TURN; //turn
		else car_V = CAR_FWD; //straight
		
		set_PWM(ctrl, car_V , E_V);
	}
	
	// �޷�
	ctrl->L = ((ctrl->L) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->L) > (MAX_VAL) ? (MAX_VAL) : (ctrl->L)));
	ctrl->R = ((ctrl->R) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->R) > (MAX_VAL) ? (MAX_VAL) : (ctrl->R)));
	ctrl->B = ((ctrl->B) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->B) > (MAX_VAL) ? (MAX_VAL) : (ctrl->B)));

	// Change on_path before switching modes
	// �����ϰ�������ģʽת��
	dis = Get_Distance();
	if (dis < SWITCH_THRESH && dis != 1) {
		ctrl->work_state = 1;
	}
}

//------------����--------------
void read_enc(void)
{
		enc.B = Read_Encoder(2);
		enc.L = Read_Encoder(4);
		enc.R = Read_Encoder(3);
}

void car_move(controller_t *ctrl, MoveDir move)//�����ϰ���ʱ���˶���ʽ
{
	switch(move)
	{
		case forward:
			ctrl->B = 0;
			ctrl->L = FWD_FACTOR;
		  ctrl->R = -FWD_FACTOR;
			break;
		
		case turn_right: // 90 deg
			MotorCtrl3w(ROT_FACTOR, ROT_FACTOR, ROT_FACTOR);
			delay_ms(PERIOD);
			MotorCtrl3w(ROT_FACTOR, ROT_FACTOR, ROT_FACTOR);
			delay_ms(PERIOD);
			ctrl->B = ROT_FACTOR;
			ctrl->L = ROT_FACTOR;
		  ctrl->R = ROT_FACTOR;
			break;
		
		case turn_big_right:
			MotorCtrl3w(2*ROT_FACTOR, 2*ROT_FACTOR, 2*ROT_FACTOR);
			delay_ms(PERIOD);
			MotorCtrl3w(2*ROT_FACTOR, 2*ROT_FACTOR, 2*ROT_FACTOR);
			delay_ms(PERIOD);
			ctrl->B = 2*ROT_FACTOR;
			ctrl->L = 2*ROT_FACTOR;
		  ctrl->R = 2*ROT_FACTOR;
			break;
		
		case turn_big_left:
			MotorCtrl3w(-(2*ROT_FACTOR), -(2*ROT_FACTOR), -(2*ROT_FACTOR));
			delay_ms(PERIOD);
			MotorCtrl3w(-(2*ROT_FACTOR), -(2*ROT_FACTOR), -(2*ROT_FACTOR));
			delay_ms(PERIOD);
			ctrl->B = -(2*ROT_FACTOR);
			ctrl->L = -(2*ROT_FACTOR);
		  ctrl->R = -(2*ROT_FACTOR);
			break;
		
		case stop:
			ctrl->B = 0;
			ctrl->L = 0;
		  ctrl->R = 0;
			break;
	}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void ultrasonic_avoid(controller_t *ctrl, photoele_t *photoele)
@����˵�������ݳ�������С��״̬���б���
@����˵����*photoele: ���ݹ�紫������ֵ���������ĵ�����룬������*ctrl
@�������أ���
@�޸�ʱ�䣺2024/08/22
@���÷�����
@��    ע������on_path�ж��Ƿ�ӷ�·�췵��·�졣���ǣ�����״̬ת����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ultrasonic_avoid(controller_t *ctrl, photoele_t *photoele){
	int dis = 0; 
	
	// ��ȡ�����Ϣ
	// Not in stm32f1xx_it.c, as must be right before below condition check.
	read_sensor(photoele);
	
	// ��·����
	if (!(photoele->a == 1 && photoele->b == 1 && 
				photoele->c == 1 && photoele->d == 1)) 
	{
		// �ӷ�·�췵��·�죬����״̬ת��
		if (ctrl->on_path == 0) { 
			// Possible bug for on_path; turn OFF optimisation
			ctrl->work_state = 0; //ѭ��ģʽ
			ctrl->on_path = 1;    //����·��
			ctrl->car_state = run;
			return;		
		}
		// ����������״̬
		ctrl->on_path = 1;
	} else {
		ctrl->on_path = 0;
	}
	
	dis = Get_Distance();
	
	switch (ctrl->car_state)
	{
		// ����ǰ��
		case run:
			if (dis >= AVOID_THRESH || dis == 1){
				car_move(ctrl, forward);
			} else {
				// �����ϰ�
				car_move(ctrl, stop);
				ctrl->car_state = find_R;
				
				dis_L = 0; // ͳһ����
				dis_R = 0;
			}
			break;
		
		// ��ת
		case find_R:
			car_move(ctrl, turn_right);
			ctrl->car_state = delay_R;
			break;
		
		// �ҷ����
		case delay_R:
			car_move(ctrl, stop);	
			if (enc.L == 0 && enc.R == 0 && enc.B == 0)
			{
				if (read_cnt++ < 5)
				{
					dis_R += dis;
				}
				else {
					// ��5��ȡƽ��
					dis_R = dis_R / 5;
					read_cnt = 0;
					
					// �ҷ����빻Զ��ֱ��ǰ��
					if (dis_R > 60 || dis_R == 1)
					{
						ctrl->car_state = run;
					}
					else {
					// ����Զ
						ctrl->car_state = find_L;
					}
				}
			}
			break;
		
		// ��ת
		case find_L:
			car_move(ctrl, turn_big_left);
			ctrl->car_state = delay_L;
			break;
		
		// �ҷ����
		case delay_L:
			car_move(ctrl, stop);
			if (enc.L == 0 && enc.R == 0 && enc.B == 0)
				{
					if (read_cnt++ < 5)
					{
						dis_L += dis;
					}
					else {
						read_cnt = 0;
						dis_L = dis_L / 5;
						
						if (dis_L > 60 || dis_R == 1)
						{
							ctrl->car_state = run;
						}
						else {
							ctrl->car_state = compare_RL;
						}
					}
				}
			break;
				
		// �Ƚ����Ҿ���
		case compare_RL:
			if ((dis_L > dis_R || dis_L == 1) && dis_R != 1)
			{
				car_move(ctrl, stop);
				ctrl->car_state = run; // Continue left
			}
			else {
				car_move(ctrl, turn_big_right);
				ctrl->car_state = run; // Turn right
			}
			break;
	}
}

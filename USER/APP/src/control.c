#include "include.h"

void init_ctrl(controller_t *ctrl) {
	ctrl->rotate_cnt = 0;
	ctrl->rotate_dir = 0;
	ctrl->rotate_times = 0;
	ctrl->work_state = 0;
	ctrl->car_state = run;
}

uint8_t invert(uint8_t val) // Use if background is white
{
	if (val == 0) val = 1;
	else val = 0;
	return val;
}

void read_sensor(photoele_t *photoele) 
{
    photoele->a = invert(Read_sensor(sensor1));
		photoele->b = invert(Read_sensor(sensor2));
    photoele->c = invert(Read_sensor(sensor3));
    photoele->d = invert(Read_sensor(sensor4));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void set_PWM(controller_t  *ctrl, int car_V, int E_V, int motor_flag)
@����˵��������set_control����ĸ��������������ĵ�����룬������*ctrl
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
@����˵����תȦѰ�ҹ켣
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
@����˵�������ݹ�紫������ֵ��
@����˵����*photoele: ���ݹ�紫������ֵ���������ĵ�����룬������*ctrl
@�������أ���
@�޸�ʱ�䣺2024/08/20
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void set_control(controller_t *ctrl, photoele_t *photoele) 
{
	int E_V, car_V;
	read_sensor(photoele);
	
	E_V = (photoele->a * 2 + photoele->b * 1) - (photoele->c * 1 + photoele->d * 2);
	
	if (photoele->a == 1 && photoele->b == 1 && photoele->c == 1 && photoele->d == 1) { //|| 
	// (photoele->a == 0 && photoele->b == 0 && photoele->c == 0 && photoele->d == 0))//all black
		// ����תȦ��reverse_cnt������һ����ֵ�ٺ���
		if (ctrl->rotate_times < ROT_THRESH){
			// תȦ
			circle_PWM(ctrl);
		} else if (ctrl->rotate_times == ROT_THRESH){
			// תȦ����ʱ
			delay_ms(TWIST_DELAY); // Wait for twist to finish
			(ctrl->rotate_times)++;
		} else {
			// ����
			car_V = -CAR_REV;
			set_PWM(ctrl, car_V , E_V);
		}
	}
	else
	{
		// ����ѭ��
		// E_V < 0��ζ�켣����ߣ���켣��ʧʱ������������ߣ�rotate_dir0�� 1��
		ctrl->rotate_dir = (E_V < 0);  
		
		//������ת��ز���
		(ctrl->rotate_cnt) = 0;
		(ctrl->rotate_times) = 0;
		
		if (abs(E_V) > 2) 
		car_V = CAR_TURN; //turn
		else 
		car_V = CAR_FWD; //straight
		set_PWM(ctrl, car_V , E_V);
	}
	ctrl->L = ((ctrl->L) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->L) > (MAX_VAL) ? (MAX_VAL) : (ctrl->L)));
	ctrl->R = ((ctrl->R) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->R) > (MAX_VAL) ? (MAX_VAL) : (ctrl->R)));
	ctrl->B = ((ctrl->B) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->B) > (MAX_VAL) ? (MAX_VAL) : (ctrl->B)));
}

//-------------------Ultrasonic----------------------
//void read_enc(encoder_t *enc)
//{
//		enc->B = Read_Encoder(2);
//		enc->L = Read_Encoder(4);
//		enc->R = Read_Encoder(3);
//}
//	
//void car_move(controller_t *ctrl, MoveDir move)//�����ϰ���ʱ���˶���ʽ
//{
//	switch(move)
//	{
//		// TODO: change to ctrl->L allocation, install of MotorCtrl func call
//		case forward:
//			MotorCtrl3w(0, FWD_FACTOR, -FWD_FACTOR);
//		
//		case turn_right:
//			MotorCtrl3w(ROT_FACTOR, ROT_FACTOR, ROT_FACTOR); //around 90 deg
//			delay_ms(100);
//		
//		case turn_big_right:
//			MotorCtrl3w((2*ROT_FACTOR), (2*ROT_FACTOR), (2*ROT_FACTOR)); //around 90 deg
//			delay_ms(100);
//		
//		case turn_big_left:
//			MotorCtrl3w(-(2*ROT_FACTOR), -(2*ROT_FACTOR), -(2*ROT_FACTOR)); //around 90 deg
//			delay_ms(100);
//		
//		case stop:
//			MotorCtrl3w(0, 0, 0);
//	}
//}

//void ultrasonic_avoid(controller_t *ctrl, encoder_t *encoder){
//	int dis_R = 0, dis_L = 0;
//	int read_cnt = 0; // No. of reads to be averaged
//	
//	int dis = Get_Distance();
//	
//	read_enc(encoder);
//	switch (ctrl->car_state)
//	{
//		case idle: 
//			// Unused
//			car_move(ctrl, stop);
//			break;
//		
//		case run:
//			if (dis > 25 || dis == 1){
//				car_move(ctrl, forward);
//			}
//			else {
//				car_move(ctrl, stop);
//				ctrl->car_state = find_R;
//			}
//			break;

//		case find_R: // turn right & detect distance
//			car_move(ctrl, turn_right);
//			ctrl->car_state = delay_R;
//			break;
//		
//		case delay_R:
//				if (encoder->L == 0 && encoder->R == 0 && encoder->B == 0)
//				{
//					if (read_cnt++ < 5)
//					{
//						dis_R += dis;
//					}
//					else {
//						dis_R = dis_R / 5;
//						read_cnt = 0;
//						if (dis_R > 60 || dis_R == 1)
//						{
//							ctrl->car_state = run;
//							dis_R = 0;
//						}
//						else {
//						ctrl->car_state = find_L;
//						}
//					}
//				}
//				break;
//				
//		case find_L:
//			car_move(ctrl, turn_big_left);
//			ctrl->car_state = delay_L;
//			break;
//		
//		case delay_L:
//			if (encoder->L == 0 && encoder->R == 0 && encoder->B == 0)
//				{
//					if (read_cnt++ < 5)
//					{
//						dis_L += dis;
//					}
//					else {
//						dis_L = dis_L / 5;
//						read_cnt = 0;
//						if (dis_L > 60 || dis_R == 1)
//						{
//							ctrl->car_state = run;
//							dis_L = 0;
//						}
//						else {
//							ctrl->car_state = compare_RL;
//						}
//					}
//				}
//			break;
//		case compare_RL:
//			if ((dis_L > dis_R || dis_L == 1) && dis_R != 1)
//			{
//				ctrl->car_state = run; // Continue left
//			}
//			else {
//				car_move(ctrl, turn_big_right);
//				ctrl->car_state = run; // Turn right
//			}
//			dis_L = 0;
//			dis_R = 0;
//		break;
//	}
//}

#include "include.h"

void init_ctrl(controller_t *ctrl) {
  ctrl->reverse_cnt = 0;
	ctrl->anti_reverse_cnt = 0;
	
	ctrl->rotate_cnt = 0;
	ctrl->rotate_dir = 0;
}

uint8_t invert(uint8_t val) // Use if background is white
{
	if (val == 0) val = 1;
	else val = 0;
	return val;
}

void read_sensor(car_sensor_t *carSensor) 
{
    carSensor->a = invert(Read_sensor(sensor1));
		carSensor->b = invert(Read_sensor(sensor2));
    carSensor->c = invert(Read_sensor(sensor3));
    carSensor->d = invert(Read_sensor(sensor4));
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
void set_PWM(controller_t  *ctrl, int car_V, int E_V, int motor_flag) {
    if (motor_flag)
		{
    ctrl->L = car_V + E_V * 150;
		ctrl->R = -car_V + E_V * 150;
			if ((E_V * 300) < 800 && E_V > 0) {
				ctrl->B = 800;
			} else if ((E_V * 300) > -800 && E_V < 0) {
				ctrl->B = -800;
			} else {
				ctrl->B = E_V * 300;
			}
		}
		else{
		ctrl->L = 0;
		ctrl->R = 0;
		ctrl->B = 0;
		}

}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void circle_PWM(controller_t *ctrl, int threshold)
@����˵����תȦѰ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2024/08/20
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void circle_PWM(controller_t *ctrl, int threshold) {
	if (abs(ctrl->rotate_cnt * PERIOD) < threshold) { // Around 135 deg
		if ((ctrl->rotate_dir) == 0) { // Right
			ctrl->L = 800;
			ctrl->R = 800;
			ctrl->B = 800;
			(ctrl->rotate_cnt)++;
		}
		else { // Left
			ctrl->L = -800;
			ctrl->R = -800;
			ctrl->B = -800;
			(ctrl->rotate_cnt)--;
		}
	}
	else if (abs(ctrl->rotate_cnt * PERIOD) >= threshold) {
		if ((ctrl->rotate_dir) == 1) { // Right
			ctrl->L = threshold * 3;
			ctrl->R = threshold * 3;
			ctrl->B = threshold * 3;
			(ctrl->rotate_dir) = 0;
			(ctrl->rotate_cnt) = 0;
			MotorCtrl3w(ctrl->B, ctrl->L, ctrl->R);
			delay_ms(PERIOD);
			MotorCtrl3w(ctrl->B, ctrl->L, ctrl->R);
			delay_ms(PERIOD);
		}
		else { // Left
			ctrl->L = -3500;
			ctrl->R = -3500;
			ctrl->B = -3500;
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
@�������ƣ�void set_control(controller_t *ctrl, car_sensor_t *carSensor, int motor_flag)
@����˵�������ݹ�紫������ֵ��
@����˵����*carSensor: ���ݹ�紫������ֵ���������ĵ�����룬������*ctrl
@�������أ���
@�޸�ʱ�䣺2024/08/20
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void set_control(controller_t *ctrl, car_sensor_t *carSensor, int motor_flag) 
{
	int E_V, car_V;
	read_sensor(carSensor);
	E_V = (carSensor->a * 2 + carSensor->b * 1) - (carSensor->c * 1 + carSensor->d * 2);
	if (carSensor->a == 1 && carSensor->b == 1 && carSensor->c == 1 && carSensor->d == 1) { //|| 
			// (carSensor->a == 0 && carSensor->b == 0 && carSensor->c == 0 && carSensor->d == 0))//all black
	//			}
		if ((ctrl->reverse_cnt) < 100) {
			circle_PWM(ctrl, 700);
			(ctrl->reverse_cnt) += 2;
		} else {
			car_V = -800;
			set_PWM(ctrl, car_V , E_V, 1);
			(ctrl->reverse_cnt) = 100;
		}
	}
	else
	{
		if ((ctrl->reverse_cnt) > 0) (ctrl->anti_reverse_cnt)++;
		if ((ctrl->anti_reverse_cnt) >= 3) {
			(ctrl->reverse_cnt) = 0;
			(ctrl->anti_reverse_cnt) = 0;
		}
		(ctrl->rotate_cnt) = 0;
		
		if (abs(E_V) > 2) 
		car_V = 800;//turn
		else 
		car_V = 2000;//straight
		set_PWM(ctrl, car_V , E_V, 1);
	}
	ctrl->L = ((ctrl->L) < (-6000) ? (-6000) : ((ctrl->L) > (6000) ? (6000) : (ctrl->L)));
	ctrl->R = ((ctrl->R) < (-6000) ? (-6000) : ((ctrl->R) > (6000) ? (6000) : (ctrl->R)));
	ctrl->B = ((ctrl->B) < (-6000) ? (-6000) : ((ctrl->B) > (6000) ? (6000) : (ctrl->B)));
}

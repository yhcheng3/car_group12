#include "include.h"

void init_ctrl(controller_t *ctrl) {
	ctrl->rotate_cnt = 0;
	ctrl->rotate_dir = 0;
	ctrl->rotate_times = 0;
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
@函数名称：void set_PWM(controller_t  *ctrl, int car_V, int E_V, int motor_flag)
@功能说明：根据set_control算出的各参数，算出合理的电机输入，并赋给*ctrl
@参数说明：car_V: 由set_control算出，“速度”；E_V: 由set_control算出，“转向灵敏度”；
@函数返回：无
@修改时间：2024/08/20
@调用方法：
@备    注：
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
@函数名称：void circle_PWM(controller_t *ctrl, int threshold)
@功能说明：转圈寻找轨迹
@参数说明：ctrl->rotate_dir：优先寻找方向
@函数返回：无
@修改时间：2024/08/20
@调用方法：
@备    注：
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
@函数名称：void set_control(controller_t *ctrl, car_sensor_t *carSensor, int motor_flag)
@功能说明：根据光电传感器的值，
@参数说明：*carSensor: 根据光电传感器的值，算出合理的电机输入，并赋给*ctrl
@函数返回：无
@修改时间：2024/08/20
@调用方法：
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void set_control(controller_t *ctrl, car_sensor_t *carSensor, int motor_flag) 
{
	if (!motor_flag)
	{
		ctrl->L = 0;
		ctrl->R = 0;
		ctrl->B = 0;
	}
	else
	{
		int E_V, car_V;
		read_sensor(carSensor);
		
		E_V = (carSensor->a * 2 + carSensor->b * 1) - (carSensor->c * 1 + carSensor->d * 2);
		
		if (carSensor->a == 1 && carSensor->b == 1 && carSensor->c == 1 && carSensor->d == 1) { //|| 
		// (carSensor->a == 0 && carSensor->b == 0 && carSensor->c == 0 && carSensor->d == 0))//all black
			// 优先转圈，reverse_cnt增加至一定阈值再后退
			if (ctrl->rotate_times < ROT_THRESH){
				// 转圈
				circle_PWM(ctrl);
			} else if (ctrl->rotate_times == ROT_THRESH){
				// 
				delay_ms(TWIST_DELAY); // Wait for twist to finish
				(ctrl->rotate_times)++;
			} else {
				// 后退
				car_V = -CAR_REV;
				set_PWM(ctrl, car_V , E_V);
			}
		}
		else
		{
			// 常规循迹
			// E_V < 0意味轨迹在左边，则轨迹消失时，该优先找左边；rotate_dir0右 1左
			ctrl->rotate_dir = (E_V < 0);  
			
			//清零旋转相关参数
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
}

/////////
void read_ENC_V(ENC_t *enc_v)
{
		enc_v->B = Read_Encoder(2);
		enc_v->L = Read_Encoder(3);
		enc_v->R = Read_Encoder(4);
};
	
void car_move(controller_t *ctrl, MotoPWM_t *motopwm, MoveDir move)//遇到障碍物时的运动方式
{
	ctrl->work_state=1;
	switch(move)
	{
		case forward:
			MotorCtrl3w(0, 800, -800);
		case turn_right:
			ctrl->rotate_dir=0;
			circle_PWM(ctrl, 450);//around 90 deg
		case turn_left:
			ctrl->rotate_dir=1;
			circle_PWM(ctrl, 450);//around 90 deg
		case stop:
			MotorCtrl3w(0, 0, 0);
	}
}


void ultrasonic_avoid(controller_t *ctrl, CarState car_state, MotoPWM_t *motopwm, int dis,ENC_t *enc_v){
	int dis_R=0, dis_L=0;
	int read_cnt=0;
	
	ctrl->work_state=1;
	ctrl->rotate_cnt=0;
	
	read_ENC_V(enc_v);
	switch (car_state)
	{
		case idle: 
			car_move(ctrl,motopwm, stop);
			break;
		case run:
			if (dis >25||dis==1){
				car_move(ctrl,motopwm, forward);
			}
			else{
				car_move(ctrl,motopwm, stop);
				car_state = barrier;
			}
			break;
		case barrier:
				car_state = find_R;
			break;
		case find_R://turn right & detect distance
			car_move(ctrl,motopwm, turn_right);
			car_state = delay_R;
			break;
		case delay_R:
				if (enc_v->L == 0 && enc_v->R == 0 && enc_v->B == 0)
				{
					if (read_cnt++ < 5)
					{
						dis_R += dis;
					}
					else{
						dis_R = dis_R / 5;
						read_cnt = 0;
						if (dis_R > 60 || dis_R == 1)
						{
							car_state = run;
							dis_R = 0;
						}
						else{
						car_state = find_L;
						}
					}
				}
				break;
		case find_L:
			car_move(ctrl,motopwm, turn_left);
			car_state = delay_L;
			break;
		case delay_L:
			if (enc_v->L == 0 && enc_v->R == 0 && enc_v->B == 0)
				{
					if (read_cnt++ < 5)
					{
						dis_L += dis;
					}
					else{
						dis_L = dis_L / 5;
						read_cnt = 0;
						if (dis_L > 60 || dis_R == 1)
						{
							car_state = run;
							dis_L = 0;
						}
						else{
						car_state = compare_RL;
						}
					}
				}
			break;
		case compare_RL:
			if ((dis_L > dis_R || dis_L == 1) && dis_R != 1)
				{
					car_state = run;// run stright in left direction
					dis_L = 0;
					dis_R = 0;
				}
			else{
				car_move(ctrl,motopwm, turn_right);
				delay_ms(PERIOD);
				car_move(ctrl,motopwm, turn_right);//turn 180 deg into right direction
				delay_ms(PERIOD);
				car_state = run;
			}
		break;
	}
}

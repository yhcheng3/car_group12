#include "include.h"

extern encoder_t enc;

// Not global variables, we want speed not space.
int dis_R = 0, dis_L = 0;
int read_cnt = 0; // No. of reads to be averaged

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
@函数名称：void set_PWM(controller_t  *ctrl, int car_V, int E_V, int motor_flag)
@功能说明：常规循迹时调用。根据set_control算出的各参数，算出合理的电机输入，并赋给*ctrl。
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
@功能说明：寻找轨迹时调用。转圈。
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
@函数名称：void set_control(controller_t *ctrl, photoele_t *photoele, int motor_flag)
@功能说明：根据光电传感器的值，转圈、后退或循迹
@参数说明：*photoele: 根据光电传感器的值，算出合理的电机输入，并赋给*ctrl
@函数返回：无
@修改时间：2024/08/20
@调用方法：
@备    注：超声波模块检测到障碍物时，切换至避障状态（work_state = 1）
					 模式0下，旋转ROT_THRESH圈仍未找到路径时，后退
					 模式1下，旋转ROT_THRESH圈仍未找到路径时，切换至避障状态（work_state = 1）
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void set_control(controller_t *ctrl, photoele_t *photoele) 
{
	int E_V, car_V, dis;
	read_sensor(photoele);
	
	E_V = (photoele->a * 2 + photoele->b * 1) - (photoele->c * 1 + photoele->d * 2);
	
	//全1：检测不到轨迹
	if (photoele->a == 1 && photoele->b == 1 && photoele->c == 1 && photoele->d == 1) { //|| 
	// (photoele->a == 0 && photoele->b == 0 && photoele->c == 0 && photoele->d == 0))//all black
		
		ctrl->on_path = 0; //不在轨迹上
		
		// -----------------寻找轨迹（转圈、后退）------------------
		if (ctrl->rotate_times < ROT_THRESH){
			// 转圈
			if ((ctrl->rotate_cnt) == 0) {
				MotorCtrl3w(0, 0, 0); // 刚跳进旋转状态，煞车
				delay_ms(PERIOD);
			}
			circle_PWM(ctrl); 
			
		} else if (ctrl->rotate_times == ROT_THRESH){
			// 转圈后延时
			delay_ms(TWIST_DELAY); // Wait for twist to finish
			(ctrl->rotate_times)++;
			
			// 模式1下触发模式转换，不后退
			if (ctrl->mode == 1) {
				ctrl->work_state = 1;
				(ctrl->rotate_cnt) = 0;
				(ctrl->rotate_times) = 0;
				return; // Possible bug
			}
			
		} else { //此时rotate_times > ROT_THRESH
			// 模式0下后退
			car_V = -CAR_REV;
			set_PWM(ctrl, car_V , E_V);
		}
	}
	
	//不是全1：检测到轨迹
	else
	{
		ctrl->on_path = 1;

		// --------------------常规循迹-----------------------
		
		// E_V < 0意味轨迹在左边，则轨迹消失时，该优先找左边；rotate_dir 0右 1左
		// rotate_dir由circle_PWM调用
		ctrl->rotate_dir = (E_V < 0);  
		
		//找回轨迹时，应当清零寻迹相关参数
		(ctrl->rotate_cnt) = 0;
		(ctrl->rotate_times) = 0;
		
		if (abs(E_V) > 2) car_V = CAR_TURN; //turn
		else car_V = CAR_FWD; //straight
		
		set_PWM(ctrl, car_V , E_V);
	}
	
	// 限幅
	ctrl->L = ((ctrl->L) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->L) > (MAX_VAL) ? (MAX_VAL) : (ctrl->L)));
	ctrl->R = ((ctrl->R) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->R) > (MAX_VAL) ? (MAX_VAL) : (ctrl->R)));
	ctrl->B = ((ctrl->B) < (-MAX_VAL) ? (-MAX_VAL) : ((ctrl->B) > (MAX_VAL) ? (MAX_VAL) : (ctrl->B)));

	// Change on_path before switching modes
	// 遇到障碍，触发模式转换
	//dis = Get_Distance();
	dis = get_distance_filtered();

	if (dis < SWITCH_THRESH && dis != 1) {
		ctrl->work_state = 1;
	}
}

//-------------------Ultrasonic----------------------
void read_enc(void)
{
		enc.B = Read_Encoder(2);
		enc.L = Read_Encoder(4);
		enc.R = Read_Encoder(3);
}

void car_move(controller_t *ctrl, MoveDir move)//遇到障碍物时的运动方式
{
	switch(move)
	{
		// TODO: possible bug - delay insufficient after jerk to right
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
@函数名称：void ultrasonic_avoid(controller_t *ctrl, photoele_t *photoele)
@功能说明：根据超声波
@参数说明：*photoele: 根据光电传感器的值，算出合理的电机输入，并赋给*ctrl
@函数返回：无
@修改时间：2024/08/20
@调用方法：
@备    注：超声波模块检测到障碍物时，切换至避障状态（work_state = 1）
					 模式0下，旋转ROT_THRESH圈仍未找到路径时，后退
					 模式1下，旋转ROT_THRESH圈仍未找到路径时，切换至避障状态（work_state = 1）
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ultrasonic_avoid(controller_t *ctrl, photoele_t *photoele){
	int dis = 0; 
	
	// 读取光电信息
	// Not in stm32f1xx_it.c, as must be right before below condition check.
	read_sensor(photoele);
	
	// 在路轨上
	if (!(photoele->a == 1 && photoele->b == 1 && 
				photoele->c == 1 && photoele->d == 1)) 
	{
		//从非路轨返回路轨，触发状态转换
		if (ctrl->on_path == 0) { 
			// Possible bug for on_path; turn OFF optimisation
			ctrl->work_state = 0; //循迹模式
			ctrl->on_path = 1;    //返回路轨
			ctrl->car_state = run;
			return;		
		}
		//刚跳到避障状态
		ctrl->on_path = 1;
	} else {
		ctrl->on_path = 0;
	}
	
	dis = Get_Distance();
	
	switch (ctrl->car_state)
	{
		case run:
			if (dis >= AVOID_THRESH || dis == 1){
				car_move(ctrl, forward);
			} else {
				car_move(ctrl, stop);
				ctrl->car_state = find_R;
				dis_L = 0; // 统一重设
				dis_R = 0;
			}
			break;

		case find_R: // turn right & detect distance
			car_move(ctrl, turn_right);
			ctrl->car_state = delay_R;
			break;
		
		case delay_R:
			car_move(ctrl, stop);	
			if (enc.L == 0 && enc.R == 0 && enc.B == 0)
			{
				dis_R = get_distance_filtered();
				if (dis_R > 60 || dis_R == 1){
					ctrl->car_state = run;
				}
				else {
					ctrl->car_state = find_L;
				}
			}
			break;
				
		case find_L:
			car_move(ctrl, turn_big_left);
			ctrl->car_state = delay_L;
			break;
		
		case delay_L:
			car_move(ctrl, stop);
			if (enc.L == 0 && enc.R == 0 && enc.B == 0){
				dis_L = get_distance_filtered();
				if (dis_L > 60 || dis_L == 1){
					ctrl->car_state = run;
				}
				else {
					ctrl->car_state = compare_RL;
				}
			}
			break;
				
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


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int get_distance_filtered(void)
@功能说明：对超声波传感器测距进行去噪
@参数说明：
@函数返回：int distance_filtered 去噪后的距离值
@修改时间：2024/08/22
@调用方法：
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

int get_distance_filtered(void){
	int dis = 0, dis_temp = 0, dis_sum = 0;
	int dis_filtered = 0;
	int distance[5]={0};
	for (int i = 0; i < 5; i++){
		dis = Get_Distance();
		distance[i] = dis;
		delay_ms(20);
	}
	
	//排序
	for (int i = 0; i < 5; i++){
		for (int j = 0; j < 5-i; j++){
			if (distance[j] > distance[j+1]){
				dis_temp = distance[j];
				distance[j] = distance[j+1];
				distance[j+1] = dis_temp;
			}
		}
	}

	//找到两个数最大的差
	int error_max = distance[1] - distance[0];
	int error_max_index = 0;
	for (int i = 1; i < 4; i++){
		if (distance[i+1]-distance[i] > error_max){
			error_max = distance[i+1]-distance[i];
			error_max_index = i;
		}
	}

	if (error_max_index + 1 > 2){//优先取前部分的数
		for (int i = 0; i < error_max_index; i++){
			dis_sum += distance[i];
		}
		dis_filtered = dis_sum / (error_max_index +1);
	}
	else{//后部分
		for (int i = 4; i > error_max_index; i--){
			dis_sum += distance[i];
		}
		dis_filtered = dis_sum / (4 - error_max_index);
	}

	return dis_filtered;

}
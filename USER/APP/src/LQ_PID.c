/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�F3277���İ��ĸ��
����    д������Ƽ�
��E-mail  ��chiusir@163.com
������汾��V1.0 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��12��24�գ��������£����ע���°棡
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��IAR7.8 KEIL5.24�����ϰ汾
��Target �� MM32F3277
��SYS PLL�� 120MHz Ƶ��̫�߿����޷�����system_mm32f327x.c
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <LQ_PID.h>

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�float constrain_float(float amt, float low, float high)
@����˵�����޷�����
@����˵����amt������ low�����ֵ high�����ֵ
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����constrain_float(pid->integrator, -pid->imax, pid->imax);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/float constrain_float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// pid������ʼ������
void PidInit(pid_param_t * pid)
{
  pid->kp        = 14.3;
  pid->ki        = 10.6;
  pid->kd        = 0;
  pid->imax      = 0;
  pid->out_p     = 0;
  pid->out_i     = 0;
  pid->out_d     = 0;
  pid->out       = 0;
  pid->integrator= 0;
  pid->last_error= 0;
  pid->last_derivative   = 0;
  pid->last_t    = 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�float constrain_float(float amt, float low, float high)
@����˵����pidλ��ʽ���������
@����˵����pid     pid����  error   pid�������
@�������أ�PID������
@�޸�ʱ�䣺2022/03/15
@���÷�����
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/float PidLocCtrl(pid_param_t * pid, float error)
{
  /* �ۻ���� */
  pid->integrator += error;
  
  /* ����޷� */
  constrain_float(pid->integrator, -pid->imax, pid->imax);
  
  
  pid->out_p = pid->kp * error;
  pid->out_i = pid->ki * pid->integrator;
  pid->out_d = pid->kd * (error - pid->last_error);
  
  pid->last_error = error;
  
  pid->out = pid->out_p + pid->out_i + pid->out_d;
  
  return pid->out;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�float constrain_float(float amt, float low, float high)
@����˵����pid����ʽ���������
@����˵����
@param    pid     pid����
@param    error   pid�������
@�������أ�PID������   ע���������Ѿ��������ϴν��
@�޸�ʱ�䣺2020��4��1��
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
float PidIncCtrl(pid_param_t * pid, float error)
{
  
  pid->out_p = pid->kp * (error - pid->last_error);
  pid->out_i = pid->ki * error;
  pid->out_d = pid->kd * ((error - pid->last_error) - pid->last_derivative);
  
  pid->last_derivative = error - pid->last_error;
  pid->last_error = error;
  
  pid->out += pid->out_p + pid->out_i + pid->out_d;
  
  return pid->out;
}


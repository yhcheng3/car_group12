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
#ifndef _LQOLED_H
#define _LQOLED_H

#include <stdint.h>

void OLED_Init(void);			//OLED��ʼ��
void OLED_Fill(void);			//OLEDȫ��
void OLED_CLS(void);      //OLED����
void OLED_PutPixel(unsigned char x,unsigned char y);      //��һ����
void OLED_ClrDot(unsigned char x,unsigned char y);        //���8����
void OLED_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char gif);//������
void OLED_P6x8Str(unsigned char x,unsigned char y, char ch[]);                                             //д��һ���׼ASCII�ַ���
void OLED_P8x16Str(unsigned char x,unsigned char y, char ch[]);                                            //д��һ���׼ASCII�ַ���
void OLED_P14x16Str(unsigned char x,unsigned char y,unsigned char ch[]);                                   //��������ַ���
void OLED_Show_BMP(unsigned char x0,unsigned char y0,unsigned char wide,unsigned char high,const unsigned char * bmp);//��ʾBMPͼƬ ��� 128��64
void OLED_Show_LQLogo(void);                                   //��ʾ����ͼƬ128��64
void OLED_Show_Frame80(void);                                  //������ OV7725��ʾͼ��ʱ ���������������
void OLED_Show_Frame94(void);                                  //������ MT9V034��ʾͼ��ʱ ���������������
void OLED_Road(unsigned short high, unsigned short wide, unsigned char *Pixle);//OLED ��ֵ��ͼ����ʾ
void OLED_WrCmd(unsigned char cmd);                 //OLED����
void OLED_Set_Pos(unsigned char x, unsigned char y);//OLED���õ�
void Test_OLED(void);                               //OLED ���Ժ���

#endif



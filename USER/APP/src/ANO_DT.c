/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�STC16���İ�
����    д��chiusir
��E-mail  ��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2021��1��23��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��IDE��STC16
��Target �� C251Keil5.6������
��SYS PLL�� 30MHzʹ���ڲ�����
=================================================================
STC16���������Ƶ��
����Ƽ�Bվ��ַ��https://space.bilibili.com/95313236
STC16�������زο���Ƶ�� https://www.bilibili.com/video/BV1gy4y1p7T1/
STC16һ����ӽ�����Ƶ�� https://www.bilibili.com/video/BV1Jy4y1e7R4/
=================================================================
����ʱ, ѡ��ʱ�� 30MHZ (�û��������޸�Ƶ��).
STC16F��������:����IRCBND=0��ISP��������Ϊ24M��
Ȼ��IRCBND=0������Ƶ��Ϊ30M��
����IRCBND=1������Ƶ��Ϊ30M������Ϊ׼
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ

�����������վ��λ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "ANO_DT.h"

//ʹ������4.3��λ��Э��
/////////////////////////////////////////////////////////////////////////////////////
//���ݲ�ֺ궨�壬�ڷ��ʹ���1�ֽڵ���������ʱ������int16��float�ȣ���Ҫ�����ݲ�ֳɵ����ֽڽ��з���
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )     /*!< uint32_t ���ݲ�� byte0  */
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )     /*!< uint32_t ���ݲ�� byte1  */
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )     /*!< uint32_t ���ݲ�� byte2  */
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )     /*!< uint32_t ���ݲ�� byte3  */

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�� �� ����vvoid ANO_DT_send_int16(short data1,..., short data8 )
@��    �ܣ�Send_Data������Э�������з������ݹ���ʹ�õ��ķ��ͺ���
@�� �� ֵ��dataToSend:Ҫ���͵������׵�ַ length:Ҫ���͵����ݳ���
@�� �� ֵ��
@�� �� ֵ���� 
@��    �ߣ�L Q
@�����£�2022��02��22�� 
@��    ע����ֲʱ���û�Ӧ��������Ӧ�õ����������ʹ�õ�ͨ�ŷ�ʽ��ʵ�ִ˺���
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ANO_DT_Send_Data(unsigned char *dataToSend , unsigned short length)
{

    /**ʹ�ô��������������ݣ������Ҫ1.5ms*/
	//UART4_PutBuff(dataToSend, length);     //�����޸Ĳ�ͬ�Ĵ��ڷ�������

}
/**  �������ݻ��� */
unsigned char data_to_send[50];

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�� �� ����vvoid ANO_DT_send_int16(short data1,..., short data8 )
@��    �ܣ�������λ���û�Э�飬ʹ��ʱռ��MCU��Դ�ϴ��ܳ�ʱ��ʹ��
@�� �� ֵ��short data1,..., short data8, �����͵�����
@�� �� ֵ��
@�� �� ֵ���� 
@��    �ߣ�L Q
@�����£�2022��02��22�� 
@����汾��V1.0
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ANO_DT_send_int16(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8 )
{
  unsigned char  _cnt=0;
	unsigned char  sum = 0, i = 0;
	
  data_to_send[_cnt++] = 0xAA;      //����Э��֡ͷ  0xAAAA
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xF1;      //ʹ���û�Э��֡0xF1
  data_to_send[_cnt++] = 16;        //8��int16_t ���� 16���ֽ�

	data_to_send[_cnt++]=BYTE1(data1);
	data_to_send[_cnt++]=BYTE0(data1);

	data_to_send[_cnt++]=BYTE1(data2);
	data_to_send[_cnt++]=BYTE0(data2);

	data_to_send[_cnt++]=BYTE1(data3);
	data_to_send[_cnt++]=BYTE0(data3);

  data_to_send[_cnt++]=BYTE1(data4);
	data_to_send[_cnt++]=BYTE0(data4);

	data_to_send[_cnt++]=BYTE1(data5);
	data_to_send[_cnt++]=BYTE0(data5);

	data_to_send[_cnt++]=BYTE1(data6);
	data_to_send[_cnt++]=BYTE0(data6);

  data_to_send[_cnt++]=BYTE1(data7);
	data_to_send[_cnt++]=BYTE0(data7);

	data_to_send[_cnt++]=BYTE1(data8);
	data_to_send[_cnt++]=BYTE0(data8);

  sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;	
	
//	uart_SendBuf(USART_3,data_to_send );
}

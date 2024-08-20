
#include "include.h"

extern short   gyro[3], accel[3], sensors;
extern float   Pitch, Roll, Yaw; 

float   Pitch2=0.00001;
float   *Pitch3;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Test_LQDMP(void)
@����˵����dmp���Ժ���
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����Test_LQDMP();
@��    ע����ʼ��ʱ����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_LQDMP(void)
{
  char  txt[10];
  u8 res;
	
  SOFT_IIC_Init();                  //IIC��ʼ��//IIC��ʼ��
	OLED_CLS();                 //OLED����
	
  printf("LQ DMP Test\r\n");
  delay_ms(100);  
	OLED_P8x16Str(0,0,"LQ 6050 Test");// �ַ�����ʾ
	
  LQ_I2C_Read(0x68, WHO_AM_I, 1, &res);   //��ȡICM20689��ID  
  sprintf(txt,"ID:0x%02X",res);  
  OLED_P6x8Str(70,2,txt);     // �ַ�����ʾ
  LQ_DMP_Init();
  delay_ms(100);
  while(1)
  {       
    LQ_DMP_Read(); 
    Pitch2=(float)Pitch;
    Pitch3=&Pitch;
     
    sprintf((char*)txt,"Pitch:%.2f", Pitch);
		OLED_P6x8Str(0,2,txt);
    sprintf((char*)txt,"Roll:%.2f", Roll);
    OLED_P6x8Str(0,3,txt);
    sprintf((char*)txt,"Yaw:%.2f", Yaw);
    OLED_P6x8Str(0,4,txt);
		
    sprintf((char*)txt,"ax:%06d",accel[0]);
    OLED_P6x8Str(0,5,txt);	
    sprintf((char*)txt,"ay:%06d",accel[1]);
		OLED_P6x8Str(0,6,txt);
    sprintf((char*)txt,"az:%06d",accel[2]);
		OLED_P6x8Str(0,7,txt);
		
    sprintf((char*)txt,"gx:%06d",gyro[0]);
		OLED_P6x8Str(70,5,txt);
    sprintf((char*)txt,"gy:%06d",gyro[1]);
		OLED_P6x8Str(70,6,txt);
    sprintf((char*)txt,"gz:%06d",gyro[2]);
		OLED_P6x8Str(70,7,txt);
    //��λ��
//    ANO_DT_send_int16((short)Pitch, (short)Roll, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
  }
}


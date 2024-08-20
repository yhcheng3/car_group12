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

#include "include.h"

#define MPU6050_ADDR  0x68  //IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

unsigned char IIC_ICM20602 = 0;
unsigned char IIC_ICM20689 = 0;
unsigned char IIC_MPU9250 = 0;
unsigned char IIC_ICM42605 = 0;
unsigned char IIC_MPU6050 = 0;
unsigned char IIC_errorid = 0;

unsigned char Gyro_ID = 0;


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char Gyro_Chose(void)
@����˵����ʶ��ʹ�õ�������
@����˵������
@�������أ�������ID
@�޸�ʱ�䣺2022/03/24
@���÷�����Gyro_Chose();
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char Gyro_Chose(void)
{
    Gyro_ID = MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);
    switch(Gyro_ID)
    {
        case 0x12:IIC_ICM20602 = 1;break;
        case 0x71:IIC_MPU9250 = 1;break;
        case 0x98:IIC_ICM20689 = 1;break;
        case 0x42:IIC_ICM42605 = 1;break;
        case 0x68:IIC_MPU6050 = 1;break;
        default :IIC_errorid = 1;return 0;
    }
    return Gyro_ID;
}


signed short  aacx,aacy,aacz;            //���ٶȴ�����ԭʼ����
signed short  gyrox,gyroy,gyroz;         //������ԭʼ����
short magx, magy, magz;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Test_IIC_Gyro(void)
@����˵������ȡMPU6050����ICM20602ԭʼ���� ����
@����˵����void
@�������أ�
@�޸�ʱ�䣺2022/03/15
@���÷����� Test_MPU6050();
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_IIC_Gyro(void)
{
	unsigned char res;
	char  txt[30];


	SOFT_IIC_Init();                  //IIC��ʼ��//IIC��ʼ��
	delay_ms(100);
	res = Gyro_Chose();                //�ж��������ͺ�
	sprintf(txt,"ID:0X%2x",res);

	OLED_Init();                //LCD��ʼ��
	delay_ms(100);
	OLED_CLS();                 //LCD����
	OLED_P6x8Str(70,2,txt);     // �ַ�����ʾ
	if(IIC_ICM20602)
			OLED_P8x16Str(0,0,"LQ 20602 Test");// �ַ�����ʾ
	if(IIC_MPU6050)
			OLED_P8x16Str(0,0,"LQ 6050 Test");// �ַ�����ʾ
	if(IIC_ICM42605)
			OLED_P8x16Str(0,0,"LQ 42605 Test");// �ַ�����ʾ
	if(IIC_MPU9250)
			OLED_P8x16Str(0,0,"LQ 9250 Test");// �ַ�����ʾ
	if(IIC_ICM20689)
			OLED_P8x16Str(0,0,"LQ 20689 Test");// �ַ�����ʾ
	if(IIC_errorid)
	{
			OLED_P8x16Str(0,0,"Gyro Test fail");// �ַ�����ʾ
//			while(1);
	}

	MPU6050_Init();
	while(1)
    {
	    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);   //�õ����ٶȴ���������
	    if(IIC_MPU9250)
			{
				MPU9250_Get_Magnetometer(&magx, &magy, &magz);
				sprintf((char*)txt,"gx:%06d",magx);
				OLED_P6x8Str(70,5,txt);
				sprintf((char*)txt,"gy:%06d",magy);
				OLED_P6x8Str(70,6,txt);
				sprintf((char*)txt,"gz:%06d",magz);
				OLED_P6x8Str(70,7,txt);
			}
			sprintf((char*)txt,"ax:%06d",aacx);
			OLED_P6x8Str(0,2,txt);
			sprintf((char*)txt,"ay:%06d",aacy);
			OLED_P6x8Str(0,3,txt);
			sprintf((char*)txt,"az:%06d",aacz);
			OLED_P6x8Str(0,4,txt);
			sprintf((char*)txt,"gx:%06d",gyrox);
			OLED_P6x8Str(0,5,txt);
			sprintf((char*)txt,"gy:%06d",gyroy);
			OLED_P6x8Str(0,6,txt);
			sprintf((char*)txt,"gz:%06d",gyroz);
			OLED_P6x8Str(0,7,txt);
		//ANO_DT_send_int16byte16(aacx,aacy,aacz,gyrox,gyroy,gyroz,gyroz,gyroz);
	    delay_ms(100);
    }

}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void delayms_mpu(uint16 ms)
@����˵��������ȷ��ʱ
@����˵������Ҫ��ʱʱ��
@�������أ���
@�޸�ʱ�䣺2022/03/15
@���÷�����delayms_mpu(100);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void delayms_mpu(u16 ms)
{
	while(ms--)
	{
		u16  i = 300*50;
		while(i--)
		{
			__NOP();
		}
	}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU6050_Init(void)
@����˵������ʼ��MPU6050
@����˵������
@�������أ�0����ʼ���ɹ�   1��ʧ��
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU6050_Init();
@��    ע����ʼ������
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU6050_Init(void)
{
    int  res;
    if (IIC_ICM42605)
    {
//        MPU_Write_Byte(MPU6050_ADDR, device_config_reg,bit_soft_reset_chip_config);//chip soft reset
//        delayms(10);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel,0x01);//Change to bank 1
//				MPU_Write_Byte(MPU6050_ADDR,intf_config4,0x03);//4 wire spi mode  ����SPI  Ĭ��ֵ0x03
//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel,0x00);
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config_reg,0x40);//Stream-to-FIFO Mode  ������Y���ݲ��˲�����Ƶ��ѡ��

//				res = MPU_Read_Byte(MPU6050_ADDR,int_source0_reg);
//				MPU_Write_Byte(MPU6050_ADDR,int_source0_reg,0x00);
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config2_reg,0x00);// watermark
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config3_reg,0x02);// watermark
//				MPU_Write_Byte(MPU6050_ADDR,int_source0_reg, (unsigned char)res);
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config1_reg,0x63);// Enable the accel and gyro to the FIFO

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel,0x00);
//				MPU_Write_Byte(MPU6050_ADDR,int_config_reg,0x36);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = (MPU_Read_Byte(MPU6050_ADDR,int_source0_reg)|bit_int_fifo_ths_int1_en);
//				MPU_Write_Byte(MPU6050_ADDR,int_source0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,accel_config0_reg)&0x1F)|(bit_accel_ui_fs_sel_8g));//8g
//				MPU_Write_Byte(MPU6050_ADDR,accel_config0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,accel_config0_reg)&0xF0)|bit_accel_odr_50hz);
//				MPU_Write_Byte(MPU6050_ADDR,accel_config0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,gyro_config0_reg)&0x1F)|(bit_gyro_ui_fs_sel_1000dps));
//				MPU_Write_Byte(MPU6050_ADDR,gyro_config0_reg,(unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,gyro_config0_reg)&0xF0)|bit_gyro_odr_50hz);
//				MPU_Write_Byte(MPU6050_ADDR,gyro_config0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = MPU_Read_Byte(MPU6050_ADDR,pwr_mgmt0_reg)|(bit_accel_mode_ln); // Accel on in LNM
//				MPU_Write_Byte(MPU6050_ADDR,pwr_mgmt0_reg, (unsigned char)res);
//				delayms(1);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = MPU_Read_Byte(MPU6050_ADDR,pwr_mgmt0_reg)|(bit_gyro_mode_ln); // Gyro on in LNM
//				MPU_Write_Byte(MPU6050_ADDR,pwr_mgmt0_reg,(unsigned char)res);
//				delay_ms(1);
				return 0;
    }
    else
    {
        res = MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);           //��ȡMPU6050��ID
        if(res != Gyro_ID)                                 //����ID��ȷ
            return 1;

        res = 0;
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU6050
        delayms_mpu(100);  //��ʱ100ms
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU6050
        res += MPU_Set_Gyro_Fsr(3);					        	   //�����Ǵ�����,��2000dps
        res += MPU_Set_Accel_Fsr(1);					       	   //���ٶȴ�����,��4g
        res += MPU_Set_Rate(1000);						       	   //���ò�����1000Hz
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //�������ֵ�ͨ�˲���   98hz
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);//����CLKSEL,PLL X��Ϊ�ο�
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);//���ٶ��������Ƕ�����

        if(IIC_MPU9250)
        {
            MPU_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);  //�ر�FIFO
            MPU_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ
            res = MPU_Read_Byte(AK8963_ADDR,MPU_MAG_WIA);   //��ȡMPU9250��ID
            if(res == AK8963_ID)                                //����ID��ȷ
            {
                MPU_Write_Byte(AK8963_ADDR,MPU_MAG_CNTL2_REG,0X01);//�������
                delayms_mpu(100);
                MPU_Write_Byte(AK8963_ADDR,MPU_MAG_CNTL1_REG,0X16);//����Ϊ����ģʽ�����16bit
            }
        }
        return 0;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Set_Gyro_Fsr(unsigned char fsr)
@����˵�������������ǲ�����Χ
@����˵����fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
@�������أ�0�����óɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU9250_Set_Gyro_Fsr(3);  //�����Ǵ�����,��2000dps
@��    ע���ڲ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_Gyro_Fsr(unsigned char fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Set_Accel_Fsr(uint8_t fsr)
@����˵�������ü��ٶȼƲ�����Χ
@����˵����fsr:0,��2g;1,��4g;2,��8g;3,��16g
@�������أ�0�����óɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Set_Accel_Fsr(1); //���ٶȴ�����,��4g
@��    ע���ڲ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_Accel_Fsr(unsigned char fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Set_LPF(uint8_t fsr)
@����˵�����������ֵ�ͨ�˲�
@����˵����lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
@�������أ�0�����óɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU6050_Set_LPF(100);
@��    ע���ڲ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_LPF(u16 lpf)
{
	unsigned char  dat=0;
	if(lpf>=188)dat=1;
	else if(lpf>=98)dat=2;
	else if(lpf>=42)dat=3;
	else if(lpf>=20)dat=4;
	else if(lpf>=10)dat=5;
	else dat=6;
	return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,dat);//�������ֵ�ͨ�˲���
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Set_Rate(uint16_t rate)
@����˵�������ò�����
@����˵����rate:4~1000(Hz)
@�������أ�0�����óɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Set_Rate(1000); //���ò�����1000Hz
@��    ע���ڲ�����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_Rate(u16 rate)
{
	unsigned char  dat;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	dat=1000/rate-1;
	MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,dat);	    //�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	                                //�Զ�����LPFΪ�����ʵ�һ��
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�short MPU_Get_Temperature(void)
@����˵������ȡ�¶�ֵ
@����˵������
@�������أ��¶�ֵ(������100��)
@�޸�ʱ�䣺2022/03/15
@���÷�����signed short temp = MPU_Get_Temperature();
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
short MPU_Get_Temperature(void)
{
    unsigned char  buf[2];
    short raw;
	float temp;
	MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((u16)buf[0]<<8)|buf[1];
    temp=21+((double)raw)/333.87;
    return (short)temp*100;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Get_Gyroscope(signed short *gx,signed short *gy,signed short *gz)
@����˵������ȡ������ֵ
@����˵����gx,gy,gz:������x,y,z���ԭʼ����(������)
@�������أ�0����ȡ�ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Get_Gyroscope(&data[0], &data[1], &data[2]);
@��    ע���趨�� signed short data[6];
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Get_Gyroscope(signed short *gx,signed short *gy,signed short *gz)
{
    unsigned char  buf[6],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];
		*gy=((u16)buf[2]<<8)|buf[3];
		*gz=((u16)buf[4]<<8)|buf[5];
	}
    return res;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Get_Accelerometer(signed short *ax,signed short *ay,signed short *az)
@����˵������ȡ���ٶ�ֵ
@����˵����ax,ay,az:������x,y,z���ԭʼ����(������)
@�������أ�0����ȡ�ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Get_Accelerometer(&data[0], &data[1], &data[2]);
@��    ע���趨�� signed short data[6];
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Get_Accelerometer(signed short *ax,signed short *ay,signed short *az)
{
    unsigned char  buf[6],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];
		*ay=((u16)buf[2]<<8)|buf[3];
		*az=((u16)buf[4]<<8)|buf[5];
	}
    return res;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Get_Raw_data(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz)
@����˵������ȡ ���ٶ�ֵ ���ٶ�ֵ
@����˵����gx,gy,gz:������x,y,z���ԭʼ����(������)��ax,ay,az:������x,y,z���ԭʼ����(������)
@�������أ�0����ȡ�ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Get_Raw_data(&data[0], &data[1], &data[2],&data[3], &data[4], &data[5]);
@��    ע���趨�� signed short data[6];
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Get_Raw_data(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz)
{
    if(IIC_ICM42605)
    {
        unsigned char  buf[14],res;
        res=MPU_Read_Len(MPU6050_ADDR,0X1F,12,buf);
        if(res==0)
        {
            *ax=((u16)buf[0]<<8)|buf[1];
            *ay=((u16)buf[2]<<8)|buf[3];
            *az=((u16)buf[4]<<8)|buf[5];
            *gx=((u16)buf[6]<<8)|buf[7];
            *gy=((u16)buf[8]<<8)|buf[9];
            *gz=((u16)buf[10]<<8)|buf[11];
        }
        return res;
    }
    else
    {
        unsigned char  buf[14],res;

        res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
        if(res==0)
        {
            *ax=((u16)buf[0]<<8)|buf[1];
            *ay=((u16)buf[2]<<8)|buf[3];
            *az=((u16)buf[4]<<8)|buf[5];
            *gx=((u16)buf[8]<<8)|buf[9];
            *gy=((u16)buf[10]<<8)|buf[11];
            *gz=((u16)buf[12]<<8)|buf[13];
        }
        return res;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU9250_Get_Magnetometer(short *mx,short *my,short *mz)
@����˵������ȡ������ֵ
@����˵����mx,my,mz:��������x,y,z���ԭʼ����(������)
@�������أ�0����ȡ�ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU9250_Get_Magnetometer(&data[0], &data[1], &data[2]);
@��    ע��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU9250_Get_Magnetometer(short *mx,short *my,short *mz)
{

  unsigned char  buf[6],res;
  res=MPU_Read_Len(AK8963_ADDR,MPU_MAG_XOUTL_REG,6,buf);
  if(res==0)
  {
    *mx=((u16)buf[1]<<8)|buf[0];
    *my=((u16)buf[3]<<8)|buf[2];
    *mz=((u16)buf[5]<<8)|buf[4];
  }
  MPU_Read_Byte(AK8963_ADDR,MPU_MAG_ST2_REG);
  return res;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
@����˵����IIC ������
@����˵����addr:������ַ��reg :Ҫ��ȡ�ļĴ�����ַ
@����˵����len :Ҫ��ȡ�ĳ��ȣ�buf :��ȡ�������ݴ洢��
@�������أ� 0 ����ȡ�ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
@��    ע���ײ����� ��ֲʱ��Ҫ�޸�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_ReadMultByteFromSlave(addr<<1, reg, len, buf);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,
@                                         unsigned char len,unsigned char *buf)
@����˵����IIC ����д
@����˵����addr:������ַ��reg :Ҫд��ļĴ�����ַ��len :Ҫд��ĳ���
@����˵����buf :д�뵽�����ݴ洢��
@�������أ�0 ��д��ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Write_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
@��    ע���ײ����� ��ֲʱ��Ҫ�޸�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_WriteMultByteToSlave(addr<<1, reg, len, buf);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Write_Byte(unsigned char addr,
@                                           unsigned char reg,unsigned char value)
@����˵����IIC дһ���Ĵ���
@����˵����addr  :������ַ��reg   :�Ĵ�����ַ��value :Ҫд���ֵ
@�������أ�0 ����ȡ�ɹ�
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,1);
@��    ע���ײ����� ��ֲʱ��Ҫ�޸�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Write_Byte(unsigned char addr,unsigned char reg,unsigned char value)
{
    return IIC_WriteByteToSlave(addr<<1, reg, value);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�unsigned char MPU_Read_Byte(unsigned char addr,unsigned char reg)
@����˵����IIC ��һ���Ĵ���
@����˵����addr  :������ַ��reg   :�Ĵ�����ַ
@�������أ���ȡ��ֵ
@�޸�ʱ�䣺2022/03/15
@���÷�����MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);
@��    ע���ײ����� ��ֲʱ��Ҫ�޸�
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Read_Byte(unsigned char addr,unsigned char reg)
{
    unsigned char value[1];
    MPU_Read_Len(addr, reg, 1, value);
    return value[0];
}

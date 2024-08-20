/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编    写：龙邱科技
@E-mail  ：chiusir@163.com
@编译IDE ：KEIL5.25.3及以上版本
@使用平台：北京龙邱智能科技全向福来轮小车
@最后更新：2022年02月19日，持续更新，请关注最新版！
@功能介绍：
@相关信息参考下列地址
@网    站：http://www.lqist.cn
@淘宝店铺：http://longqiu.taobao.com
@软件版本：V1.0 版权所有，单位使用请先联系授权
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

//龙邱OLED 模块管脚顺序
#define OLED_CK       GPIOC,GPIO_PIN_15     //OLED CK管脚
#define OLED_DI	      GPIOC,GPIO_PIN_14     //OLED DI管脚
#define OLED_RST      GPIOC,GPIO_PIN_13     //OLED RST管脚
#define OLED_DC	      GPIOA,GPIO_PIN_0      //OLED DC管脚
//#define OLED_CS	      GPIOA,GPIO_PIN_12   //OLED CS管脚 默认拉低，可以不用

#define OLED_DC_H  	HAL_GPIO_WritePin(OLED_DC,GPIO_PIN_SET);
#define OLED_DC_L  	HAL_GPIO_WritePin(OLED_DC,GPIO_PIN_RESET);

#define OLED_RST_H 	HAL_GPIO_WritePin(OLED_RST,GPIO_PIN_SET);
#define OLED_RST_L 	HAL_GPIO_WritePin(OLED_RST,GPIO_PIN_RESET);

#define OLED_DI_H  	HAL_GPIO_WritePin(OLED_DI,GPIO_PIN_SET);
#define OLED_DI_L  	HAL_GPIO_WritePin(OLED_DI,GPIO_PIN_RESET);

#define OLED_CK_H  	HAL_GPIO_WritePin(OLED_CK,GPIO_PIN_SET);
#define OLED_CK_L  	HAL_GPIO_WritePin(OLED_CK,GPIO_PIN_RESET);

//#define OLED_CS_H  HAL_GPIO_WritePin(OLED_CS,1);      /*!< CS管脚 */
//#define OLED_CS_L  HAL_GPIO_WritePin(OLED_CS,0);     /*!< CS管脚 */


#define X_WIDTH 132
#define Y_WIDTH 64

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：OLED_Init
@功能说明：OLED初始化
@参数说明：无
@函数返回：无
@修改时间：2019/6/13
@调用方法：OLED_Init();
@备    注：如果修改管脚 需要修改初始化的管脚 
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void OLED_Init(void)
{ 
  
  GPIO_InitTypeDef GPIO_Initure;
    
	__HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
	__HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOB时钟

	GPIO_Initure.Pin=GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;  //PC13 14 15
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //输出
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
	HAL_GPIO_Init(GPIOC,&GPIO_Initure); 				//配置初始化

	GPIO_Initure.Pin=GPIO_PIN_0;            		//PA0
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;         //输出
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 				//配置

  OLED_CK_H;
  OLED_RST_L;
  delay_ms(50);
  OLED_RST_H;
  
  OLED_WrCmd(0xae);//--turn off oled panel
  OLED_WrCmd(0x00);//---set low column address
  OLED_WrCmd(0x10);//---set high column address
  OLED_WrCmd(0x40);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
  OLED_WrCmd(0x81);//--set contrast control register
  OLED_WrCmd(0xcf); // Set SEG Output Current Brightness
  OLED_WrCmd(0xa1);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
  OLED_WrCmd(0xc8);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
  OLED_WrCmd(0xa6);//--set normal display
  OLED_WrCmd(0xa8);//--set multiplex ratio(1 to 64)
  OLED_WrCmd(0x3f);//--1/64 duty
  OLED_WrCmd(0xd3);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
  OLED_WrCmd(0x00);//-not offset
  OLED_WrCmd(0xd5);//--set display clock divide ratio/oscillator frequency
  OLED_WrCmd(0x80);//--set divide ratio, Set Clock as 100 Frames/Sec
  OLED_WrCmd(0xd9);//--set pre-charge period
  OLED_WrCmd(0xf1);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  OLED_WrCmd(0xda);//--set com pins hardware configuration
  OLED_WrCmd(0x12);
  OLED_WrCmd(0xdb);//--set vcomh
  OLED_WrCmd(0x40);//Set VCOM Deselect Level
  OLED_WrCmd(0x20);//-Set Page Addressing Mode (0x00/0x01/0x02)
  OLED_WrCmd(0x02);//
  OLED_WrCmd(0x8d);//--set Charge Pump enable/disable
  OLED_WrCmd(0x14);//--set(0x10) disable
  OLED_WrCmd(0xa4);// Disable Entire Display On (0xa4/0xa5)
  OLED_WrCmd(0xa6);// Disable Inverse Display On (0xa6/a7)
  OLED_WrCmd(0xaf);//--turn on oled panel
  OLED_CLS();      //初始清屏
  OLED_Set_Pos(0,0);
  
} 
/** 6*8字库  */
const unsigned char F6x8[][6] =
{
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
  { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
  { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
  { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
  { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
  { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
  { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
  { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
  { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
  { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
  { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
  { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
  { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
  { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
  { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
  { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
  { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
  { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
  { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
  { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
  { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
  { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
  { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
  { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
  { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
  { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
  { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
  { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
  { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
  { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
  { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
  { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
  { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
  { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
  { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
  { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
  { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
  { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
  { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
  { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
  { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
  { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
  { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
  { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
  { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
  { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
  { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
  { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
  { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
  { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
  { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
  { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
  { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
  { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
  { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
  { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
  { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
  { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
  { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
  { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
  { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
  { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
  { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
  { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
  { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
  { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
  { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
  { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
  { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
  { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
  { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
  { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
  { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
  { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
  { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
  { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
  { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
  { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
  { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
  { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
  { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
  { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
  { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
  { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
  { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
  { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
  { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
  { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
  { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
  { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines
};
/* MDK AC6编译器不支持 GB3212编码的中文 因此会出现警告 不影响正常使用 忽略即可 */
/** 简单的汉字字库 */
const unsigned char F14x16_Idx[] =
{
  "北京电子科技职业学院龙邱智能机器人遥控器接收发送按键："
};

/** 14*16 简单的汉字字库*/
const unsigned char F14x16[] = {
  0x20,0x20,0x20,0x20,0xFE,0x00,0x00,0x00,0xFE,0x80,0x40,0x20,0x10,0x00,
  0x10,0x10,0x08,0x04,0x7F,0x00,0x00,0x00,0x3F,0x40,0x40,0x40,0x70,0x00,//北0
  0x08,0x08,0xE8,0x28,0x28,0x2A,0x2C,0x28,0x28,0x28,0xE8,0x08,0x08,0x00,
  0x00,0x20,0x13,0x0A,0x42,0x42,0x7E,0x02,0x02,0x0A,0x13,0x20,0x00,0x00,//京1
  0x00,0xF0,0x90,0x90,0x90,0x90,0xFE,0x90,0x90,0x90,0x90,0xF0,0x00,0x00,
  0x00,0x0F,0x04,0x04,0x04,0x04,0x3F,0x44,0x44,0x44,0x44,0x47,0x70,0x00,//电2
  0x80,0x82,0x82,0x82,0x82,0x82,0xC2,0xA2,0x92,0x8A,0x86,0x82,0x80,0x00,
  0x00,0x00,0x00,0x00,0x40,0x40,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//子3
  0x40,0x48,0x48,0xF8,0x44,0x44,0x20,0xC4,0x18,0x00,0xFE,0x00,0x00,0x00,
  0x08,0x04,0x03,0x7F,0x02,0x04,0x02,0x02,0x02,0x02,0x7F,0x01,0x01,0x00,//科4
  0x10,0x10,0xFE,0x10,0x00,0x90,0x90,0x90,0xFE,0x90,0x90,0x10,0x00,0x00,
  0x42,0x42,0x7F,0x01,0x40,0x40,0x23,0x14,0x08,0x14,0x23,0x40,0x40,0x00,//技5
  0x04,0xFC,0x24,0x24,0xFC,0x04,0x00,0xFC,0x04,0x04,0x04,0xFC,0x00,0x00,
  0x10,0x1F,0x11,0x09,0x7F,0x08,0x40,0x33,0x02,0x02,0x02,0x13,0x60,0x00,//职6
  0x10,0x60,0x80,0x00,0xFE,0x00,0x00,0x00,0xFE,0x00,0x80,0x70,0x00,0x00,
  0x40,0x40,0x43,0x40,0x7F,0x40,0x40,0x40,0x7F,0x42,0x41,0x40,0x40,0x00,//业7
  0x70,0x10,0x92,0x94,0x90,0x92,0x94,0x90,0x98,0x94,0x12,0x10,0x70,0x00,
  0x04,0x04,0x04,0x04,0x44,0x44,0x7E,0x06,0x05,0x04,0x04,0x04,0x04,0x00,//学8
  0xFC,0x04,0x64,0x9C,0x00,0x18,0x48,0x48,0x4A,0x4C,0x48,0x48,0x18,0x00,
  0x7F,0x08,0x08,0x07,0x40,0x22,0x12,0x0E,0x02,0x02,0x3E,0x42,0x62,0x00,//院9
  0x20,0x20,0x20,0x20,0xA0,0x7E,0x20,0xE0,0x24,0x28,0x20,0xA0,0x20,0x00,
  0x40,0x20,0x10,0x0C,0x23,0x20,0x10,0x3F,0x44,0x42,0x41,0x40,0x70,0x00,//龙10
  0x00,0xFC,0x44,0x44,0x44,0xC2,0x42,0x00,0xFC,0x04,0x44,0xB4,0x0C,0x00,
  0x10,0x1F,0x10,0x10,0x08,0x0F,0x08,0x00,0x7F,0x00,0x10,0x10,0x0F,0x00,//邱11
  0x28,0xA6,0x64,0x3C,0x64,0xA4,0x20,0xFC,0x84,0x84,0x84,0xFC,0x00,0x00,
  0x01,0x00,0x7E,0x2A,0x2A,0x2A,0x2A,0x2A,0x2A,0x2A,0x7E,0x00,0x00,0x00,//智12
  0x10,0x98,0x94,0x92,0x90,0x98,0x30,0x00,0x3E,0x48,0x48,0x44,0x74,0x00,
  0x00,0x7F,0x0A,0x0A,0x4A,0x7F,0x00,0x00,0x3F,0x44,0x44,0x42,0x71,0x00,//能13
  0x10,0x90,0xFE,0x90,0x10,0x00,0xFC,0x04,0x04,0x04,0xFC,0x00,0x00,0x00,
  0x02,0x01,0x7F,0x00,0x41,0x30,0x0F,0x00,0x00,0x00,0x3F,0x40,0x70,0x00,//机14
  0x80,0x9E,0x92,0x92,0x92,0x9E,0xC0,0x9E,0x92,0xB2,0xD2,0x9E,0x80,0x00,
  0x04,0x7C,0x4A,0x4A,0x49,0x78,0x00,0x78,0x49,0x4A,0x4A,0x7C,0x04,0x00,//器15
  0x00,0x00,0x00,0x00,0x00,0x80,0x7E,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
  0x40,0x40,0x20,0x10,0x0C,0x03,0x00,0x03,0x0C,0x10,0x20,0x40,0x40,0x00,//人16
  0x40,0x42,0xCC,0x00,0x84,0x6C,0x54,0x4C,0xD4,0x44,0x52,0x4E,0x02,0x00,
  0x40,0x20,0x1F,0x20,0x42,0x5A,0x52,0x52,0x5F,0x52,0x52,0x5A,0x42,0x00,//遥17
  0x10,0x10,0xFE,0x10,0x00,0x98,0x48,0x2A,0x0C,0x08,0x28,0x48,0x98,0x00,
  0x42,0x42,0x7F,0x01,0x00,0x40,0x42,0x42,0x7E,0x42,0x42,0x42,0x40,0x00,//控18
  0x80,0x9E,0x92,0x92,0x92,0x9E,0xC0,0x9E,0x92,0xB2,0xD2,0x9E,0x80,0x00,
  0x04,0x7C,0x4A,0x4A,0x49,0x78,0x00,0x78,0x49,0x4A,0x4A,0x7C,0x04,0x00,//器19
  0x10,0x10,0xFE,0x10,0x50,0x48,0x58,0xEA,0x4C,0x68,0x58,0x48,0x40,0x00,
  0x42,0x42,0x7F,0x01,0x42,0x4A,0x2E,0x13,0x12,0x2A,0x26,0x42,0x02,0x00,//接20
  0x00,0xF8,0x00,0x00,0xFE,0x00,0x40,0x20,0xDE,0x10,0x10,0xF0,0x10,0x00,
  0x00,0x07,0x02,0x01,0x7F,0x00,0x40,0x20,0x17,0x08,0x16,0x21,0x40,0x00,//收21
  0x00,0x30,0x2C,0x20,0xA0,0x70,0x2E,0x20,0x20,0x20,0x24,0x28,0x20,0x00,
  0x10,0x08,0x44,0x42,0x41,0x27,0x29,0x11,0x29,0x25,0x43,0x40,0x40,0x00,//发22
  0x80,0x84,0x98,0x00,0x80,0x92,0x94,0x90,0xF0,0x90,0x94,0x92,0x80,0x00,
  0x40,0x20,0x1F,0x20,0x40,0x50,0x48,0x46,0x41,0x42,0x44,0x58,0x40,0x00,//送23
  0x10,0x10,0xFE,0x10,0x00,0xB8,0x88,0x88,0xEA,0x8C,0x88,0x88,0xB8,0x00,
  0x42,0x42,0x7F,0x01,0x00,0x40,0x44,0x27,0x18,0x08,0x14,0x23,0x40,0x00,//按24
  0x20,0x50,0xCE,0x48,0x00,0xC8,0xB8,0x20,0xA8,0xFE,0xA8,0xF8,0x20,0x00,
  0x02,0x02,0x3F,0x52,0x28,0x10,0x2F,0x20,0x4A,0x7F,0x4A,0x4A,0x48,0x00,//键25
  0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x19,0x19,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//：26
};


/** 128X64I液晶底层驱动[8X16]字体库 [8X16]西文字符的字模数据 (纵向取模,字节倒序) !"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\]^_`abcdefghijklmnopqrstuvwxyz{|}~ */
const unsigned char F8X16[]=
{
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,// 0
  0x00,0x00,0x00,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x33,0x30,0x00,0x00,0x00,//!1
  0x00,0x10,0x0C,0x06,0x10,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//"2
  0x40,0xC0,0x78,0x40,0xC0,0x78,0x40,0x00,0x04,0x3F,0x04,0x04,0x3F,0x04,0x04,0x00,//#3
  0x00,0x70,0x88,0xFC,0x08,0x30,0x00,0x00,0x00,0x18,0x20,0xFF,0x21,0x1E,0x00,0x00,//$4
  0xF0,0x08,0xF0,0x00,0xE0,0x18,0x00,0x00,0x00,0x21,0x1C,0x03,0x1E,0x21,0x1E,0x00,//%5
  0x00,0xF0,0x08,0x88,0x70,0x00,0x00,0x00,0x1E,0x21,0x23,0x24,0x19,0x27,0x21,0x10,//&6
  0x10,0x16,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//'7
  0x00,0x00,0x00,0xE0,0x18,0x04,0x02,0x00,0x00,0x00,0x00,0x07,0x18,0x20,0x40,0x00,//(8
  0x00,0x02,0x04,0x18,0xE0,0x00,0x00,0x00,0x00,0x40,0x20,0x18,0x07,0x00,0x00,0x00,//)9
  0x40,0x40,0x80,0xF0,0x80,0x40,0x40,0x00,0x02,0x02,0x01,0x0F,0x01,0x02,0x02,0x00,//*10
  0x00,0x00,0x00,0xF0,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x1F,0x01,0x01,0x01,0x00,//+11
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xB0,0x70,0x00,0x00,0x00,0x00,0x00,//,12
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,//-13
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,//.14
  0x00,0x00,0x00,0x00,0x80,0x60,0x18,0x04,0x00,0x60,0x18,0x06,0x01,0x00,0x00,0x00,///15
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x0F,0x10,0x20,0x20,0x10,0x0F,0x00,//016
  0x00,0x10,0x10,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//117
  0x00,0x70,0x08,0x08,0x08,0x88,0x70,0x00,0x00,0x30,0x28,0x24,0x22,0x21,0x30,0x00,//218
  0x00,0x30,0x08,0x88,0x88,0x48,0x30,0x00,0x00,0x18,0x20,0x20,0x20,0x11,0x0E,0x00,//319
  0x00,0x00,0xC0,0x20,0x10,0xF8,0x00,0x00,0x00,0x07,0x04,0x24,0x24,0x3F,0x24,0x00,//420
  0x00,0xF8,0x08,0x88,0x88,0x08,0x08,0x00,0x00,0x19,0x21,0x20,0x20,0x11,0x0E,0x00,//521
  0x00,0xE0,0x10,0x88,0x88,0x18,0x00,0x00,0x00,0x0F,0x11,0x20,0x20,0x11,0x0E,0x00,//622
  0x00,0x38,0x08,0x08,0xC8,0x38,0x08,0x00,0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00,//723
  0x00,0x70,0x88,0x08,0x08,0x88,0x70,0x00,0x00,0x1C,0x22,0x21,0x21,0x22,0x1C,0x00,//824
  0x00,0xE0,0x10,0x08,0x08,0x10,0xE0,0x00,0x00,0x00,0x31,0x22,0x22,0x11,0x0F,0x00,//925
  0x00,0x00,0x00,0xC0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00,0x00,0x00,//:26
  0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x60,0x00,0x00,0x00,0x00,//;27
  0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x00,0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x00,//<28
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x00,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x00,//=29
  0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00,//>30
  0x00,0x70,0x48,0x08,0x08,0x08,0xF0,0x00,0x00,0x00,0x00,0x30,0x36,0x01,0x00,0x00,//?31
  0xC0,0x30,0xC8,0x28,0xE8,0x10,0xE0,0x00,0x07,0x18,0x27,0x24,0x23,0x14,0x0B,0x00,//@32
  0x00,0x00,0xC0,0x38,0xE0,0x00,0x00,0x00,0x20,0x3C,0x23,0x02,0x02,0x27,0x38,0x20,//A33
  0x08,0xF8,0x88,0x88,0x88,0x70,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x11,0x0E,0x00,//B34
  0xC0,0x30,0x08,0x08,0x08,0x08,0x38,0x00,0x07,0x18,0x20,0x20,0x20,0x10,0x08,0x00,//C35
  0x08,0xF8,0x08,0x08,0x08,0x10,0xE0,0x00,0x20,0x3F,0x20,0x20,0x20,0x10,0x0F,0x00,//D36
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x20,0x23,0x20,0x18,0x00,//E37
  0x08,0xF8,0x88,0x88,0xE8,0x08,0x10,0x00,0x20,0x3F,0x20,0x00,0x03,0x00,0x00,0x00,//F38
  0xC0,0x30,0x08,0x08,0x08,0x38,0x00,0x00,0x07,0x18,0x20,0x20,0x22,0x1E,0x02,0x00,//G39
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x20,0x3F,0x21,0x01,0x01,0x21,0x3F,0x20,//H40
  0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//I41
  0x00,0x00,0x08,0x08,0xF8,0x08,0x08,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,0x00,//J42
  0x08,0xF8,0x88,0xC0,0x28,0x18,0x08,0x00,0x20,0x3F,0x20,0x01,0x26,0x38,0x20,0x00,//K43
  0x08,0xF8,0x08,0x00,0x00,0x00,0x00,0x00,0x20,0x3F,0x20,0x20,0x20,0x20,0x30,0x00,//L44
  0x08,0xF8,0xF8,0x00,0xF8,0xF8,0x08,0x00,0x20,0x3F,0x00,0x3F,0x00,0x3F,0x20,0x00,//M45
  0x08,0xF8,0x30,0xC0,0x00,0x08,0xF8,0x08,0x20,0x3F,0x20,0x00,0x07,0x18,0x3F,0x00,//N46
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x10,0x20,0x20,0x20,0x10,0x0F,0x00,//O47
  0x08,0xF8,0x08,0x08,0x08,0x08,0xF0,0x00,0x20,0x3F,0x21,0x01,0x01,0x01,0x00,0x00,//P48
  0xE0,0x10,0x08,0x08,0x08,0x10,0xE0,0x00,0x0F,0x18,0x24,0x24,0x38,0x50,0x4F,0x00,//Q49
  0x08,0xF8,0x88,0x88,0x88,0x88,0x70,0x00,0x20,0x3F,0x20,0x00,0x03,0x0C,0x30,0x20,//R50
  0x00,0x70,0x88,0x08,0x08,0x08,0x38,0x00,0x00,0x38,0x20,0x21,0x21,0x22,0x1C,0x00,//S51
  0x18,0x08,0x08,0xF8,0x08,0x08,0x18,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//T52
  0x08,0xF8,0x08,0x00,0x00,0x08,0xF8,0x08,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//U53
  0x08,0x78,0x88,0x00,0x00,0xC8,0x38,0x08,0x00,0x00,0x07,0x38,0x0E,0x01,0x00,0x00,//V54
  0xF8,0x08,0x00,0xF8,0x00,0x08,0xF8,0x00,0x03,0x3C,0x07,0x00,0x07,0x3C,0x03,0x00,//W55
  0x08,0x18,0x68,0x80,0x80,0x68,0x18,0x08,0x20,0x30,0x2C,0x03,0x03,0x2C,0x30,0x20,//X56
  0x08,0x38,0xC8,0x00,0xC8,0x38,0x08,0x00,0x00,0x00,0x20,0x3F,0x20,0x00,0x00,0x00,//Y57
  0x10,0x08,0x08,0x08,0xC8,0x38,0x08,0x00,0x20,0x38,0x26,0x21,0x20,0x20,0x18,0x00,//Z58
  0x00,0x00,0x00,0xFE,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x7F,0x40,0x40,0x40,0x00,//[59
  0x00,0x0C,0x30,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x06,0x38,0xC0,0x00,//\60
  0x00,0x02,0x02,0x02,0xFE,0x00,0x00,0x00,0x00,0x40,0x40,0x40,0x7F,0x00,0x00,0x00,//]61
  0x00,0x00,0x04,0x02,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//^62
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,//_63
  0x00,0x02,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//`64
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x19,0x24,0x22,0x22,0x22,0x3F,0x20,//a65
  0x08,0xF8,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x3F,0x11,0x20,0x20,0x11,0x0E,0x00,//b66
  0x00,0x00,0x00,0x80,0x80,0x80,0x00,0x00,0x00,0x0E,0x11,0x20,0x20,0x20,0x11,0x00,//c67
  0x00,0x00,0x00,0x80,0x80,0x88,0xF8,0x00,0x00,0x0E,0x11,0x20,0x20,0x10,0x3F,0x20,//d68
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x22,0x22,0x22,0x22,0x13,0x00,//e69
  0x00,0x80,0x80,0xF0,0x88,0x88,0x88,0x18,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//f70
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x6B,0x94,0x94,0x94,0x93,0x60,0x00,//g71
  0x08,0xF8,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//h72
  0x00,0x80,0x98,0x98,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//i73
  0x00,0x00,0x00,0x80,0x98,0x98,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x7F,0x00,0x00,//j74
  0x08,0xF8,0x00,0x00,0x80,0x80,0x80,0x00,0x20,0x3F,0x24,0x02,0x2D,0x30,0x20,0x00,//k75
  0x00,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x00,//l76
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x20,0x3F,0x20,0x00,0x3F,0x20,0x00,0x3F,//m77
  0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x3F,0x21,0x00,0x00,0x20,0x3F,0x20,//n78
  0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x1F,0x20,0x20,0x20,0x20,0x1F,0x00,//o79
  0x80,0x80,0x00,0x80,0x80,0x00,0x00,0x00,0x80,0xFF,0xA1,0x20,0x20,0x11,0x0E,0x00,//p80
  0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x00,0x00,0x0E,0x11,0x20,0x20,0xA0,0xFF,0x80,//q81
  0x80,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x20,0x20,0x3F,0x21,0x20,0x00,0x01,0x00,//r82
  0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x33,0x24,0x24,0x24,0x24,0x19,0x00,//s83
  0x00,0x80,0x80,0xE0,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x1F,0x20,0x20,0x00,0x00,//t84
  0x80,0x80,0x00,0x00,0x00,0x80,0x80,0x00,0x00,0x1F,0x20,0x20,0x20,0x10,0x3F,0x20,//u85
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x00,0x01,0x0E,0x30,0x08,0x06,0x01,0x00,//v86
  0x80,0x80,0x00,0x80,0x00,0x80,0x80,0x80,0x0F,0x30,0x0C,0x03,0x0C,0x30,0x0F,0x00,//w87
  0x00,0x80,0x80,0x00,0x80,0x80,0x80,0x00,0x00,0x20,0x31,0x2E,0x0E,0x31,0x20,0x00,//x88
  0x80,0x80,0x80,0x00,0x00,0x80,0x80,0x80,0x80,0x81,0x8E,0x70,0x18,0x06,0x01,0x00,//y89
  0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x21,0x30,0x2C,0x22,0x21,0x30,0x00,//z90
  0x00,0x00,0x00,0x00,0x80,0x7C,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x3F,0x40,0x40,//{91
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,//|92
  0x00,0x02,0x02,0x7C,0x80,0x00,0x00,0x00,0x00,0x40,0x40,0x3F,0x00,0x00,0x00,0x00,//}93
  0x00,0x06,0x01,0x01,0x02,0x02,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//~94
  
};

/** 字库 龙邱logo 数据水平，字节垂直,字节内像素数据反序 */
const unsigned char longqiu104x64[832] = {
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X3F,0X3F,0X3F,0X3F,0X3F,0XC0,
  0XC0,0XC0,0XC0,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XC0,0XE0,0XE0,0XF0,0XF0,
  0XF0,0XF0,0X00,0X00,0X00,0X00,0X00,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,
  0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,
  0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0XF0,0X07,0X07,0X07,0X07,0X07,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0XE0,0XF0,0XF0,0XF0,0X00,0X00,0X00,0X00,0X00,0X60,
  0XE0,0XE0,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0XC0,0XE0,0XE0,0XE0,0X60,0X60,0X60,
  0X60,0X70,0X70,0X30,0X30,0X00,0XE0,0XE0,0XE0,0XE0,0X20,0X60,0XE0,0XE0,0XE0,0X40,
  0XF0,0XFE,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0XFF,
  0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,
  0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0X1F,0XFF,0XFF,0XFF,0XFF,0XFE,
  0XFE,0XFE,0XFE,0XFE,0X00,0X00,0X00,0X06,0X06,0X06,0X06,0X06,0XC6,0XFE,0XFF,0XFF,
  0X1F,0X07,0X06,0X06,0XFE,0XFE,0XFE,0X06,0X06,0X06,0X86,0XC6,0X66,0X06,0X00,0X00,
  0X00,0XFF,0XFF,0XFF,0XFF,0X30,0X30,0XF0,0XF0,0XF0,0XF0,0X30,0X30,0X00,0XFF,0XFF,
  0XFF,0XFF,0X40,0X78,0XFF,0XFF,0XFF,0X80,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,
  0X00,0XC0,0XF0,0XFE,0XFF,0X3F,0X07,0X80,0XC0,0XC0,0X60,0X70,0XFF,0XFF,0XFF,0X0E,
  0X07,0X03,0X01,0X00,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0XFF,
  0XFF,0XFF,0XFF,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0XFF,
  0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
  0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X80,0XFF,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X04,0X0F,0X0F,0X07,0X03,0X00,0X00,0X03,0X01,
  0X01,0X00,0X00,0X00,0X07,0X0F,0X0F,0X0E,0X0C,0X0C,0X0C,0X0C,0X0C,0X0C,0X00,0X00,
  0X04,0X07,0X07,0X07,0X07,0X06,0X02,0X03,0X03,0X03,0X03,0X03,0X03,0X00,0X07,0X0F,
  0X0F,0X07,0X03,0X03,0X03,0X03,0X01,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0X00,0X00,0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X42,
  0X42,0X42,0XFE,0XFE,0XFF,0XE7,0X43,0X43,0X41,0X00,0X06,0X1E,0X3E,0X3C,0X30,0X00,
  0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0X00,0X40,0X40,0XFF,0XFF,0XFF,0XFF,0X40,0X00,
  0X08,0X08,0X08,0X08,0X08,0X08,0XFF,0XFF,0XFF,0XFF,0X08,0X08,0X08,0X08,0X08,0X08,
  0X7F,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XC0,0XC0,0XC0,0XC0,0XC0,0XC1,
  0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,0XC1,
  0XC1,0XC1,0XC1,0XC1,0XC1,0X01,0X01,0X01,0X01,0X01,0X01,0XFF,0XFF,0XFF,0XFF,0XFF,
  0XFF,0XFF,0XFF,0XFF,0X00,0X00,0X00,0XFF,0XFF,0X00,0XFF,0XFF,0XFF,0X00,0XFE,0XFF,
  0X00,0X00,0X03,0X0F,0X1F,0X1E,0X18,0X00,0XFF,0XFF,0XFF,0XFF,0X80,0X80,0X00,0X00,
  0X80,0X80,0XFF,0XFF,0XFF,0XFF,0X40,0X00,0X00,0X02,0X1A,0X7A,0XFA,0XF2,0XE3,0X83,
  0X83,0XC3,0XF2,0XFE,0X7E,0X3E,0X0E,0X00,0X00,0X07,0X0F,0X1F,0X3F,0X3F,0X7F,0X7F,
  0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,
  0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X7F,0X00,0X00,0X00,
  0X00,0X00,0X00,0X7F,0X7F,0X7F,0X3F,0X3F,0X1F,0X0F,0X07,0X00,0X00,0X00,0X00,0X7F,
  0X7F,0X00,0X7F,0X7F,0X7F,0X00,0X3F,0X7F,0X00,0X00,0X03,0X01,0X01,0X01,0X01,0X01,
  0X7F,0X7F,0X7F,0X7F,0X00,0X00,0X00,0X00,0X40,0X60,0X7F,0X7F,0X7F,0X3F,0X00,0X00,
  0X00,0X40,0X60,0X70,0X78,0X7D,0X3F,0X1F,0X1F,0X3F,0X7F,0X78,0X70,0X60,0X40,0X00,
};

/*!
* @brief    简单延时函数
*
* @param    Del_1ms ：延时时间
*
* @return   无
*
* @note     无
*
* @see      lq_oled_short_delay(1);
*
* @date     2019/6/13 星期四
*/

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：lq_oled_short_delay
@功能说明：简单延时函数
@参数说明：Del ：延时时间
@函数返回：无
@修改时间：2019/6/13
@调用方法：lq_oled_short_delay(500);
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void lq_oled_short_delay(unsigned int Del)	//
{
  while(Del--)
  {
    __asm("NOP");
    
  }
}
/*!
* @brief    OLED画点命令
*
* @param    data ：要画的点
*
* @return   无
*
* @note     无
*
* @see      内部调用
*
* @date     2019/6/13 星期四
*/

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：OLED_WrDat
@功能说明：OLED画点命令
@参数说明：data ：要画的点
@函数返回：无
@修改时间：2019/6/13
@调用方法：无
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void OLED_WrDat(unsigned char data)
{
  unsigned char i=8;
  //OLED_CS=0;;
  OLED_DC_H;
  OLED_CK_L;
  lq_oled_short_delay(1);
  while(i--)
  {
    if(data&0x80){OLED_DI_H;}
    else{OLED_DI_L;}
    OLED_CK_H;
    lq_oled_short_delay(1);
    //asm("nop");            
    OLED_CK_L;
    data<<=1;    
  }
  //OLED_CS=1;
}

/*!
* @brief    OLED命令
*
* @param    cmd ：命令
*
* @return   无
*
* @note     无
*
* @see      内部调用
*
* @date     2019/6/13 星期四
*/
void OLED_WrCmd(unsigned char cmd)
{
  unsigned char i=8;
  
  //OLED_CS=0;;
  OLED_DC_L;
  OLED_CK_L;
  lq_oled_short_delay(1);
  while(i--)
  {
    if(cmd&0x80){OLED_DI_H;}
    else{OLED_DI_L;}
    OLED_CK_H;
    lq_oled_short_delay(1);
    OLED_CK_L;
    cmd<<=1;
  } 	
  //OLED_CS=1;
}

/*!
* @brief    OLED设置点
*
* @param    x ：坐标   
* @param    y ：坐标
*
* @return   无
*
* @note     左上角为0,0  右下角 127,63
*
* @see      内部调用
*
* @date     2019/6/13 星期四
*/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{ 
  OLED_WrCmd(0xb0+y);
  OLED_WrCmd(((x&0xf0)>>4)|0x10);
  OLED_WrCmd((x&0x0f));
} 

/*!
* @brief    OLED全亮
*
* @param    无   
*
* @return   无
*
* @note     无
*
* @see      OLED_Fill(); //OLED全亮 用于检测坏点
*
* @date     2019/6/13 星期四
*/
void OLED_Fill(void)
{
  unsigned char y,x;
  
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(0xff);
  }
}

/*!
* @brief    OLED清屏
*
* @param    无  
*
* @return   无
*
* @note     无
*
* @see      OLED_CLS(); //清屏
*
* @date     2019/6/13 星期四
*/
void OLED_CLS(void)
{
  unsigned char y,x;	
  for(y=0;y<8;y++)
  {
    OLED_WrCmd(0xb0+y);
    OLED_WrCmd(0x01);
    OLED_WrCmd(0x10);
    for(x=0;x<X_WIDTH;x++)
      OLED_WrDat(0);
  }
}



/*!
* @brief    画一个点
*
* @param    x  : 0-127 
* @param    y  : 0-63
*
* @return   无
*
* @note     OLED是按字节显示的 一个字节8位，代表8个纵向像素点，
* @note     因此如果画点 OLED_PutPixel(0,0); OLED_PutPixel(0,1); 只会显示点(0, 1) (0,0)则在画点(0,1)时清除掉
*
* @see      OLED_PutPixel(0, 0); //画一个点 左上角为0,0  右下角 127,63
*
* @date     2019/6/13 星期四
*/
void OLED_PutPixel(unsigned char x,unsigned char y)
{
  unsigned char data1;  //data1当前点的数据 
  
  OLED_Set_Pos(x,(unsigned char)(y>>3));
  data1 =(unsigned char)(0x01<<(y%8)); 	
  OLED_WrCmd((unsigned char)(0xb0+(y>>3)));
  OLED_WrCmd((unsigned char)(((x&0xf0)>>4)|0x10));
  OLED_WrCmd((unsigned char)((x&0x0f)|0x00));
  OLED_WrDat(data1);
}

/*!
* @brief    清除8个点
*
* @param    x  : 0-127
* @param    y  : 0-63
*
* @return   无
*
* @note     OLED是按字节显示的 一个字节8位，代表8个纵向像素点，
*
* @see      OLED_ClrDot(0, 0); //清除(0,0)-(0,7)8个点 左上角为0,0  右下角 127,63
*
* @date     2019/6/13 星期四
*/
void OLED_ClrDot(unsigned char x,unsigned char y)
{
  OLED_Set_Pos(x,(unsigned char)(y>>3));
  OLED_WrCmd((unsigned char)(0xb0+(y>>3)));
  OLED_WrCmd((unsigned char)(((x&0xf0)>>4)|0x10));
  OLED_WrCmd((unsigned char)((x&0x0f)|0x00));
  OLED_WrDat(0x00);
} 


/*!
* @brief    画矩形
*
* @param    x1  : 0-127   左上角坐标（x1,y1）
* @param    y1  : 0-63
* @param    x2  : 0-127   右下角坐标（x2，y2）
* @param    y2  : 0-63
* @param    gif : 是否开启动画效果
*
* @return   无
*
* @note     无
*
* @see      OLED_Rectangle(0, 0, 127,62,0);   //绘制矩形
*
* @date     2019/6/13 星期四
*/
void OLED_Rectangle(unsigned char x1,unsigned char y1,unsigned char x2,unsigned char y2,unsigned char gif)
{	
  unsigned char n; 
  OLED_Set_Pos(x1,y1>>3);
  for(n=x1;n<=x2;n++)
  {
    OLED_WrDat(0x01<<(y1%8));
    if(gif == 1) 	
      delay_ms(10);
  }  
  OLED_Set_Pos(x1,y2>>3);
  for(n=x1;n<=x2;n++)
  {
    OLED_WrDat(0x01<<(y2%8));
    if(gif == 1) 	
      delay_ms(10);
  }
  
  for(n = y1>>3; n <= y2>>3; n++)
  {
    OLED_Set_Pos(x1, n);
    OLED_WrDat(0xff);
    if(gif == 1) 	
      delay_ms(50);
  }
  
  for(n = y1>>3; n <= y2>>3; n++)
  {
    OLED_Set_Pos(x2, n);
    OLED_WrDat(0xff);
    if(gif == 1) 	
      delay_ms(50);
  }
}  


/*!
* @brief    写入一组标准ASCII字符串
*
* @param    x  : 0-127   
* @param    y  : 0-7
* @param    ch : 要显示的字符串
*
* @return   无
*
* @note     无
*
* @see      OLED_P6x8Str(0,0,"OLED 3.");  //显示字符串
*
* @date     2019/6/13 星期四
*/
void OLED_P6x8Str(unsigned char x,unsigned char y, char ch[])
{
  unsigned char c=0,i=0,j=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    OLED_Set_Pos(x,y);
    for(i=0;i<6;i++)     
      OLED_WrDat(F6x8[c][i]);
    x+=6;
    j++;
  }
}


/*!
* @brief    写入一组标准ASCII字符串
*
* @param    x  : 0-127   
* @param    y  : 0-6
* @param    ch : 要显示的字符串
*
* @return   无
*
* @note     一个字符高16  OLED高度64  
*
* @see      OLED_P8x16Str(0,0,"OLED 3.");  //显示字符串
*
* @date     2019/6/13 星期四
*/
void OLED_P8x16Str(unsigned char x,unsigned char y, char ch[])
{
  unsigned char c=0,i=0,j=0;
  
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>120){x=0;y++;}
    OLED_Set_Pos(x,y);
    for(i=0;i<8;i++)     
      OLED_WrDat(F8X16[c*16+i]);
    OLED_Set_Pos(x,y+1);
    for(i=0;i<8;i++)     
      OLED_WrDat(F8X16[c*16+i+8]);
    x+=8;
    j++;
  }
}

/*!
* @brief    输出汉字字符串
*
* @param    x  : 0-127   
* @param    y  : 0-6
* @param    ch : 要显示的字符串
*
* @return   无
*
* @note     一个字符高16  OLED高度64   字库需要自己制作
*
* @see      OLED_P14x16Str(0,4,"北京龙邱");   //显示汉字
*
* @date     2019/6/13 星期四
*/
void OLED_P14x16Str(unsigned char x,unsigned char y, unsigned char ch[])
{
  unsigned char wm=0,ii = 0;
  unsigned int adder=1; 
  
  while(ch[ii] != '\0')
  {
    wm = 0;
    adder = 1;
    while(F14x16_Idx[wm] > 127)
    {
      if(F14x16_Idx[wm] == (unsigned char)ch[ii])
      {
        if(F14x16_Idx[wm + 1] == ch[ii + 1])
        {
          adder = wm * 14;
          break;
        }
      }
      wm += 2;			
    }
    if(x>118){x=0;y++;}
    OLED_Set_Pos(x , y);
    if(adder != 1)// 显示汉字					
    {
      OLED_Set_Pos(x , y);
      for(wm = 0;wm < 14;wm++)               
      {
        OLED_WrDat(F14x16[adder]);
        adder += 1;
      }      
      OLED_Set_Pos(x,y + 1);
      for(wm = 0;wm < 14;wm++)          
      {
        OLED_WrDat(F14x16[adder]);
        adder += 1;
      }   		
    }
    else			  //显示空白字符			
    {
      ii += 1;
      OLED_Set_Pos(x,y);
      for(wm = 0;wm < 16;wm++)
      {
        OLED_WrDat(0);
      }
      OLED_Set_Pos(x,y + 1);
      for(wm = 0;wm < 16;wm++)
      {   		
        OLED_WrDat(0);
      }
    }
    x += 14;
    ii += 2;
  }
}



/*!
* @brief    显示BMP图片 最大 128×64
*
* @param    x0  : 0-127   左上角坐标（x0,y0）
* @param    y0  : 0-63
* @param    wide: 1-127   图片宽度
* @param    high: 1-64    图片高度
* @param    bmp : 要显示的图片
*
* @return   无
*
* @note     图片需要用软件转化
*
* @see      OLED_Show_BMP(0, 0, 104, 64, longqiu104x64); //画龙邱loge
*
* @date     2019/6/13 星期四
*/
void OLED_Show_BMP(unsigned char x0,unsigned char y0,unsigned char wide,unsigned char high,const unsigned char * bmp)
{ 	
  unsigned int ii=0;
  unsigned char x,y;
  unsigned char x1,y1;
  if(wide + x0 > 128 || high + y0 > 64)
  {
    return;  //超出显示范围
  }
  y1 = (y0+high-1)/8;
  x1 = x0 + wide;
  for(y=y0/8;y<=y1;y++)
  {
    OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++)
    {      
      OLED_WrDat(bmp[ii++]);
    }
  }
}

/*!
* @brief    显示龙邱图片128×64
*
* @param    无 
*
* @return   无
*
* @note     无
*
* @see      OLED_Show_LQLogo();
*
* @date     2019/6/13 星期四
*/
void OLED_Show_LQLogo(void)
{ 	
  unsigned int ii=0;
  unsigned char x,y;       
  
  for(y=0;y<8;y++)
  {
    OLED_Set_Pos(16,y);
    for(x=14;x<118;x++)
    {      
      OLED_WrDat(longqiu104x64[ii++]);
    }
  }
}




/*!
* @brief    画边线 OV7725显示图像时 可以用这个画边线
*
* @param    无 
*
* @return   无
*
* @note     无
*
* @see      OLED_Show_Frame80();
*
* @date     2019/6/13 星期四
*/
void OLED_Show_Frame80(void)
{ 	
  
  OLED_Set_Pos(23,1);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(23,2);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(23,3);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(23,4);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(23,5);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(23,6);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(23,7);
  OLED_WrDat(0xFF);
  
  OLED_Set_Pos(104,1);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(104,2);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(104,3);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(104,4);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(104,5);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(104,6);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(104,7);
  OLED_WrDat(0xFF);
}

/*!
* @brief    画边线 MT9V034显示图像时 可以用这个画边线
*
* @param    无 
*
* @return   无
*
* @note     无
*
* @see      OLED_Show_Frame94();
*
* @date     2019/6/13 星期四
*/
void OLED_Show_Frame94(void)
{ 	
  //OLED_Set_Pos(13,0);//第0行，第13列
  //OLED_WrDat(0xFF); //画竖线
  OLED_Set_Pos(16,1);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(16,2);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(16,3);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(16,4);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(16,5);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(16,6);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(16,7);
  OLED_WrDat(0xFF);
  
  //OLED_Set_Pos(114,0);
  //OLED_WrDat(0xFF);
  OLED_Set_Pos(111,1);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(111,2);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(111,3);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(111,4);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(111,5);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(111,6);
  OLED_WrDat(0xFF);
  OLED_Set_Pos(111,7);
  OLED_WrDat(0xFF);
}

/**
* @brief    OLED 二值化图像显示
*
* @param    high ： 显示图像高度
* @param    wide ： 显示图像宽度
* @param    Pixle： 显示图像数据地址
*
* @return   无
*
* @note     无
*
* @see      OLED_Road(OLEDH, OLEDW, (unsigned char *)Pixle);
*
* @date     2019/6/25 星期二
*/
void OLED_Road(unsigned short high, unsigned short wide, unsigned char *Pixle)
{ 	 
  unsigned char i = 0, j = 0,temp=0;
  unsigned char wide_start = (128 - wide)/2;
  for(i=0;i<high-7;i+=8)
  {
    
    OLED_Set_Pos(wide_start,i/8+1);
    
    for(j=0;j<wide;j++) 
    { 
      temp=0;
      if(Pixle[(0+i)*wide + j]) temp|=1;
      if(Pixle[(1+i)*wide + j]) temp|=2;
      if(Pixle[(2+i)*wide + j]) temp|=4;
      if(Pixle[(3+i)*wide + j]) temp|=8;
      if(Pixle[(4+i)*wide + j]) temp|=0x10;
      if(Pixle[(5+i)*wide + j]) temp|=0x20;
      if(Pixle[(6+i)*wide + j]) temp|=0x40;
      if(Pixle[(7+i)*wide + j]) temp|=0x80;
      OLED_WrDat(temp);
    }
  }  
}



/**
* @brief    OLED 测试函数
*
* @param    无
*
* @return   无
*
* @note     无
*
* @example  Test_OLED();
*
* @date     2019/6/25 星期二
*/
void Test_OLED(void)
{      
	int num=0;
	uart_init(USART_2, 115200);			//初始化串口
	LED_Init();							//初始化LED	
    OLED_Init();                          //OLED初始化
	printf("OLED Init is ok\n");
    OLED_Show_LQLogo();                   //显示LOGO
    delay_ms(2000);
    OLED_CLS();
  
  while (1)
  {  
    printf("num:%d",num);
	LED_Ctrl(RVS);
    OLED_P6x8Str(0,7,"OLED 3.");         //字符串
    delay_ms(1000);
    OLED_Rectangle(0, 0, 127,62,0);      //绘制矩形
    OLED_Rectangle(24,8,88,55,1);
    OLED_Rectangle(32,16,80,47,1);
    OLED_Rectangle(40,24,72,39,1);
    delay_ms(1000);
    OLED_CLS();                          //清屏
    
    OLED_P8x16Str(0, 0,"OLED 4.");       //显示字符串
    OLED_P14x16Str(0,4,(unsigned char*)"北京龙邱");      //显示汉字
    OLED_PutPixel(120,60);
    delay_ms(1000);
    OLED_CLS();                          //清屏
    
    OLED_Fill();                         //填充
    delay_ms(1000);
    OLED_CLS();                          //清屏
    
    //延时
    delay_ms(50);
  }
}


	 
#include "include.h"

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� ������2
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//ѭ������,ֱ���������   
  USART2->DR = (u8) ch;      
	return ch;
}
#endif 



//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  

u8 aRxBuffer[RXBUFFERSIZE];//HAL��ʹ�õĴ��ڽ��ջ���
UART_HandleTypeDef USART2_Handler; //USART2���
UART_HandleTypeDef USART3_Handler; //USART3���

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void uart_init(USART_NUM usart,u32 bound)		
@����˵�������ڳ�ʼ��
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�������
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void uart_init(USART_NUM usart,u32 bound)				//��ʼ������
{
	switch(usart)
	{
		case 2:
			USART2_Handler.Instance=USART2;                         //USART2
			USART2_Handler.Init.BaudRate=bound;	                    //������
			USART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;      //�ֳ�Ϊ8λ���ݸ�ʽ
			USART2_Handler.Init.StopBits=UART_STOPBITS_1;           //һ��ֹͣλ
			USART2_Handler.Init.Parity=UART_PARITY_NONE;            //����żУ��λ
			USART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;      //��Ӳ������
			USART2_Handler.Init.Mode=UART_MODE_TX_RX;               //�շ�ģʽ
			HAL_UART_Init(&USART2_Handler);                         //HAL_UART_Init()��ʹ��UART3
			HAL_UART_Receive_IT(&USART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
		
		case 3:
			USART3_Handler.Instance=USART3;                         //USART3
			USART3_Handler.Init.BaudRate=bound;                     //������
			USART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;      //�ֳ�Ϊ8λ���ݸ�ʽ
			USART3_Handler.Init.StopBits=UART_STOPBITS_1;	    	//һ��ֹͣλ
			USART3_Handler.Init.Parity=UART_PARITY_NONE;            //����żУ��λ
			USART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;      //��Ӳ������
			USART3_Handler.Init.Mode=UART_MODE_TX_RX;               //�շ�ģʽ
			HAL_UART_Init(&USART3_Handler);	                        //HAL_UART_Init()��ʹ��UART3
			HAL_UART_Receive_IT(&USART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
		break;
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART2)//����Ǵ���3�����д���3 MSP��ʼ��
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();					//ʹ��GPIOAʱ��
		__HAL_RCC_USART2_CLK_ENABLE();				//ʹ��USART3ʱ��
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_2;					//PA2
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;				//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//��ʼ��A2

		GPIO_Initure.Pin=GPIO_PIN_3;					//PA3
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//ģʽҪ����Ϊ��������ģʽ��	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	  //��ʼ��A3
		

		HAL_NVIC_EnableIRQ(USART2_IRQn);			//ʹ��USART2�ж�ͨ��
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);//��ռ���ȼ�3�������ȼ�3

	}
	
	if(huart->Instance==USART3)//����Ǵ���3�����д���3 MSP��ʼ��
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//ʹ��GPIOAʱ��
		__HAL_RCC_USART3_CLK_ENABLE();			//ʹ��USART3ʱ��
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//�����������
		GPIO_Initure.Pull=GPIO_PULLUP;			//����
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//ģʽҪ����Ϊ��������ģʽ��	
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//��ʼ��PB11

		HAL_NVIC_EnableIRQ(USART3_IRQn);				//ʹ��USART3�ж�ͨ��
		HAL_NVIC_SetPriority(USART3_IRQn,3,2);			//��ռ���ȼ�3�������ȼ�2

	}
}

u8 ReadBuff[1024];
u16 num2=0, num3=0;
u8 receive_oneline2=0;
u8 receive_oneline3=0;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
@����˵�������ڻص�����
@����˵����*huart (���ھ��)
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�������
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)//����Ǵ���2
	{
		if(HAL_UART_Receive_IT(&USART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) == HAL_OK)//���ճɹ�
		{
			if(aRxBuffer[0] == '\n')// || aRxBuffer[0] == '\r')  //�Իس�Ϊ������
			{
				
				ReadBuff[num2] = '\0';
				num2=0;
                receive_oneline2 = 1;
			}else{
				ReadBuff[num2++] = aRxBuffer[0];
			}
		}else{
			printf("receive faild\n");
		}
	}
	if(huart->Instance==USART3)//����Ǵ���3
	{
		if(HAL_UART_Receive_IT(&USART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) == HAL_OK)//���ճɹ�
		{

//            uart_SendBuf(&USART3_Handler, aRxBuffer);
            
			if(aRxBuffer[0] == '\n')// || aRxBuffer[0] == '\r')//�Իس�Ϊ������
			{
				ReadBuff[num3] = '\0'; //����β�ӡ�\0��
                num3=0;
                receive_oneline3 = 1;  //������ɱ�־ 1
				
			}else{
				ReadBuff[num3++] = aRxBuffer[0];
			}
            
            
		}else{
			printf("receive faild\n");
		}

	}
}
 
//����2�жϷ������
void USART2_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&USART2_Handler);	//����HAL���жϴ����ú���-----�ú���������жϱ�־��ȡ���ж�ʹ�ܣ�����ӵ��ûص�����
} 

//����3�жϷ������
void USART3_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&USART3_Handler);	//����HAL���жϴ����ú���-----�ú���������жϱ�־��ȡ���ж�ʹ�ܣ�����ӵ��ûص�����
} 


void uart_SendBuf(UART_HandleTypeDef *huart, uint8_t *pData)
{
//	while(len--)
//	{
		if(huart->Instance==USART2)
		{
			HAL_UART_Transmit(&USART2_Handler, pData, strlen((const char*)pData), 100);
		}
		if(huart->Instance==USART3)
		{
			HAL_UART_Transmit(&USART3_Handler, pData, strlen((const char*)pData), 100);
		}
	//}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@�������ƣ�void Test_Usart(void)
@����˵��������ģ����Ժ���(����3)
@����˵������
@�������أ���
@�޸�ʱ�䣺2022/02/24
@���÷�����Test_Usart3()
@��    ע����
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_Usart(void)
{	
	uart_init(USART_3, 115200);			//��ʼ������    
	uart_init(USART_2, 115200);			//��ʼ������    �����ӿ�
    OLED_Init();
	LED_Init();							//��ʼ��LED	
	printf("Long Qiu Ke Ji\n");
	printf("USART3 test,This is USART2.\n");
    uart_SendBuf(&USART3_Handler, "This is USART3!\n");
    OLED_P6x8Str(0,3,"Long Qiu Ke Ji");         //�ַ���
    OLED_P6x8Str(0,5,"UART Test");              //�ַ���
	while(1)
	{
		if(receive_oneline2 != 0)//һ�н��ճɹ�
		{
			printf("����receive %d byte data is %s\n",receive_oneline2, ReadBuff);
			receive_oneline2=0;
		}
		if(receive_oneline3 != 0)
		{
			uart_SendBuf(&USART3_Handler, (uint8_t*)ReadBuff);
			receive_oneline3=0;
		}
		delay_ms(200);   
		LED_Ctrl(RVS); 
	}
}


#include "usart2.h"

u16 rx_buffer[2][12];
int buff[20];
u16 USART_RX_STA=0;       //接收状态标记
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u8 Uart2_Buffer[256]; 
u8 Uart2_Rx=0;

u8 Uart2_head1;
u8 Uart2_head2;          
u8 Uart2_Len;  
u8 Uart2_Sta;
u8 Uart2_tx2;

unsigned char Getdata[16];


void USART2_Init(uint32_t baudrate)	
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef usart2;
	
	//结构体初始化
	NVIC_InitTypeDef nvic;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_DeInit(USART2);
	USART_StructInit(&usart2);
	//USART_GetFlagStatus(USART2,USART_FLAG_TC);
	usart2.USART_BaudRate = baudrate;
	usart2.USART_WordLength = USART_WordLength_8b;
	usart2.USART_StopBits = USART_StopBits_1;
	usart2.USART_Parity = USART_Parity_No;
	usart2.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&usart2);
	
	USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART2,ENABLE);
	USART_GetFlagStatus(USART2,USART_FLAG_TC);

	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;	//2
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
}

u8 Res;
void USART2_IRQHandler(void)                   
{

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  
  {
        //USART_ClearITPendingBit(USART1,USART_IT_RXNE);
     Res =USART_ReceiveData(USART2);//(USART1->DR);    
        //USART_RX_BUF[buf_index++]=Res;
         
  } 

}


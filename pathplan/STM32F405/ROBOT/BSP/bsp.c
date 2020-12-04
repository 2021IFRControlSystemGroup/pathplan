#include "bsp.h"

void BSP_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//设置系统中断优先级分组1
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2 
	USART2_Init(115200);	//串口初始化波特率为115200
	
//	delay_init(168);	
//	chassis_PID_Init();//底盘pid初始化
//	Yun_PID_Init();
//	LED_Init();
//	USART1_DMA_Config(10000);//遥控器初始化
//	chassis_PID_Init();
//	TIM5_Int_Init(100-1,84-1)
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN1初始化
//	delay_ms(1000);
}



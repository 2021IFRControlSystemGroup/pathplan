#include "bsp.h"

void BSP_Init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//����ϵͳ�ж����ȼ�����1
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2 
	USART2_Init(115200);	//���ڳ�ʼ��������Ϊ115200
	
//	delay_init(168);	
//	chassis_PID_Init();//����pid��ʼ��
//	Yun_PID_Init();
//	LED_Init();
//	USART1_DMA_Config(10000);//ң������ʼ��
//	chassis_PID_Init();
//	TIM5_Int_Init(100-1,84-1)
//	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN1��ʼ��
//	delay_ms(1000);
}



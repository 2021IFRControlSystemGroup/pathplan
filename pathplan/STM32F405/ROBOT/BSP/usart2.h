#ifndef __USART2_H
#define __USART2_H

#include "sys.h"

#define  PR_FRAME_LENGTH      17


#define USART_REC_LEN  			200  	//�����������ֽ��� 200
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 


void USART2_Init(uint32_t baudrate);
#endif


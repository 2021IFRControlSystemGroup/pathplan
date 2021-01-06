#ifndef __USART2_H
#define __USART2_H

#include "sys.h"

#define  PR_FRAME_LENGTH      17


#define USART_REC_LEN  			200  	//定义最大接收字节数 200
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 


void USART2_Init(uint32_t baudrate);
#endif


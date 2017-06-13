#ifndef _USART_H
#define _USART_H
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
void Usart1_Init(int br_num);
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
void Uart1_Put_String(unsigned char *Str);
#endif

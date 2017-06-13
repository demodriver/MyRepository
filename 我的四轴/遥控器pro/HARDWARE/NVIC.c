#include "NVIC.h"

void NVIC_INIT(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	NVIC_InitStruct.NVIC_IRQChannel= TIM3_IRQn;		    //时钟3
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;  //抢占优先级2
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;		  //响应优先级1
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;			  //IRQ通道使能
	NVIC_Init(&NVIC_InitStruct);						  //中断初始化	
	//串口
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	//外部中断
	NVIC_InitStruct.NVIC_IRQChannel=EXTI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
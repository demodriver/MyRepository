#ifndef _TIME_H
#define _TIME_H
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

extern u32 TIM3_IRQCNT;
void Tim3_Control(u8 sta);
void time3_Init(u16 period_num);
#endif

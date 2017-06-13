#ifndef _MOTO_H
#define _MOTO_H
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
extern vs16 Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4; //µç»úÊä³ö
void MotoReFlash(int16_t Moto_PWM_1,int16_t Moto_PWM_2,int16_t Moto_PWM_3,int16_t Moto_PWM_4);
void Moto_Init(void);
void Time4_Init(void);
#endif

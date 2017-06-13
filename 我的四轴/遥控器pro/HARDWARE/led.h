#ifndef _LED_H
#define _LED_H
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#define LED1_ON   GPIOA->BRR = GPIO_Pin_8;      //低电平导通
#define LED1_OFF   GPIOA->BSRR = GPIO_Pin_8;  
#define LED2_ON   GPIOB->BRR = GPIO_Pin_12;      //低电平导通
#define LED2_OFF   GPIOB->BSRR = GPIO_Pin_12;   

void LED_INIT(void);
void LED_FLASH(void);
void LED1_ONOFF(void);
void LED2_ONOFF(void);
#endif

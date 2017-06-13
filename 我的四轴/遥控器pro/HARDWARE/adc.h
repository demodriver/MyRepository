#ifndef _ADC_H
#define _ADC_H
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

extern u16 ADC_Temp[10];
void DMA_INIT(void);
void ADC_INIT(void);
void Get_Adc(void);
#endif


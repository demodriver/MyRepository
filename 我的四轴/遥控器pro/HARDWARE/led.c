#include "led.h"
#include "time.h"

void Delay_ms_led(u16 nms)
{	
	uint16_t i,j;
	for(i=0;i<nms;i++)
		for(j=0;j<8500;j++);
} 

void LED_INIT(void)
{
	GPIO_InitTypeDef GPIO_Structure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//设置LED使用到得管脚
	GPIO_Structure.GPIO_Pin =  GPIO_Pin_12;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOB, &GPIO_Structure);
	
	GPIO_Structure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(GPIOA, &GPIO_Structure);
	
}


void LED_FLASH(void)
{
	LED1_ON;
	LED2_ON;
	Delay_ms_led(100);
	LED1_OFF;
	LED2_OFF;
	Delay_ms_led(100);
	LED1_ON;
	LED2_ON;
	Delay_ms_led(100);
	LED1_OFF;
	LED2_OFF;
	Delay_ms_led(100);
	LED1_ON;
	LED2_ON;
	Delay_ms_led(100);
	LED1_OFF;
	LED2_OFF;
	Delay_ms_led(100);
	LED1_ON;
	LED2_ON;
	Delay_ms_led(100);
	LED1_OFF;
	LED2_OFF;
	Delay_ms_led(100);
	LED1_ON;
	LED2_ON;
	Delay_ms_led(100);
	LED1_OFF;
	LED2_OFF;
	Delay_ms_led(100);
}
void LED1_ONOFF(void)
{
	static uint8_t busy=0;
	static uint8_t led1_sta=1;
	static uint32_t time_temp;
	if(led1_sta)
	{
		if(!busy)
		{
			time_temp=TIM3_IRQCNT;
			busy=1;
		}
		else if((time_temp+1500)<TIM3_IRQCNT)//200 time delay
		{
			led1_sta=0;
			LED1_OFF;
			busy=0;
		}
	}
	else
	{
		if(!busy)
		{
			time_temp=TIM3_IRQCNT;
			busy=1;
		}
		else if((time_temp+1500)<TIM3_IRQCNT)
		{
			led1_sta=1;
			LED1_ON;
			busy=0;
		}
	}
}

void LED2_ONOFF(void)
{
	static uint8_t busy=0;
	static uint8_t led2_sta=1;
	static uint32_t time_temp;
	if(led2_sta)
	{
		if(!busy)
		{
			time_temp=TIM3_IRQCNT;
			busy=1;
		}
		else if((time_temp+1500)<TIM3_IRQCNT)//200 time delay
		{
			led2_sta=0;
			LED2_OFF;
			busy=0;
		}
	}
	else
	{
		if(!busy)
		{
			time_temp=TIM3_IRQCNT;
			busy=1;
		}
		else if((time_temp+1500)<TIM3_IRQCNT)
		{
			led2_sta=1;
			LED2_ON;
			busy=0;
		}
	}
}


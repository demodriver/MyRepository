#include "stm32f10x.h"
#include "nrf24l01.h"
#include "Datatansfer.h"
#include "usart.h"
#include "time.h"
#include "adc.h"
#include "NVIC.h"
u8 FLAG_ATT;
int main()
{
 	u16 i=1000;
	NVIC_INIT();
	//Usart1_Init(9600);
	ADC_INIT();
	DMA_INIT();
	time3_Init(500);
	spi2_Init();
	while(i--);
	Nrf24l01_Init(2,40);
	while(i--);
	Tim3_Control(1);
	while(1)
	{	
		if(FLAG_ATT==1)
		{
			FLAG_ATT=0;
			Rc_Data.THROTTLE=ADC_Temp[1];
			Rc_Data.ROL=ADC_Temp[2];
			Rc_Data.YAW=ADC_Temp[3];
			Rc_Data.PIT=ADC_Temp[4];
			Rc_Data.AUX1=ADC_Temp[5];
			Rc_Data.AUX2=ADC_Temp[6];
		}
		TX_Funcation();
	}
}


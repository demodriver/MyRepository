#include "stm32f10x.h"
#include "spi.h"
#include "attitude1_calculate.h"
#include "Control.h"
#include "time.h"
#include "usart.h"
#include "Datatansfer.h"
#include "Moto.h"
#include "Rc.h"
#include "led.h"
#include "Nrf24l01.h"

MPU_value Final_senser; //
MPU_value last_value;//上一次测量值	

u8 FLAG_ATT=0;//1ms标志位

void System_Init(void)
{
		LED_INIT();
		spi_Init();
		spi2_Init();
		Init_MPU9250();
		NVIC_INIT();
		time3_Init(500);
	  Usart1_Init(115200);
		Moto_Init();		
		MPU9250_CalOff_Gyr();
		Nrf24l01_Init(1,40);
  	if(Nrf24l01_Check())
		Uart1_Put_String("NRF24L01 IS OK !\r\n");
		else 									
	 Uart1_Put_String("NRF24L01 IS NOT OK !\r\n");
		
		LED_FLASH(); //5次闪烁表示初始化完成
		Time4_Init();
		Tim3_Control(1); //时钟3使能
	}

int main()
{
	static u8 att_cnt=0,rc_cnt=0,senser_cnt=0,
		status_cnt=0,dt_rc_cnt=0,dt_moto_cnt=0;//姿态、传感器、标志位
	
	System_Init();
	
	NRF_CE_L;
	NRF_Write_Reg(FLUSH_RX,0xff); 
	NRF_Write_Reg(FLUSH_TX,0xff); 
	NRF_CE_H;
	
	
 	while(1)
	{		
 		if(FLAG_ATT)
		{
			FLAG_ATT=0; 
			att_cnt++;
			rc_cnt++;
			
			LED1_ONOFF(); //led闪烁表示飞行正常
			LED2_ONOFF(); 
			
			if(rc_cnt==20)    //每20ms读取遥控数据
			{
				rc_cnt = 0;
				#ifdef CONTROL_USE_RC
				Rc_GetValue(&Rc_Data); 
				#endif
				Rc_Fun(&Rc_Data,&Rc_Control);
			}
			
			if(att_cnt==1)           //每2ms解算姿态
			{
				last_value=current_value;
			}	
			else
			{
				att_cnt=0;													
				attitude_calculate(current_value);              //姿态结算	
				Control(&Q_ANGLE,&Rc_Data,Rc_Control,&Final_senser);          //控制			
			}
			
				senser_cnt++;
				status_cnt++;
				dt_rc_cnt++;
				dt_moto_cnt++;

				if(senser_cnt==3)  //每3ms上传一次传感器数据
				{
					senser_cnt = 0;
					Send_Senser = 1;
				}
				if(status_cnt==5) //每5ms上传一次姿态数据
				{
					status_cnt = 0;
					Send_Status = 1;
				}
				if(dt_rc_cnt==9)  //每10ms上传一次遥控数据
				{
					dt_rc_cnt=0;
					Send_RCData = 1;
				}
				if(dt_moto_cnt==7) //每7ms上传一次电机数据
				{
					dt_moto_cnt=0;
					Send_MotoPwm = 1;
				}			
		}
	}
	
}

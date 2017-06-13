#ifndef _DATATRANSFER_H
#define _DATATRANSFER_H
#include "stm32f10x.h"

//#define DATA_TRANSFER_USE_USART   
#define DATA_TRANSFER_USE_SPI_NRF  

typedef struct {
		int16_t ROL;
  	int16_t PIT;
		int16_t THROTTLE;
		int16_t YAW;
		int16_t AUX1;
		int16_t AUX2;
		int16_t AUX3;
		int16_t AUX4;
		int16_t AUX5;
		int16_t AUX6;
}T_RC_Data;
extern T_RC_Data  Rc_Data;
extern u8 Send_RCData; 
extern u8 data_to_send[35];
void Data_Receive_Anl(u8 *data_buf,u8 len);
void Data_Exchange(void);
//void Data_Send_Status(void);
//void Data_Send_Senser(void);
void Data_Send_RCData(void);
//void Data_Send_MotoPWM(void);
//void Data_Send_OFFSET(void);
//void Data_Send_PID1(void);
//void Data_Send_Check(u16 check);
//////////////////////////////////////
u8 Nrf_Check_Event(void);
u8 Nrf_Get_FIFOSta(void);

#endif

#ifndef _DATATRANSFER_H
#define _DATATRANSFER_H
#include "stm32f10x.h"

#define DATA_TRANSFER_USE_USART   
#define DATA_TRANSFER_USE_SPI_NRF  
#define CONTROL_USE_RC


extern u8 Data_Check,Send_Status,Send_Senser,Send_RCData,Send_GpsData,Send_Offset,Send_PID1,Send_PID2,Send_PID3,Send_MotoPwm;
void Data_Receive_Anl(u8 *data_buf,u8 len);
void Data_Exchange(void);
void Data_Send_Status(void);
void Data_Send_Senser(void);
void Data_Send_RCData(void);
void Data_Send_MotoPWM(void);
void Data_Send_OFFSET(void);
void Data_Send_PID1(void);
void Data_Send_Check(u16 check);
//////////////////////////////////////
void Nrf_Check_Event(void);
u8 Nrf_Get_FIFOSta(void);

#endif

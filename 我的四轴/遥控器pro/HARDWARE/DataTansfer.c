#include "Datatansfer.h"
#include "usart.h"
#include "Nrf24L01.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
T_RC_Data  Rc_Data; //遥控通道缓存
u8 Send_RCData;
u8 data_to_send[35];

//***********************************************************//
//NRF事件检查
//***********************************************************//
u8 Nrf_Check_Event(void)
{
	u8 flag=0;
//*************************************************************//
	u8 rx_len=0,sta;
	sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);   //读取状态寄存器
	//NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);  //清空标志位
	
//	if(sta & (1<<RX_DR))                                  //接收中断
//	{
//		rx_len = NRF_Read_Reg(R_RX_PL_WID);              //数据长度读取
//		if(rx_len<33)
//		{
//			NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len); // read receive payload from RX_FIFO buffer
//			Data_Receive_Anl(NRF24L01_RXDATA,rx_len);         //数据分析
//		}
//		else 
//		{
//			NRF_Write_Reg(FLUSH_RX,0xff);                     //清空缓冲区
//		}
//	}
//*************************************************************//
	if(sta & (1<<TX_DS))                                   //发送中断
	{ 
		flag=1;                                           //发送成功
	}
	////////////////////////////////////////////////////////////////
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	                                     //TX FIFO FULL
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);//
	sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);
	return flag;
}
//*****************NRF FIFO status***************************//
u8 Nrf_Get_FIFOSta(void)
{
	return NRF_Read_Reg(NRF_READ_REG + FIFO_STATUS);
}
//***********************************************//
//数据接收分析
//***********************************************//
void Data_Receive_Anl(u8 *data_buf,u8 len)
{
	u8 sum = 0,i;
	
	for(i=0;i<(len-1);i++)
	{
		sum += *(data_buf+i);
	}		
	if(!(sum==*(data_buf+len-1)))		return;		     //判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAA))		return;		//判断帧头
//////////////////////////////////////////////////////////////////
//	if(*(data_buf+2)==0X03)
//	{
//		Rc_send_Data.THROTTLE = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
//		Rc_send_Data.YAW = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
//		Rc_send_Data.ROL = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
//		Rc_send_Data.PIT = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
//		Rc_send_Data.AUX1 = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
//		Rc_send_Data.AUX2 = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
//		Rc_send_Data.AUX3 = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
//		Rc_send_Data.AUX4 = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
//		Rc_send_Data.AUX5 = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
//		Rc_send_Data.AUX6 = (vs16)(*(data_buf+21)<<8)|*(data_buf+22);
		//Rc_Fun(&Rc_Data,&Rc_Control);
//	}
//	if(*(data_buf+2)==0X10)								//更改PID
//	{
//			PID_ROL.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_ROL.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
//			PID_ROL.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//		
//			PID_PIT.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_PIT.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
//			PID_PIT.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//		
//			PID_YAW.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_YAW.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
//			PID_YAW.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
//			Data_Send_Check(sum);  //返回相同数据
//	}
//	if(*(data_buf+2)==0X11)								//PID2
//	{
//			PID_ALT.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_ALT.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_ALT.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			PID_POS.P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
//			PID_POS.I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/100;
//			PID_POS.D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
//			PID_PID_1.P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
//			PID_PID_1.I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/100;
//			PID_PID_1.D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
//			Data_Send_Check(sum);
//	}
//	if(*(data_buf+2)==0X12)								//PID3
//	{
//			PID_PID_2.P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
//			PID_PID_2.I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/100;
//			PID_PID_2.D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
//			Data_Send_Check(sum);
//			EE_SAVE_PID();
//	if(*(data_buf+2)==0X16)								//OFFSET
//	{
//			offset_value. = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
//			offset_value = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
//	}
//	
}
//***********************************************//
//数据交换
//***********************************************//
void Data_Exchange(void)
{	
	u8 sta,flag=0; 
	flag=Nrf_Check_Event();             //NRF事件检查 
	sta = Nrf_Get_FIFOSta(); 
	//if((sta &(1<<MAX_RT))==0) return;
	if(Send_RCData==1)
	{
		Send_RCData = 0;
		Data_Send_RCData(); //遥控数据	
	}
}
//****************************************************//
//飞控to上位机
//****************************************************//
//void Data_Send_Status(void)
//{
//	u8 _cnt=0;
//	u8 sum = 0,i;
//	vs16 _temp;
//	data_to_send[_cnt++]=0xAA;//帧头
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x01;//功能字
//	data_to_send[_cnt++]=0;

//	_temp = (int)(Q_ANGLE.ROL*100); //拆分数据
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(Q_ANGLE.PIT*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = (int)(Q_ANGLE.YAW*100);
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
////	vs32 _temp2 = Alt;
////	data_to_send[_cnt++]=BYTE3(_temp2);
////	data_to_send[_cnt++]=BYTE2(_temp2);
////	data_to_send[_cnt++]=BYTE1(_temp2);
////	data_to_send[_cnt++]=BYTE0(_temp2);
///////////////////////////////////////////////////////////////////		
//	if(Rc_Control.ARMED==0)			data_to_send[_cnt++]=0xA0;	//解锁状态发送
//	else if(Rc_Control.ARMED==1)		data_to_send[_cnt++]=0xA1;
//////////////////////////////////////////////////////////////////	
//	data_to_send[3] = _cnt-4;//数据长度
//	
//	
//	for(i=0;i<_cnt;i++) //校验和
//		sum += data_to_send[i];
//	data_to_send[_cnt++]=sum;
////*******************************************//
////数据发送选择(suart & NRF)
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt); 
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
////////////////////////////////////////////
//void Data_Send_Senser(void)
//{
//	u8 _cnt=0;
//	u8 sum = 0,i;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x02;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=BYTE1(Final_senser.Accel[0]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Accel[0]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Accel[1]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Accel[1]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Accel[2]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Accel[2]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Gyro[0]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Gyro[0]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Gyro[1]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Gyro[1]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Gyro[2]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Gyro[2]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Mag[0]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Mag[0]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Mag[1]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Mag[1]);
//	data_to_send[_cnt++]=BYTE1(Final_senser.Mag[2]);
//	data_to_send[_cnt++]=BYTE0(Final_senser.Mag[2]);
//	
//	data_to_send[3] = _cnt-4;
//	
//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	data_to_send[_cnt++] = sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
//////////////////////////////////////////
void Data_Send_RCData(void)
{
	u8 _cnt=0;
	u8 sum = 0,i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Rc_Data.THROTTLE);
	data_to_send[_cnt++]=BYTE0(Rc_Data.THROTTLE);
	data_to_send[_cnt++]=BYTE1(Rc_Data.YAW);
	data_to_send[_cnt++]=BYTE0(Rc_Data.YAW);
	data_to_send[_cnt++]=BYTE1(Rc_Data.ROL);
	data_to_send[_cnt++]=BYTE0(Rc_Data.ROL);
	data_to_send[_cnt++]=BYTE1(Rc_Data.PIT);
	data_to_send[_cnt++]=BYTE0(Rc_Data.PIT);
	data_to_send[_cnt++]=BYTE1(Rc_Data.AUX1);
	data_to_send[_cnt++]=BYTE0(Rc_Data.AUX1);
	data_to_send[_cnt++]=BYTE1(Rc_Data.AUX2);
	data_to_send[_cnt++]=BYTE0(Rc_Data.AUX2);
//	data_to_send[_cnt++]=0;//BYTE1(Rc_Data.AUX3);
//	data_to_send[_cnt++]=0;//BYTE0(Rc_Data.AUX3);
//	data_to_send[_cnt++]=0;//BYTE1(Rc_Data.AUX4);
//	data_to_send[_cnt++]=0;//BYTE0(Rc_Data.AUX4);
//	data_to_send[_cnt++]=0;//BYTE1(Rc_Data.AUX5);
//	data_to_send[_cnt++]=0;//BYTE0(Rc_Data.AUX5);
//	data_to_send[_cnt++]=0;//BYTE1(Rc_Data.AUX6);
//	data_to_send[_cnt++]=0;//BYTE0(Rc_Data.AUX6);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(data_to_send,_cnt);
#else
	
	//NRF_TxPacket(data_to_send,_cnt);
#endif
}
///////////////////////////////////////////////
//void Data_Send_MotoPWM(void)
//{
//	u8 _cnt=0;
//	u8 sum = 0,i;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x06;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_1);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_1);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_2);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_2);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_3);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_3);
//	data_to_send[_cnt++]=BYTE1(Moto_PWM_4);
//	data_to_send[_cnt++]=BYTE0(Moto_PWM_4);
//	data_to_send[_cnt++]=0;//BYTE1(Moto_PWM_5);
//	data_to_send[_cnt++]=0;//BYTE0(Moto_PWM_5);
//	data_to_send[_cnt++]=0;//BYTE1(Moto_PWM_6);
//	data_to_send[_cnt++]=0;//BYTE0(Moto_PWM_6);
//	data_to_send[_cnt++]=0;//BYTE1(Moto_PWM_7);
//	data_to_send[_cnt++]=0;//BYTE0(Moto_PWM_7);
//	data_to_send[_cnt++]=0;//BYTE1(Moto_PWM_8);
//	data_to_send[_cnt++]=0;//BYTE0(Moto_PWM_8);
//	
//	data_to_send[3] = _cnt-4;

//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
//void Data_Send_OFFSET(void)
//{
//	u8 _cnt=0;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x16;
//	data_to_send[_cnt++]=0;
//	vs16 _temp = AngleOffset_Rol*1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = AngleOffset_Pit*1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	
//	u8 sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
//void Data_Send_PID1(void)
//{
//	u8 _cnt=0;
//	vs16 _temp;
//	u8 sum = 0,i;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAF;
//	data_to_send[_cnt++]=0x10;
//	data_to_send[_cnt++]=0;
//	
//	_temp = PID_ROL.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_ROL.I * 1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_ROL.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PIT.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PIT.I * 1000;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PIT.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_YAW.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_YAW.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_YAW.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	

//	for(i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
//void Data_Send_PID2(void)
//{
//	u8 _cnt=0;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x11;
//	data_to_send[_cnt++]=0;
//	
//	vs16 _temp;
//	_temp = PID_ALT.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_ALT.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_ALT.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_POS.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_POS.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_POS.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_1.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_1.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_1.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	
//	u8 sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}
//void Data_Send_PID3(void)
//{
//	u8 _cnt=0;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x12;
//	data_to_send[_cnt++]=0;
//	
//	vs16 _temp;
//	_temp = PID_PID_2.P * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_2.I * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	_temp = PID_PID_2.D * 100;
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
//	
//	data_to_send[3] = _cnt-4;
//	
//	u8 sum = 0;
//	for(u8 i=0;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;
//	
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,_cnt);
//#else
//	NRF_TxPacket(data_to_send,_cnt);
//#endif
//}

//void Data_Send_Check(u16 check)  
//{
//	u8 sum = 0,i;
//	data_to_send[0]=0xAA;
//	data_to_send[1]=0xAA;
//	data_to_send[2]=0xF0;
//	data_to_send[3]=3;
//	data_to_send[4]=0xBA;
//	
//	data_to_send[5]=BYTE1(check);
//	data_to_send[6]=BYTE0(check);
//	
//	for(i=0;i<7;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[7]=sum;
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(data_to_send,8);
//#else
//	NRF_TxPacket(data_to_send,8);
//#endif
//}



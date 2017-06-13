#include "Nrf24l01.h"
//#include "spi.h"

uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH]={0xaa,0xaf,0x03,0x05,0x03,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x64};//nrf24l01需要发送的数据
u8  TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//本地地址
u8  RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//接收地址
u8 flag=0;
extern u8 data_to_send[35];
//写寄存器
void NRF_Write_Reg(u8 reg,u8 data)
{
	u8 status;
	NRF_ENABLE;//使能
	status=SPI2_ReadWriteByte(reg); //写地址
	SPI2_ReadWriteByte(data); //写数据
	NRF_DISENABLE;
}
//读寄存器
u8 NRF_Read_Reg(u8 reg)
{
	u8 reg_val;
	NRF_ENABLE;//使能
	SPI2_ReadWriteByte(reg); //写地址
	reg_val=SPI2_ReadWriteByte(0xff); 
	NRF_DISENABLE;
	return reg_val;
}
//*****************************************************************//
// 写缓冲区
//*****************************************************************//

uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	NRF_ENABLE;				        /* 选通器件 */
	status = SPI2_ReadWriteByte(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		SPI2_ReadWriteByte(pBuf[i]);		/* 写数据 */
	}
	NRF_DISENABLE;						/* 禁止该器件 */
   return 	status;	
}
/*
*****************************************************************
* 读缓冲区
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	NRF_ENABLE;						/* 选通器件 */
	status = SPI2_ReadWriteByte(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = SPI2_ReadWriteByte(0); /* 读取返回数据 */ 	
	}
	NRF_DISENABLE;						/* 禁止该器件 */
    return 	status;
}
/*
*****************************************************************
* 写数据包
*****************************************************************
*/
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{
	u16 i=500;
	NRF_CE_L;		 //StandBy I模式	
	
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据	
	NRF_CE_H;		 //置高CE，激发数据发送
	//while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)!=RESET);
}
////////////////////////////////////////////////////////////////
u8 Nrf24l01_Check(void)
{ 
	u8 buf1[5]; 
	u8 i;
	NRF_CE_L;
	/*写入5个字节的地址. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5);
	/*读出写入的地址 */ 
	NRF_CE_L;
	NRF_Read_Buf(TX_ADDR,buf1,5); 
	/*比较*/ 
	for(i=0;i<5;i++) 
	{ 
		if(buf1[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
		return SUCCESS ; //MCU与NRF成功连接 
	else
		return ERROR ; //MCU与NRF不正常连接 
}

//******************************************************//
void Nrf24l01_Init(u8 model, u8 ch)
{
	NRF_CE_L;
	NRF_Write_Reg(NRF_WRITE_REG+CONFIG,0x0d);//掉电
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x03);//地址宽度
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//使能通道0的自动应答 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//使能通道0的接收地址 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//设置自动重发间隔时间:500us;最大自动重发次数:10次 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,0);														//设置RF通道为CHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
	}
	else if(model==2)		//TX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
	}
	else if(model==3)		//RX2
	{
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收
		
		SPI2_ReadWriteByte(0x50);
		SPI2_ReadWriteByte(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	else								//TX2
	{
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		
		SPI2_ReadWriteByte(0x50);
		SPI2_ReadWriteByte(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	NRF_CE_H;
}

void spi2_Init()
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	//GPIO口配置设置//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
    /*Configure PA.4(NSS)--------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
	 /* SPI1 configuration */ 
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//主机模式
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //8位数据
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_High=模式3，时钟空闲为高 //SPI_CPOL_Low=模式0，时钟空闲为低
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;//SPI_CPHA_1Edge, SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Soft;//SPI_NSS_Hard
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//SPI_BaudRatePrescaler_2=32M;//SPI_BaudRatePrescaler_4=18MHz
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//数据从高位开始发送
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	
	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设
}

//void EXTIX_Init(void) //IRO
//{  
//	EXTI_InitTypeDef EXTI_InitStructure;   
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0;
//  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
// 
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);  //GPIOE.2 中断线以及中断初始化配置，下降沿触发     
//	EXTI_InitStructure.EXTI_Line=EXTI_Line1;   
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;   
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;    
//	EXTI_Init(&EXTI_InitStructure);                      //初始化
//}

//void EXTI1_IRQHandler(void) 
//{ 
//	u8 sta;
//	EXTI_ClearITPendingBit(EXTI_Line1);  //清除中断标志位
//	 
//  //NRF_TxPacket(NRF24L01_TXDATA, RX_PLOAD_WIDTH);    //发送数据
//	sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);   //读取状态
//	NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);  //清楚标志位
//		////////////////////////////////////////////////////////////////
//	  if(sta & (1<<TX_DS))                                   //发送中断
//	  { 
//		 flag=1;                                               //发送成功
//	  }
//		if((sta & (1<<MAX_RT)))
//		{
//			if(sta & 0x01)	                                     //TX FIFO FULL
//			{
//				NRF_Write_Reg(FLUSH_TX,0xff);
//			}
//		} 	
//}


/***************************************************************/
//SPIx 
//TxData:发送一个字节
//返回值:data
/***************************************************************/
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //等待SPI发送标志位空
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI2, TxData); //发送数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //等待SPI接收标志位空
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI2); //接收数据					    
}

u8 TX_Funcation()
{
		u8 sta,j=250;
		NRF_TxPacket(data_to_send, RX_PLOAD_WIDTH);    //发送数据s	
		sta = NRF_Read_Reg(NRF_READ_REG + NRFRegSTATUS);   //读取状态
		NRF_Write_Reg(NRF_WRITE_REG + NRFRegSTATUS, sta);  //清楚标志位
		////////////////////////////////////////////////////////////////
		if(sta & (1<<TX_DS))
		{
			return 1;                                    //发送成功
		}
		if((sta & (1<<MAX_RT)))
		{
			if(sta & 0x01)	                                     //TX FIFO FULL
			{
				NRF_Write_Reg(FLUSH_TX,0xff);
			}
		}
		while(j--);
		return 0;
		///////////////////////////////////////////////////////////////	
}







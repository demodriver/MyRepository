#ifndef _SPI_H_
#define _SPI_H_
#include "stdbool.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"

// 定义MPU9250内部地址
/*****************************************************************/
#define	SMPLRT_DIV		                      0x19	//陀螺仪采样率
#define	SPI_CONFIG			                    0x1A	
#define	GYRO_CONFIG		                      0x1B	
#define	ACCEL_CONFIG	                      0x1C	
#define	ACCEL_CONFIG_2                      0x1D 

#define INT_PIN_CFG                         0x37 //中断配置
#define USER_CTRL                           0x6a
#define I2C_MST_CTRL                        0x24
#define I2C_MST_DELAY_CTRL                  0x67
//--------------------i2c slv0-------------------------------//
#define I2C_SLV0_ADDR                       0x25  
#define I2C_SLV0_REG                        0x26
#define I2C_SLV0_CTRL                       0x27 
#define I2C_SLV0_DO                         0x63 //output reg
//--------------------AK8963 reg addr------------------------//
#define MPU9250_AK8963_ADDR                 0x0C  //AKM addr
#define AK8963_WHOAMI_REG                   0x00  //AKM ID addr
#define AK8963_WHOAMI_ID                    0x48  //ID
#define AK8963_ST1_REG                      0x02  //Data Status1
#define AK8963_ST2_REG                      0x09  //Data reading end register & check Magnetic sensor overflow occurred
#define AK8963_ST1_DOR                      0x02
#define AK8963_ST1_DRDY                     0x01 //Data Ready
#define AK8963_ST2_BITM                     0x10
#define AK8963_ST2_HOFL                     0x08 // Magnetic sensor overflow 
#define AK8963_CNTL1_REG                    0x0A
#define AK8963_CNTL2_REG                    0x0B
#define AK8963_CNTL2_SRST                   0x01 //soft Reset
#define AK8963_ASAX                         0x10 //X-axis sensitivity adjustment value 
#define AK8963_ASAY                         0x11 //Y-axis sensitivity adjustment value
#define AK8963_ASAZ                         0x12 //Z-axis sensitivity adjustment value
//--------------------9axis  reg addr-----------------------//
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define XA_OFFSET_H   0x77
#define XA_OFFSET_L   0x78
#define YA_OFFSET_H   0x7A
#define YA_OFFSET_L   0x7B
#define ZA_OFFSET_H   0x7D
#define ZA_OFFSET_L   0x7E

#define	TEMP_OUT_H		0x41   //temperture
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define XG_OFFSET_H   0x13
#define XG_OFFSET_L   0x14
#define YG_OFFSET_H   0x15
#define YG_OFFSET_L   0x16
#define ZG_OFFSET_H   0x17
#define ZG_OFFSET_L   0x18

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08
//--------------------other reg addr-----------------------//
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//ID地址寄存器(正确数值0x71，只读)

#define EXT_SENS_DATA_00    0x49  //MPU9250 IIC外挂器件读取返回寄存器00
#define EXT_SENS_DATA_01    0x4a  //MPU9250 IIC外挂器件读取返回寄存器01
#define EXT_SENS_DATA_02    0x4b  //MPU9250 IIC外挂器件读取返回寄存器02
#define EXT_SENS_DATA_03    0x4c  //MPU9250 IIC外挂器件读取返回寄存器03
#define EXT_SENS_DATA_04    0x4d  //MPU9250 IIC外挂器件读取返回寄存器04
#define EXT_SENS_DATA_05    0x4e  //MPU9250 IIC外挂器件读取返回寄存器05


/************************SPI CS ********************************/
#define MPU_9250_DISENABLE  GPIOB->BSRR = GPIO_Pin_0;//片选
#define MPU_9250_ENABLE  GPIOB->BRR = GPIO_Pin_0;

typedef struct{
	int16_t Accel[3];//Accel X,Y,Z
	int16_t Gyro[3];//Gyro X,Y,Z
}MPU_value;
extern MPU_value current_value; 

u8 SPI1_ReadWriteByte(u8 TxData);
void spi_Init(void);
void Init_MPU9250(void);
u8 MPU9250_Write_Reg(u8 reg,u8 value);//SPI写
u8 MPU9250_Read_Reg(u8 reg);//SPI读
void READ_MPU9250_ACCEL(void);//读取加速度
void READ_MPU9250_GYRO(void);//读取陀螺仪
void GET_MPU9250_MAG(void);
void READ_MPU9250_MAG(void);//读取地磁计

void MPU_DATANGEL(void);
void MPU9250_CalOff_Gyr(void);
void MPU9250_CalOff_Acc(void);

#endif

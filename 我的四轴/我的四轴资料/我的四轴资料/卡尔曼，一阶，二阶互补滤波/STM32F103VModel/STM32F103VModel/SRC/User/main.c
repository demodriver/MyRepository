/**
  ****************************************************
  *@file: 
  *@author: wszdxmh
  *@version: V1.0
  *@date: 2016.05.13
  *@brief:
  ****************************************************
	*@attention
	*
	*
	****************************************************
  */

/*引用头文件*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include "led.h"
#include "tim.h"
#include "usart.h"
#include "delay.h"
#include "sys.h"
#include "spi.h"
#include "stdio.h"
#include "stdlib.h"
#include "motor.h"
#include "mpu6050.h"
#include "ANO-Tech.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "Kalman_filter.h"


/*定义全局变量*/
uint16_t TIM5CH1_CAPTURE_VAL;//TIM5_IRQHeader
uint8_t TIM5CH1_CAPTURE_STA;//TIM5_IRQHearder
//定义变量
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
extern float  Acc_angle,Gry_vivi;
extern float Angle,Gyro_x;         //小车滤波后倾斜角度/角速度	
extern float Angle_x_temp;  //由加速度计算的x倾斜角度
extern float Angle_y_temp;  //由加速度计算的y倾斜角度
extern float Angle_z_temp;
extern float Angle_X_Final; //X最终倾斜角度
extern float Angle_Y_Final; //Y最终倾斜角度
extern float Angle_Z_Final; //Z最终倾斜角度
extern float Gyro_x;		 //X轴陀螺仪数据暂存
extern float Gyro_y;        //Y轴陀螺仪数据暂存
extern float Gyro_z;		 //Z轴陀螺仪数据暂存
extern float A_P,A_R,A2_P;


int main(void)
{
	//初始化硬件
	SystemInit();
	delay_init();
	LED_Init();
	LED_Open(0,0);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	TIM1_Init(2499,7199);
	Motor_Init(999,71);
	Usart_Init(500000);
	MPU6050_Init();
	while(mpu_dmp_init());
	while(1)
	{
//		MOTOR_FORWARD(500);
//		delay_ms(1500);
//		MOTOR_REVERSE(500);
//		delay_ms(1500);
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
//			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			Angle_Calcu();
			ANO_DT_Send_Senser(Angle_x_temp,Angle_X_Final,(Angle_y_temp),(Angle_Y_Final),(Angle_z_temp),(Angle_Z_Final));
		}
	}
}

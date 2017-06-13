#ifndef _CONTROL_H
#define _CONTROL_H
#include "stm32f10x.h"
#include "attitude1_calculate.h"
  
typedef struct{
	float PIT;
	float ROL;
	float YAW;
}ANGEL;
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
typedef struct{
		u8	ARMED;    //解锁状态
}T_RC_Control;

typedef struct{
	float P;
	float I;
	float D;
}PID;


extern T_RC_Data  Rc_Data; //遥控通道数据
extern T_RC_Control Rc_Control; //遥控功能控制
//extern PID  PID_PIT,PID_ROL,PID_YAW;//PID值

void Control(Out_Angel *Q_ANGLE , T_RC_Data *Rc_Data, T_RC_Control Rc_Control,MPU_value *Final_senser);
#endif

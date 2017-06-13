#include "Control.h"
#include "Moto.h"
#define outMax   200  
#define outMin  -200
#define PIDMIX(X,Y,Z) Rc_Data->THROTTLE +(Shell_OutPut.ROL*X + Shell_OutPut.PIT*Y + Shell_OutPut.YAW*Z)

extern MPU_value Final_senser;//
T_RC_Data  Rc_Data; //遥控通道数据
T_RC_Control Rc_Control; //遥控功能控制
PID  PID_SHELL_PIT,PID_SHELL_ROL,PID_SHELL_YAW;//外环PID值
PID  PID_CORE_PIT,PID_CORE_ROL,PID_CORE_YAW; //内环PID
ANGEL ITerm,Shell_OutPut,OutPut;//积分、PID输出
ANGEL ANGEL_ERROR,dInPut;//误差值 ，误差积分值
//***********************************************************************************//
//                               PID控制
//***********************************************************************************//
void Control(Out_Angel *Q_ANGLE , T_RC_Data *Rc_Data, T_RC_Control Rc_Control,MPU_value *Final_senser)//传入参数：当前角度值、期望角度值、功能
{
	static T_RC_Data Rc_Data_temp;
	static ANGEL LAST_ANGEL;
//**********************************************************************************//
	PID_SHELL_PIT.P=1.6f;
	PID_SHELL_PIT.I=0.0f;
	PID_SHELL_PIT.D=0.0f;
	
	PID_SHELL_ROL.P=1.6f;
	PID_SHELL_ROL.I=0.0f;
	PID_SHELL_ROL.D=0.0f;
	
//	PID_SHELL_YAW.P=0.0;
//	PID_SHELL_YAW.I=0.0;
//	PID_SHELL_YAW.D=0.0;
	
//**********************************************************************************//	
//	PID_YAW.P=0.0f;
//	PID_YAW.I=0.0f;
//	PID_YAW.D=0.0f;
	
//	PID_CORE_PIT.P=3.0f;
//	PID_CORE_PIT.I=0.0f;
//	PID_CORE_PIT.D=10.0f;
//	
//	PID_CORE_ROL.P=3.0f;
//	PID_CORE_ROL.I=0.0f;
//	PID_CORE_ROL.D=10.0f;
	
//*************************************************************************************//
   Rc_Data_temp.YAW=(Rc_Data->YAW - 1000)/50 ;
	 Rc_Data_temp.PIT=(Rc_Data->PIT - 1000)/50 ;
	 Rc_Data_temp.ROL=(Rc_Data->ROL - 1000)/50 ;
//*************************************************************************************//
	ANGEL_ERROR.PIT = Rc_Data_temp.PIT - Q_ANGLE->PIT;    //当前Pitch  减去  期望pitch 误差
	
	ITerm.PIT += PID_SHELL_PIT.I * ANGEL_ERROR.PIT;    //误差积分(消除纹波)
	
	 if(ITerm.PIT > outMax)     //对PID的输出进行范围限制
	 {
		  ITerm.PIT = outMax;  
	 }	     
	 else if(ITerm.PIT < outMin) 
	 {
		  ITerm.PIT = outMin;
	 }
	dInPut.PIT =Final_senser->Gyro[1];//(Q_ANGLE->PIT - LAST_ANGEL.PIT);//输入微分
	
	 Shell_OutPut.PIT = PID_SHELL_PIT.P * ANGEL_ERROR.PIT + ITerm.PIT + PID_SHELL_PIT.D * dInPut.PIT;
	 if(Shell_OutPut.PIT > 500) Shell_OutPut.PIT=500;
	 if(Shell_OutPut.PIT < -500) Shell_OutPut.PIT=-500;
//*************************************************************************************//	
	ANGEL_ERROR.ROL = Rc_Data_temp.ROL -Q_ANGLE->ROL;    //Roll 误差   当前 - 期望
	ITerm.ROL += PID_SHELL_ROL.I * ANGEL_ERROR.ROL;    //误差积分(消除纹波)
	
	 if(ITerm.ROL > outMax)     //对PID的输出进行范围限制
	 {
		  ITerm.ROL = outMax;  
	 }	     
	 else if(ITerm.ROL < outMin) 
	 {
		  ITerm.ROL = outMin;
	 }
	dInPut.ROL =Final_senser->Gyro[0];//(Q_ANGLE->ROL - LAST_ANGEL.ROL);//输入微分 （减少微分毛刺）
	
	 Shell_OutPut.ROL = PID_SHELL_ROL.P * ANGEL_ERROR.ROL + ITerm.ROL + PID_SHELL_ROL.D * dInPut.ROL;
	 if(Shell_OutPut.ROL > 500) Shell_OutPut.ROL=500;
	 if(Shell_OutPut.ROL < -500) Shell_OutPut.ROL=-500;
//*************************************************************************************//	
//	ANGEL_ERROR.YAW =0 - Q_ANGLE->YAW;    //yaw误差  当前 - 期望
//	ITerm.YAW += PID_SHELL_YAW.I * ANGEL_ERROR.YAW;    //误差积分(消除纹波)
//	
//	 if(ITerm.YAW > 200)     //对PID的输出进行范围限制
//	 {
//		  ITerm.YAW = 200;  
//	 }	     
//	 else if(ITerm.YAW < -200) 
//	 {
//		  ITerm.YAW = -200;
//	 }
//	dInPut.YAW =(Q_ANGLE->YAW - LAST_ANGEL.YAW);//输入微分 （减少微分毛刺）
//	
//	 Shell_OutPut.YAW = PID_SHELL_YAW.P * ANGEL_ERROR.YAW + ITerm.YAW - PID_SHELL_YAW.D * dInPut.YAW;
//*********************************************************************//
//	 LAST_ANGEL.PIT =Q_ANGLE->PIT;
//	 LAST_ANGEL.ROL =Q_ANGLE->ROL;
//	 LAST_ANGEL.YAW =Q_ANGLE->YAW;
	 
	 if(Rc_Data->THROTTLE>200)
	 {
		  Moto_PWM_1=PIDMIX(+1,-1,-1);//front right 
			Moto_PWM_2=PIDMIX(-1,-1,+1);// front lift
			Moto_PWM_3=PIDMIX(+1,+1,+1);//back right
			Moto_PWM_4=PIDMIX(-1,+1,-1);//back left
	 }
	 else
	 {
		  Moto_PWM_1=0;
			Moto_PWM_2=0;
			Moto_PWM_3=0;
			Moto_PWM_4=0;
	 }
	 if(Rc_Control.ARMED) //解锁状态下，开启PWM
	{
		MotoReFlash(Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4);	 
	}
	else
	{
		MotoReFlash(0,0,0,0);	 
	}
		
}


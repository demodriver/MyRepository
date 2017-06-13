#include "attitude1_calculate.h"

//*************************************************************//
//                           互补滤波
//*************************************************************//
Out_Angel Q_ANGLE; //最终姿态角
#define Rad 57.324841f  //弧度转角度
#define PI_RAD 0.017453f
#define Kp1 12.0f          //proportional gain governs rate of convergence to accelerometer/magnetometer 
#define Ki1 0.008f        //integral gain governs rate of convergence of gyroscopebiases      
#define halfT 0.001f //采样时间的一半
extern MPU_value Final_senser;//
extern MPU_value last_value;//上一次测量值	
extern MPU_value offset_value;           //零漂
float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // quaternion elements representing the estimated orientation 
float eaxInt=0,eayInt=0,eazInt=0; // scaledintegralerror
float emxInt=0,emyInt=0,emzInt=0; // scaledintegralerror
extern MPU_value last_value;

//******************************************************************************
// 扩展反正弦函数：检查输入值的范围(-1~1)若非数字(isnan函数判断)返回0
//******************************************************************************
//float safe_asin(float v)
//{
//	if (isnan(v)) return 0.0f;

//	if (v >= 1.0f) return 3.1415926/2;

//	if (v <= -1.0f)return -3.1415926/2;
//	
//	return asin(v);
//}
//*****************************************************************************//
//  IMU姿态解算
//*****************************************************************************//
static void IMUupdate(float gx,float gy,float gz,float ax,float ay,float az)
{ 
	float norm; 
	float vx,vy,vz;  
	//float hx,hy,hz,bz,bx;
	//float wx,wy,wz;  
	float eax,eay,eaz;//,emx,emy,emz;  // 误差
	//计算用到的参数
  float q0q0=q0*q0;
	float q0q1=q0*q1; 
	float q0q2=q0*q2; 
	float q1q1=q1*q1; 
	float q1q3=q1*q3;
	float q2q2=q2*q2;
	float q2q3=q2*q3;
	float q3q3=q3*q3;
	float q0q3=q0*q3;
	float q1q2=q1*q2;
	
	if(ax*ay*az==0) 
	return;

	//normalise the measurements 
//*************************加速度误差计算*****************//	
	norm = sqrt(ax*ax + ay*ay + az*az); //数据归一化
	ax=ax/ norm;         //载体坐标系测量重力矢量
	ay=ay/ norm; 
	az=az/ norm; 
	
	vx=2*(q1q3-q0q2);  //参考坐标系实际重力矢量
	vy=2*(q0q1+q2q3); 
	vz=q0q0-q1q1-q2q2+q3q3; 
	
	eax= (ay*vz-az*vy);  //和误差
	eay= (az*vx-ax*vz);
	eaz= (ax*vy-ay*vx);
	
	eaxInt = eaxInt + eax * Ki1;    //误差积分
	eayInt = eayInt + eay * Ki1;
	eazInt = eazInt + eaz * Ki1;
////**************************磁力计误差计算***************//
//	norm = sqrt(mx*mx + my*my + mz*mz);
//	mx=mx/ norm; //归一化   //载体坐标系测量矢量
//	my=my/ norm; 
//	mz=mz/ norm; 
//	
//		//计算参考磁通方向 
//	hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);  
//	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);  
//	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);           
//	bx = sqrt((hx*hx) + (hy*hy));  
//	bz = hz;          
//	//估计磁通
//	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);  
//	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);  
//	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);    

//	emx= (my*wz - mz*wy);  //和误差
//	emy= (mz*wx - mx*wz);
//	emz= (mx*wy - my*wx);
//	
//	emxInt=emxInt+emx*Ki2;    //误差积分
//	emyInt=emyInt+emy*Ki2;
//	emzInt=emzInt+emz*Ki2;
//************************陀螺仪纠正********************//	
//应用算法:	互补滤波算法
	gx=gx + eax*Kp1 + eaxInt;//+Kp2*emx+emxInt; //将误差PI后补偿零点漂移
	gy=gy + eay*Kp1 + eayInt;//+Kp2*emx+emxInt; 
	gz=gz + eaz*Kp1 + eazInt;//+Kp2*emx+emxInt; 
//********************更新四元数***********************//
	q0=q0+(-q1*gx-q2*gy-q3*gz)*halfT; 
	q1=q1+(q0*gx+ q2*gz-q3*gy)*halfT; 
	q2=q2+(q0*gy- q1*gz+q3*gx)*halfT; 
	q3=q3+(q0*gz+ q1*gy-q2*gx)*halfT; 
//*********************姿态角计算*********************//	
	norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3); //四元数归一化
	q0=q0/norm; 
	q1=q1/norm; 
	q2=q2/norm; 
	q3=q3/norm; 
//----------------------------------------------------//	
	//Q_ANGLE.YAW =atan2(2*q1*q2+2*q0*q3,-2*q2*q2-2*q3*q3+1)*Rad; //yaw 
	//Q_ANGLE.YAW += gz;
	Q_ANGLE.PIT =asin(-2*q1*q3+2*q0*q2)*Rad;// pitch 
	Q_ANGLE.ROL =atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * Rad; //roll
		
}
//*****************************************************//
//滑动均滤波
//*****************************************************//
static void Prepare_Data(float Acc_x,float Acc_y,float Acc_z)
{
	#define FILTER_NUM 	10
	static uint8_t 	filter_cnt=0;
	static int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];
	uint32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;

	ACC_X_BUF[filter_cnt] = Acc_x;
	ACC_Y_BUF[filter_cnt] = Acc_y;
	ACC_Z_BUF[filter_cnt] = Acc_z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}	
	Acc_x = temp1 / FILTER_NUM;
	Acc_y = temp2 / FILTER_NUM;
	Acc_z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}

//******************************************************//
//                    姿态解算
//******************************************************//
void attitude_calculate(MPU_value current_value)//当前9轴数据
{
				float Acc_x;
				float Acc_y;
				float Acc_z;
				float Gyro_x;
				float Gyro_y;
				float Gyro_z;
	
	       Acc_x=(last_value.Accel[0] + current_value.Accel[0])/2.0;//滑动均值滤波
				 Acc_y=(last_value.Accel[1] + current_value.Accel[1])/2.0;
				 Acc_z=(last_value.Accel[2] + current_value.Accel[2])/2.0;
				
				 Gyro_x=(last_value.Gyro[0] + current_value.Gyro[0])/2.0; //- offset_value.Gyro[0];
				 Gyro_y=(last_value.Gyro[1] + current_value.Gyro[1])/2.0; //- offset_value.Gyro[1];
				 Gyro_z=(last_value.Gyro[2] + current_value.Gyro[2])/2.0; //- offset_value.Gyro[2];
////////////////////////////////////////////////////////////////////////////
				Acc_x/=164;//数据处理
				Acc_y/=164;
				Acc_z/=164;				
				Gyro_x/=16.4; //数据处理
				Gyro_y/=16.4;
				Gyro_z/=16.4;	
				
				Prepare_Data(Acc_x,Acc_y,Acc_z);//滑动滤波
				
				Final_senser.Accel[0]=Acc_x;
				Final_senser.Accel[1]=Acc_y;
				Final_senser.Accel[2]=Acc_z;
				Final_senser.Gyro[0]=Gyro_x;
				Final_senser.Gyro[1]=Gyro_y;
				Final_senser.Gyro[2]=Gyro_z;
/////////////////////////////////////////////////////////////////				
				Gyro_x*=PI_RAD; //度转弧度
				Gyro_y*=PI_RAD;
				Gyro_z*=PI_RAD;					
//*********************姿态融合********************************//	
	IMUupdate(Gyro_x,Gyro_y,Gyro_z,Acc_x,Acc_y,Acc_z);
}
	




#include "attitude_calculate.h"
#define  PItoAngel  180/3.14159      //弧度化角度
//MPU_value current_value;//当前值
float kp1=10.0,ki1=0.08,kp2=10.0,ki2=0.08;//kp决定互补滤波截止频率，ki决定消除静态偏差时间 ki==kp的0.01~~0.1倍
float last_attitude[3];//姿态角
float offset_Gyro[3];//陀螺仪补偿值
double eabI[3],ecI[3];//误差积分
float Gyro[3];
float Rxx=1,Rxy=0,Rxz=0,Ryx=0,Ryy=1,Ryz=0,Rzx=0,Rzy=0,Rzz=1;
float sinC,cosC;//C为偏航角
/*****************互补滤波**********************/
void attitude_calculate(MPU_value current_value)//当前9轴数据
{
	u8 i,Acc_x,Acc_y,Acc_z,Mag_x,Mag_y;
  float tmp,tmp_e;//tmp_e为向量模长//Rxx为方向余弦元素，假设初始载体坐标与参考系坐标重合
  float ax,ay,az;//单位加速度三轴值
  float eab[3],ec[3];//pitch和roll误差、yow误差
	float time=0.01;//采样时间1ms(积分中的dt)

  //处理x轴加速度
	if (current_value.Accel[0]<200) Acc_x = current_value.Accel[0];
	else              Acc_x = 1-(current_value.Accel[0] - 200);
	 //处理y轴加速度
	if (current_value.Accel[1]<200) Acc_y = current_value.Accel[1];
	else              Acc_y = 1-(current_value.Accel[1] - 200);
	 //处理z轴加速度
	if (current_value.Accel[2]<200) Acc_z = current_value.Accel[2];
	else              Acc_z = 1-(current_value.Accel[2] - 200);
	
	//处理x轴陀螺仪
	if (current_value.Gyro[0]<2000) Gyro[0] = current_value.Gyro[0];
	else              Gyro[0] = 1-(current_value.Gyro[0] - 2000);
	 //处理y轴陀螺仪
	if (current_value.Gyro[1]<2000) Gyro[1] = current_value.Gyro[1];
	else              Gyro[1] = 1-(current_value.Gyro[1] - 2000);
	 //处理z轴陀螺仪
	if (current_value.Gyro[2]<2000) Gyro[2] = current_value.Gyro[2];
	else              Gyro[2] = 1-(current_value.Gyro[2] - 2000);

	//处理x轴磁力计
	if (current_value.Mag[0]<4912) Mag_x = current_value.Mag[0];
	else              Mag_x = 1-(current_value.Mag[0] - 4912);
	//处理y轴磁力计
	if (current_value.Mag[1]<4912) Mag_x = current_value.Mag[1];
	else              Mag_y = 1-(current_value.Mag[1] - 4912);
	
	
	
//**********************************计算相关参数*****************************//	
	tmp=fabs(Mag_x)*fabs(Mag_x)+fabs(Mag_y)*fabs(Mag_y);//偏航角为电子罗盘测量值 （取绝对值计算）
	tmp_e=sqrt(tmp);
	sinC=Mag_x/tmp_e;
	cosC=Mag_y/tmp_e;
//------------------------------------------------//	
	tmp=fabs(Acc_x)*fabs(Acc_x)+fabs(Acc_y)*fabs(Acc_y)+fabs(Acc_z)*fabs(Acc_z);
	tmp_e=sqrt(tmp);    //加速度模
	ax=Acc_x/tmp_e;    //加速度单位化
	ay=Acc_y/tmp_e;
	az=Acc_z/tmp_e;
//********************计算姿态角误差***************************//	
	eab[0]=(ax*Rzy-ay*Rzz);//计算pitch和roll误差//计算值全为加速度向量值
	eab[1]=(ax*Rzz-Rzx*az);
	eab[2]=(Rzx*ay-ax*Rzy);
	eabI[0]+=eab[0]*ki1;//x轴加速度向量积分
	eabI[1]+=eab[1]*ki1;//y轴加速度向量积分
	eabI[2]+=eab[2]*ki1;//z轴加速度向量积分
//-------------------------------------------------//	
	tmp=fabs(Rxx)*fabs(Rxx)+fabs(Ryx)*fabs(Ryx);//用磁力计的航向数据计算yow误差
	tmp_e=(Rxx*sinC-Ryx*cosC)/sqrt(tmp);
	ec[0]=Rxz*tmp_e;//注意此处将参考系向量变换到载体系，应为  {x,y,z}*R的逆矩阵（或R的转置矩阵 ，因为R-1==RT（正交矩阵性质））
	ec[1]=Ryz*tmp_e;
	ec[2]=Rzz*tmp_e;
	ecI[0]+=ec[0]*ki2;//x轴磁力计向量积分
	ecI[1]+=ec[1]*ki2;//y轴磁力计向量积分
	ecI[2]+=ec[2]*ki2;//z轴磁力计向量积分
	
	  for(i=0;i<3;i++)
		{
			offset_Gyro[i]=kp1*eab[i]+eabI[i]+kp2*ec[i]+ecI[i]+Gyro[i];//角融合
		}
//*************************更新余弦方阵****************************//			
			Rxx=Rxx+(Rxy*offset_Gyro[2]-Rxz*offset_Gyro[1])*time;
			Rxy=Rxy+(Rxz*offset_Gyro[0]-Rxx*offset_Gyro[2])*time;
			Rxz=Rxz+(Rxx*offset_Gyro[1]-Rxy*offset_Gyro[0])*time;
			
			Ryx=Ryx+(Ryy*offset_Gyro[2]-Ryz*offset_Gyro[1])*time;
			Ryy=Ryy+(Ryz*offset_Gyro[0]-Ryx*offset_Gyro[2])*time;
			Ryz=Ryz+(Ryx*offset_Gyro[1]-Ryy*offset_Gyro[0])*time;
			
			Rzx=Rzx+(Rzy*offset_Gyro[2]-Rzz*offset_Gyro[1])*time;
			Rzy=Rzy+(Rzz*offset_Gyro[0]-Rzx*offset_Gyro[2])*time;
			Rzz=Rzz+(Rzx*offset_Gyro[1]-Rzy*offset_Gyro[0])*time;
//**************************姿态角*********************************//	
	tmp=fabs(Rzx)*fabs(Rzx)+fabs(Rzy)*fabs(Rzy)+fabs(Rzz)*fabs(Rzz);
	tmp_e=sqrt(tmp);    //加速度模
	Rzx=Rzx/tmp_e;    //加速度单位化
	Rzy=Rzy/tmp_e;
	Rzz=Rzz/tmp_e;
	tmp=fabs(Ryx)*fabs(Ryx)+fabs(Ryy)*fabs(Ryy)+fabs(Ryz)*fabs(Ryz);
	tmp_e=sqrt(tmp);    //加速度模
	Ryx=Rzx/tmp_e;    //加速度单位化
	Ryy=Ryy/tmp_e;
	Ryz=Ryz/tmp_e;
	tmp=fabs(Rxx)*fabs(Rxx)+fabs(Rxy)*fabs(Rxy)+fabs(Rxz)*fabs(Rxz);
	tmp_e=sqrt(tmp);    //加速度模
	Rxx=Rxx/tmp_e;    //加速度单位化
	Rxy=Rxy/tmp_e;
	Rxz=Rxz/tmp_e;
		
			last_attitude[0]=asin(-Rzx)*PItoAngel;  //final pitch			 	  
			last_attitude[1]=atan(Rzy/Rzz)*PItoAngel; //final roll
			last_attitude[2]=atan(Ryx/Rxx)*PItoAngel; //final yow

}





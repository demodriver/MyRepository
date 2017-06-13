#include "time.h"
#include "Datatansfer.h"
extern u8 FLAG_ATT;
//u32 TIM3_IRQCNT = 0;  //led标志位
//NVIC中断配置//
//******************************************//
//时钟配置
void time3_Init(u16 period_num)//period_num==500   
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	
	
	TIM_TimeBaseInitStruct.TIM_Prescaler=72-1; //prescaler is 72,that is 72000000/72/period_num=2000Hz;
	TIM_TimeBaseInitStruct.TIM_Period=period_num; //装载值
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);//时钟初始化
	//clear the TIM3 overflow interrupt flag
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//清除标志位
	//TIM3 overflow interrupt enable
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	//中断使能
}
//***************************//
//时钟使能控制
void Tim3_Control(u8 sta)
{
	if(sta==0)
		TIM_Cmd(TIM3,DISABLE);
	if(sta==1)
		TIM_Cmd(TIM3,ENABLE);
}


//*************************//
//u32				Debug_cnt;
//u32 debug_cntshow=0;
void TIM3_IRQHandler(void)		//0.5ms中断一次
{	
	static u8 ms1_cnt=0;
	static u16 s1_cnt=0,s2_cnt=0;
	
	if(TIM3->SR & TIM_IT_Update)		//if ( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET ) 
	{     
		TIM3->SR = ~TIM_FLAG_Update;//TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);   //清除中断标志
		//TIM3_IRQCNT ++;
		
		ms1_cnt++;
		s1_cnt++;
		s2_cnt++;
		if(ms1_cnt==2)//1ms 
		{
			ms1_cnt = 0;
			FLAG_ATT=1;
		}
		if(s1_cnt==20)//2ms 
		{
			s1_cnt = 0;
			Send_RCData=1;
		}
		Data_Exchange(); //发送控制信息
	}
}


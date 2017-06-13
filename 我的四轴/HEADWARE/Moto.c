#include "Moto.h"
#define Moto_PwmMax 1999
#define Moto_PwmMin 500
vs16 Moto_PWM_1,Moto_PWM_2,Moto_PWM_3,Moto_PWM_4; //电机输出
void MotoReFlash(int16_t Moto_PWM_1,int16_t Moto_PWM_2,int16_t Moto_PWM_3,int16_t Moto_PWM_4)
{
	if(Moto_PWM_1>Moto_PwmMax)	Moto_PWM_1 = Moto_PwmMax;
	if(Moto_PWM_2>Moto_PwmMax)	Moto_PWM_2 = Moto_PwmMax;
	if(Moto_PWM_3>Moto_PwmMax)	Moto_PWM_3 = Moto_PwmMax;
	if(Moto_PWM_4>Moto_PwmMax)	Moto_PWM_4 = Moto_PwmMax;
	if(Moto_PWM_1<Moto_PwmMin)	Moto_PWM_1 =0;
	if(Moto_PWM_2<Moto_PwmMin)	Moto_PWM_2 =0;
	if(Moto_PWM_3<Moto_PwmMin)	Moto_PWM_3 =0;
	if(Moto_PWM_4<Moto_PwmMin)	Moto_PWM_4 =0;
	
	TIM4->CCR1 = Moto_PWM_4;
	TIM4->CCR2 = Moto_PWM_3;
	TIM4->CCR3 = Moto_PWM_2;
	TIM4->CCR4 = Moto_PWM_1;
}
//********************************************************//
//PWM
void Time4_Init()
{
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure; 
	TIM_OCInitTypeDef  TIM_OCInitStructure;  
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	//时钟配置
	TIM_TimeBaseStructure.TIM_Period = 1999;            //装载周期
	TIM_TimeBaseStructure.TIM_Prescaler =72-1;          //预分频值 计数器时钟1M
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //时钟分频:TDTS = Tck_tim 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);   //初始化TIMx  
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;     //选择PWM模式1 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //输出极性高
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);                 //通道OC1初始化  
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能预装载寄存器
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);                 //通道OC2初始化  
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能预装载寄存器
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);                 //通道OC3初始化  
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能预装载寄存器
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);                 //通道OC4初始化  
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能预装载寄存器
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);               //自动重装使能
  TIM_Cmd(TIM4, ENABLE);                          //使能TIM4 
}

void Moto_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能电机用的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

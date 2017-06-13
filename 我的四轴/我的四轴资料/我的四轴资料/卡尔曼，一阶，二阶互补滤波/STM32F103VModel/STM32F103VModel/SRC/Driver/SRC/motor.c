
#include "motor.h"

/**
  ************************************
  * @brief  PWM的初始化函数
  * @param  arr: 自动重装周期值   psc:预分频值
  * @retval arr=19999  psc=71     f=72000000/((19999+1)*(71+1))=50Hz
	************************************
*/
void Motor_Init(uint16_t arr, uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;//定义结构体
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//初始化I/O时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//GPIOA外部时钟使能，特别重要！！！
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//设置IO口
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//设置为普通推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//频率50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	//初始化PWM
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE); //使能GPIO外设和AFIO复用功能模块时钟
	
	//设置该引脚为复用输出功能,输出TIM3 CH3 CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	
	//定时器TIM2初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler = psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM2 Channel2 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	//初始化TIM3 chanl3
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC3
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

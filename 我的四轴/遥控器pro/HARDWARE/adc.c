#include "adc.h"

u16 ADC_Temp[10];
void ADC_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //开启时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //adc时钟分频因子  12M	
	
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//模拟输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);         //GPIOA
 
	ADC_DeInit(ADC1); 
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC工作模式：独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //AD 扫描通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //AD 连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //软件启动转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC 数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 7; //顺序进行adc转换通道数目 6
	ADC_Init(ADC1, &ADC_InitStructure); //初始化 ADCx  
	
	ADC_Cmd(ADC1, ENABLE); //使能ADC1 
	ADC_ResetCalibration(ADC1); //开启adc1校准
	while(ADC_GetResetCalibrationStatus(ADC1)); //等待复位校准结束
	ADC_StartCalibration(ADC1);             //开启ad校准
	while(ADC_GetCalibrationStatus(ADC1));  //等待ad校准结束
	
	//设置指定adc的规则通道，设置他们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, 1, 1, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_RegularChannelConfig(ADC1, 2, 2, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_RegularChannelConfig(ADC1, 3, 3, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_RegularChannelConfig(ADC1, 4, 4, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_RegularChannelConfig(ADC1, 5, 5, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_RegularChannelConfig(ADC1, 6, 6, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_RegularChannelConfig(ADC1, 7, 7, ADC_SampleTime_239Cycles5 ); //adc1、通道1，规则采样顺序值1，采样时间239.5周期   
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能指定的adc1的软件转换功能 
}

void DMA_INIT(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA时钟
	 
	DMA_InitStructure.DMA_PeripheralBaseAddr =((uint32_t)0x4001244C); 	//DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_Temp;  //DMA 内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //从外设读取发送到内存
	DMA_InitStructure.DMA_BufferSize = 7;  //DMA缓存大小 （一次传输数据量）  7*16
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //16位半字传输 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道x拥有中优先级  
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //非内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //初始化
	ADC_DMACmd(ADC1, ENABLE);//使能DMA
	DMA_Cmd(DMA1_Channel1, ENABLE);
}


void Get_Adc(void)    
{
	u8 i;	
	for(i=0;i<7;i++)
	{
	  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	  //ADC_Temp[i]=ADC_GetConversionValue(ADC1); //返回最近一次adc1规则组转化结果
	}
} 


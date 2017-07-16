#include "adc.h"
#include "include.h"

/* 电池电压采集函数 */

uint16_t ADC_ConvertedValue = 0;
u16 Battry_Voltage;		//单位：V * 100

void Rheostat_ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 使能 GPIO 时钟
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_GPIO_CLK, ENABLE);

	// 配置 IO
	GPIO_InitStructure.GPIO_Pin = RHEOSTAT_ADC_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //不上拉不下拉
	GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStructure);
}

void Rheostat_ADC_Mode_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	// 开启ADC时钟
	RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK , ENABLE);

	// -------------------ADC Common 结构体 参数 初始化------------------------
	// 独立ADC模式
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// 时钟为fpclk x分频	
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// 禁止DMA直接访问模式	
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// 采样时间间隔	
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  
	ADC_CommonInit(&ADC_CommonInitStructure);

	// -------------------ADC Init 结构体 参数 初始化--------------------------
	ADC_StructInit(&ADC_InitStructure);
	// ADC 分辨率
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	// 禁止扫描模式，多通道采集才需要	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	// 连续转换	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
	//禁止外部边沿触发
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//外部触发通道，本例子使用软件触发，此值随便赋值即可
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	//数据右对齐	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//转换通道 1个
	ADC_InitStructure.ADC_NbrOfConversion = 1;                                    
	ADC_Init(RHEOSTAT_ADC, &ADC_InitStructure);
	//---------------------------------------------------------------------------

	// 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
	ADC_RegularChannelConfig(RHEOSTAT_ADC, RHEOSTAT_ADC_CHANNEL, 1, ADC_SampleTime_56Cycles);
	// ADC 转换结束产生中断，在中断服务程序中读取转换值


	ADC_ITConfig(RHEOSTAT_ADC, ADC_IT_EOC, ENABLE);
	// 使能ADC
	ADC_Cmd(RHEOSTAT_ADC, ENABLE);  
	//开始adc转换，软件触发
	ADC_SoftwareStartConv(RHEOSTAT_ADC);
}

// 配置中断优先级
static void Rheostat_ADC_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = Rheostat_ADC_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

void ADC_IRQHandler(void)
{
	if(ADC_GetITStatus(RHEOSTAT_ADC,ADC_IT_EOC)==SET)
	{
		// 读取ADC的转换值
		ADC_ConvertedValue = ADC_GetConversionValue(RHEOSTAT_ADC);
	}
	ADC_ClearITPendingBit(RHEOSTAT_ADC,ADC_IT_EOC);
}	

void ANO_ADC_Init(void)
{
	Rheostat_ADC_GPIO_Config();
	Rheostat_ADC_Mode_Config();
	Rheostat_ADC_NVIC_Config();
}

//读取数据
void ADC_Read(void)
{
	float tmp;
	
	//计算当前电压
	tmp = (float)ADC_ConvertedValue * Voltage_To_V;
	Battry_Voltage = (u16)(tmp * 100);
}

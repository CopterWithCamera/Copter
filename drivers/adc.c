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
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	// ------------------DMA Init 结构体参数 初始化--------------------------
	
	// ADC1使用DMA2，数据流0，通道0，这个是手册固定死的
	
	// 开启DMA时钟
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_DMA_CLK, ENABLE); 
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = RHEOSTAT_ADC_DR_ADDR;	// 外设基址为：ADC 数据寄存器地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&ADC_ConvertedValue;  	// 存储器地址，实际上就是一个内部SRAM的变量	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;				// 数据传输方向为外设到存储器	
	DMA_InitStructure.DMA_BufferSize = 1;								// 缓冲区大小为，指一次传输的数据量
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// 外设寄存器只有一个，地址不用递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; 			// 存储器地址固定
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 	// 外设数据大小为半字，即两个字节 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		//	存储器数据大小也为半字，跟外设数据大小相同
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;					// 循环传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;				// DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  	// 禁止DMA FIFO	，使用直连模式
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;	// FIFO 大小，FIFO模式禁止时，这个不用配置	
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	DMA_InitStructure.DMA_Channel = RHEOSTAT_ADC_DMA_CHANNEL; 	// 选择 DMA 通道，通道存在于流中
	DMA_Init(RHEOSTAT_ADC_DMA_STREAM, &DMA_InitStructure);	//初始化DMA流，流相当于一个大的管道，管道里面有很多通道
	
	DMA_Cmd(RHEOSTAT_ADC_DMA_STREAM, ENABLE);	// 使能DMA流

	// 开启ADC时钟
	RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK , ENABLE);
	
	// -------------------ADC Common 结构体 参数 初始化------------------------
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	//独立ADC模式
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;	//时钟为fpclk x分频	
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;	//禁止DMA直接访问模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  //采样时间间隔	
	ADC_CommonInit(&ADC_CommonInitStructure);

	// -------------------ADC Init 结构体 参数 初始化--------------------------
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	// ADC 分辨率
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 			// 禁止扫描模式，多通道采集才需要	
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 	// 连续转换
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 	// 单次转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;	//禁止外部边沿触发
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;	//使用软件触发，外部触发不用配置，注释掉即可
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//数据右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;   //转换通道 1个
	ADC_Init(RHEOSTAT_ADC, &ADC_InitStructure);
	//---------------------------------------------------------------------------

	// 配置 ADC 通道转换顺序为1，第一个转换，采样时间为3个时钟周期
	ADC_RegularChannelConfig(RHEOSTAT_ADC, RHEOSTAT_ADC_CHANNEL, 1, ADC_SampleTime_56Cycles);
	ADC_DMARequestAfterLastTransferCmd(RHEOSTAT_ADC, ENABLE);	// 使能DMA请求 after last transfer (Single-ADC mode)
	ADC_DMACmd(RHEOSTAT_ADC, ENABLE);	// 使能ADC DMA
	ADC_Cmd(RHEOSTAT_ADC, ENABLE); 	// 使能ADC

}

//初始化
void ANO_ADC_Init(void)
{
	Rheostat_ADC_GPIO_Config();
	Rheostat_ADC_Mode_Config();
}

//读取数据
void ADC_Read(void)
{
	float tmp;
	
	//开始下一次adc转换，软件触发
	ADC_SoftwareStartConv(RHEOSTAT_ADC);
	
	//计算当前电压
	tmp = (float)ADC_ConvertedValue * Voltage_To_V;
	Battry_Voltage = (u16)(tmp * 100);
}

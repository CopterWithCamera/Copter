#include "adc.h"
#include "include.h"

/* ��ص�ѹ�ɼ����� */

uint16_t ADC_ConvertedValue = 0;
u16 Battry_Voltage;		//��λ��V * 100

static void Rheostat_ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// ʹ�� GPIO ʱ��
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_GPIO_CLK, ENABLE);

	// ���� IO
	GPIO_InitStructure.GPIO_Pin = RHEOSTAT_ADC_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	    
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //������������
	GPIO_Init(RHEOSTAT_ADC_GPIO_PORT, &GPIO_InitStructure);		
}

static void Rheostat_ADC_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	//DMA����
	
	// ------------------DMA Init �ṹ����� ��ʼ��--------------------------
	// ADC1ʹ��DMA2��������0��ͨ��0��������ֲ�̶�����
	// ����DMAʱ��
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_DMA_CLK, ENABLE); 
	// �����ַΪ��ADC ���ݼĴ�����ַ
	DMA_InitStructure.DMA_PeripheralBaseAddr = RHEOSTAT_ADC_DR_ADDR;	
	// �洢����ַ��ʵ���Ͼ���һ���ڲ�SRAM�ı���	
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&ADC_ConvertedValue;  
	// ���ݴ��䷽��Ϊ���赽�洢��	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
	// ��������СΪ��ָһ�δ����������
	DMA_InitStructure.DMA_BufferSize = 1;	
	// ����Ĵ���ֻ��һ������ַ���õ���
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	// �洢����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; 
	// // �������ݴ�СΪ���֣��������ֽ� 
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
	//	�洢�����ݴ�СҲΪ���֣����������ݴ�С��ͬ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	
	// ѭ������ģʽ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	// DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	// ��ֹDMA FIFO	��ʹ��ֱ��ģʽ
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  
	// FIFO ��С��FIFOģʽ��ֹʱ�������������	
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	// ѡ�� DMA ͨ����ͨ������������
	DMA_InitStructure.DMA_Channel = RHEOSTAT_ADC_DMA_CHANNEL; 
	//��ʼ��DMA�������൱��һ����Ĺܵ����ܵ������кܶ�ͨ��
	DMA_Init(RHEOSTAT_ADC_DMA_STREAM, &DMA_InitStructure);
	// ʹ��DMA��
	DMA_Cmd(RHEOSTAT_ADC_DMA_STREAM, ENABLE);

	//ADC����

	// ����ADCʱ��
	RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK , ENABLE);

	// -------------------ADC Common �ṹ�� ���� ��ʼ��------------------------
	// ����ADCģʽ
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// ʱ��Ϊfpclk x��Ƶ	
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// ��ֹDMAֱ�ӷ���ģʽ	
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// ����ʱ����	
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  
	ADC_CommonInit(&ADC_CommonInitStructure);

	// -------------------ADC Init �ṹ�� ���� ��ʼ��--------------------------
	ADC_StructInit(&ADC_InitStructure);
	// ADC �ֱ���
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	// ��ֹɨ��ģʽ����ͨ���ɼ�����Ҫ	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	// ����ת��
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	//��ֹ�ⲿ���ش���
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//�ⲿ����ͨ����������ʹ�������������ֵ��㸳ֵ����
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	//�����Ҷ���	
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//ת��ͨ�� 1��
	ADC_InitStructure.ADC_NbrOfConversion = 1;                                    
	ADC_Init(RHEOSTAT_ADC, &ADC_InitStructure);
	//---------------------------------------------------------------------------

	// ���� ADC ͨ��ת��˳��Ϊ1����һ��ת��������ʱ��Ϊ3��ʱ������
	ADC_RegularChannelConfig(RHEOSTAT_ADC, RHEOSTAT_ADC_CHANNEL, 1, ADC_SampleTime_56Cycles);

	// ʹ��DMA���� after last transfer (Single-ADC mode)
	ADC_DMARequestAfterLastTransferCmd(RHEOSTAT_ADC, ENABLE);
	
	// ʹ��ADC DMA
	ADC_DMACmd(RHEOSTAT_ADC, ENABLE);

	// ʹ��ADC
	ADC_Cmd(RHEOSTAT_ADC, ENABLE);

}

void ANO_ADC_Init(void)
{
	Rheostat_ADC_GPIO_Config();
	Rheostat_ADC_Mode_Config();
}

//��ȡ����
void ADC_Read(void)
{
	float tmp;
	static u8 counter = 0;
	
	counter++;
	if(counter > 10)	//�˺�����50ms�߳��ÿ10�β�1������ÿ0.5s����1��
	{
		counter = 0;
		
		//��ʼadcת�����������
		ADC_SoftwareStartConv(RHEOSTAT_ADC);
		
		//���㵱ǰ��ѹ
		tmp = (float)ADC_ConvertedValue * Voltage_To_V;
		Battry_Voltage = (u16)(tmp * 100);
	}
}

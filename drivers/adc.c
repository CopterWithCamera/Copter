#include "adc.h"
#include "include.h"

/* ��ص�ѹ�ɼ����� */

uint16_t ADC_ConvertedValue = 0;
u16 Battry_Voltage;		//��λ��V * 100

void Rheostat_ADC_GPIO_Config(void)
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

void Rheostat_ADC_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	// ------------------DMA Init �ṹ����� ��ʼ��--------------------------
	
	// ADC1ʹ��DMA2��������0��ͨ��0��������ֲ�̶�����
	
	// ����DMAʱ��
	RCC_AHB1PeriphClockCmd(RHEOSTAT_ADC_DMA_CLK, ENABLE); 
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = RHEOSTAT_ADC_DR_ADDR;	// �����ַΪ��ADC ���ݼĴ�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)&ADC_ConvertedValue;  	// �洢����ַ��ʵ���Ͼ���һ���ڲ�SRAM�ı���	
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;				// ���ݴ��䷽��Ϊ���赽�洢��	
	DMA_InitStructure.DMA_BufferSize = 1;								// ��������СΪ��ָһ�δ����������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	// ����Ĵ���ֻ��һ������ַ���õ���
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; 			// �洢����ַ�̶�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 	// �������ݴ�СΪ���֣��������ֽ� 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		//	�洢�����ݴ�СҲΪ���֣����������ݴ�С��ͬ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;					// ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;				// DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  	// ��ֹDMA FIFO	��ʹ��ֱ��ģʽ
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;	// FIFO ��С��FIFOģʽ��ֹʱ�������������	
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
	DMA_InitStructure.DMA_Channel = RHEOSTAT_ADC_DMA_CHANNEL; 	// ѡ�� DMA ͨ����ͨ������������
	DMA_Init(RHEOSTAT_ADC_DMA_STREAM, &DMA_InitStructure);	//��ʼ��DMA�������൱��һ����Ĺܵ����ܵ������кܶ�ͨ��
	
	DMA_Cmd(RHEOSTAT_ADC_DMA_STREAM, ENABLE);	// ʹ��DMA��

	// ����ADCʱ��
	RCC_APB2PeriphClockCmd(RHEOSTAT_ADC_CLK , ENABLE);
	
	// -------------------ADC Common �ṹ�� ���� ��ʼ��------------------------
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;	//����ADCģʽ
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;	//ʱ��Ϊfpclk x��Ƶ	
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;	//��ֹDMAֱ�ӷ���ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;  //����ʱ����	
	ADC_CommonInit(&ADC_CommonInitStructure);

	// -------------------ADC Init �ṹ�� ���� ��ʼ��--------------------------
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	// ADC �ֱ���
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 			// ��ֹɨ��ģʽ����ͨ���ɼ�����Ҫ	
	//ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 	// ����ת��
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 	// ����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;	//��ֹ�ⲿ���ش���
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;	//ʹ������������ⲿ�����������ã�ע�͵�����
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//�����Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 1;   //ת��ͨ�� 1��
	ADC_Init(RHEOSTAT_ADC, &ADC_InitStructure);
	//---------------------------------------------------------------------------

	// ���� ADC ͨ��ת��˳��Ϊ1����һ��ת��������ʱ��Ϊ3��ʱ������
	ADC_RegularChannelConfig(RHEOSTAT_ADC, RHEOSTAT_ADC_CHANNEL, 1, ADC_SampleTime_56Cycles);
	ADC_DMARequestAfterLastTransferCmd(RHEOSTAT_ADC, ENABLE);	// ʹ��DMA���� after last transfer (Single-ADC mode)
	ADC_DMACmd(RHEOSTAT_ADC, ENABLE);	// ʹ��ADC DMA
	ADC_Cmd(RHEOSTAT_ADC, ENABLE); 	// ʹ��ADC

}

//��ʼ��
void ANO_ADC_Init(void)
{
	Rheostat_ADC_GPIO_Config();
	Rheostat_ADC_Mode_Config();
}

//��ȡ����
void ADC_Read(void)
{
	float tmp;
	
	//��ʼ��һ��adcת�����������
	ADC_SoftwareStartConv(RHEOSTAT_ADC);
	
	//���㵱ǰ��ѹ
	tmp = (float)ADC_ConvertedValue * Voltage_To_V;
	Battry_Voltage = (u16)(tmp * 100);
}

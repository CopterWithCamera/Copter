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
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

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
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
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
	// ADC ת�����������жϣ����жϷ�������ж�ȡת��ֵ


	ADC_ITConfig(RHEOSTAT_ADC, ADC_IT_EOC, ENABLE);
	// ʹ��ADC
	ADC_Cmd(RHEOSTAT_ADC, ENABLE);  
	//��ʼadcת�����������
	ADC_SoftwareStartConv(RHEOSTAT_ADC);
}

// �����ж����ȼ�
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
		// ��ȡADC��ת��ֵ
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

//��ȡ����
void ADC_Read(void)
{
	float tmp;
	
	//���㵱ǰ��ѹ
	tmp = (float)ADC_ConvertedValue * Voltage_To_V;
	Battry_Voltage = (u16)(tmp * 100);
}

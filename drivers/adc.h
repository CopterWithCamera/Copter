#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f4xx.h"

// ADC GPIO �궨��
#define RHEOSTAT_ADC_GPIO_PORT    GPIOC
#define RHEOSTAT_ADC_GPIO_PIN     GPIO_Pin_5
#define RHEOSTAT_ADC_GPIO_CLK     RCC_AHB1Periph_GPIOC

// ADC ��ź궨��
#define RHEOSTAT_ADC              ADC1
#define RHEOSTAT_ADC_CLK          RCC_APB2Periph_ADC1
#define RHEOSTAT_ADC_CHANNEL      ADC_Channel_15


// ADC �жϺ궨��
#define Rheostat_ADC_IRQ          ADC_IRQn

void ANO_ADC_Init(void);	//��ʼ��
void ADC_Read(void);		//��ȡ����

extern u16 Battry_Voltage;

#endif


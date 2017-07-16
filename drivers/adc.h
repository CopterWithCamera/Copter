#ifndef _ADC_H_
#define _ADC_H_

#include "stm32f4xx.h"

// ADC GPIO 宏定义
#define RHEOSTAT_ADC_GPIO_PORT    GPIOC
#define RHEOSTAT_ADC_GPIO_PIN     GPIO_Pin_5
#define RHEOSTAT_ADC_GPIO_CLK     RCC_AHB1Periph_GPIOC

// ADC 序号宏定义
#define RHEOSTAT_ADC              ADC1
#define RHEOSTAT_ADC_CLK          RCC_APB2Periph_ADC1
#define RHEOSTAT_ADC_CHANNEL      ADC_Channel_15


// ADC 中断宏定义
#define Rheostat_ADC_IRQ          ADC_IRQn

void ANO_ADC_Init(void);	//初始化
void ADC_Read(void);		//读取数据

extern u16 Battry_Voltage;

#endif


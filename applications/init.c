/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：init.c
 * 描述    ：飞控初始化
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "include.h"
#include "pwm_out.h"
#include "mpu6050.h"
#include "i2c_soft.h"
#include "led.h"
#include "ctrl.h"
#include "ms5611.h"
#include "ak8975.h"
#include "ultrasonic.h"
#include "adc.h"

u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置
	
	SysTick_Configuration(); 	//滴答时钟
	
	I2c_Soft_Init();					//初始化模拟I2C
	
	PWM_IN_Init();						//初始化接收机采集功能
	
	PWM_Out_Init(400);				//初始化电调输出功能	
	
	Usb_Hid_Init();						//飞控usb接口的hid初始化
	
	MS5611_Init();						//气压计初始化
	
	Delay_ms(400);						//延时
	
	MPU6050_Init(20);   			//加速度计、陀螺仪初始化，配置20hz低通
	
	LED_Init();								//LED功能初始化
	
	Usart1_Init(115200);
	Usart2_Init(115200);					//串口2初始化，函数参数为波特率
	Usart3_Init(115200);
	
	Para_Init();							//参数初始化
	
	Delay_ms(100);						//延时
	
	#if defined(USE_ANO_OF)	//使用光流	//初始化光流串口（跟超声波公用一个口）
		Uart5_Init(500000);	
	#else					//使用超声波	//超声波初始化（串口5初始化）
		Ultrasonic_Init();
	#endif
	
	ak8975_ok = !(ANO_AK8975_Run());
	
	ANO_ADC_Init();					//ADC电池电压采集
	
	if( !mpu6050_ok )
	{
		LED_MPU_Err();
	}
	else if( !ak8975_ok )
	{
		LED_Mag_Err();
	}
	else if( !ms5611_ok )
	{
		LED_MS5611_Err();
	}
	
	aircraft_mode_led(MAXMOTORS);
	
	Cycle_Time_Init();
	
 	return (1);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

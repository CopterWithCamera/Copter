#ifndef _INCLUDE_H_
#define _INCLUDE_H_

#include "stm32f4xx.h"
#include "scheduler.h"
#include "time.h"
#include "init.h"
#include "parameter.h"
#include "pwm_in.h"
#include "usart.h"
#include "usbd_user_hid.h"
#include "data_transfer.h"
#include "track_function.h"

//==============机型匹配=================

#define __COPTER_NO1
//#define __COPTER_NO2

//================系统===================

#define USE_US100				//使用us100型号超声波
//#define	USE_KS103			//使用KS103型号超声波
//#define	USE_ANO_OF			//使用光流传感器

#define MAXMOTORS 			(4)		//电机数量
#define GET_TIME_NUM 		(10)		//设置获取时间的数组数量
#define CH_NUM 				(8) 	//接收机通道数量

#define ANO_DT_USE_USART2 		//开启串口2数传功能
#define ANO_DT_USE_USB_HID		//开启飞控USBHID连接上位机功能

//#define ANO_OF_FUNCTION			//光流线程使能

//=============中断优先级=================

#define NVIC_GROUP 				NVIC_PriorityGroup_3	//中断分组选择
#define NVIC_PWMIN_P			1			//接收机采集
#define NVIC_PWMIN_S			1
#define NVIC_TIME_P       		2			//Timer2（配置但未使用）
#define NVIC_TIME_S       		0

//================传感器===================
#define ACC_ADJ_EN 								//是否允许校准加速度计,(定义则允许)

#define OFFSET_AV_NUM		50					//校准偏移量时的平均次数。
#define FILTER_NUM 			10					//滑动平均滤波数值个数

#define TO_ANGLE 			0.06103f 			//0.061036 //   4000/65536  +-2000   ???

#define FIX_GYRO_Y 			1.02f				//陀螺仪Y轴固有补偿
#define FIX_GYRO_X 			1.02f				//陀螺仪X轴固有补偿

#define TO_M_S2 			0.23926f   			//  980cm/s2    +-8g	980/4096
#define ANGLE_TO_RADIAN 	0.01745329f 		//  *0.01745 = /57.3	角度转弧度

#define MAX_ACC  			4096.0f				//  +-8G		加速度计量程
#define TO_DEG_S 			500.0f      		//  T = 2ms  默认为2ms ，数值等于1/T

#define	Voltage_To_V 		0.00886f			// 分压系数为11    11 * 3.3V / 4096 = 0.000805664 * 11 = 0.0088623

#define HEIGHT_SOURCE		1					// 1：加速度融合数据		2：低通滤波数据

//================控制=====================
#define MAX_VERTICAL_SPEED_UP	5000										//最大上升速度mm/s
#define MAX_VERTICAL_SPEED_DW	3000										//最大下降速度mm/s

#define MAX_CTRL_ANGLE			30.0f										//遥控能达到的最大角度
#define ANGLE_TO_MAX_AS 		30.0f										//角度误差N时，期望角速度达到最大（可以通过调整CTRL_2的P值调整）
#define CTRL_2_INT_LIMIT 		0.5f *MAX_CTRL_ANGLE						//外环积分幅度

#define MAX_CTRL_ASPEED 	 	300.0f										//ROL,PIT允许的最大控制角速度
#define MAX_CTRL_YAW_SPEED 		150.0f										//YAW允许的最大控制角速度
#define CTRL_1_INT_LIMIT 		0.5f *MAX_CTRL_ASPEED						//内环积分幅度


#define MAX_PWM			100			///%	最大PWM输出为100%油门
#define MAX_THR       	80 			///%	油门通道最大占比80%，留20%给控制量
#define READY_SPEED   	20			///%	解锁后电机转速20%油门
//=========================================

enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_Y ,
 G_X ,
 G_Z ,
 TEM ,
 ITEMS ,
};

// CH_filter[],0横滚，1俯仰，2油门，3航向		
enum
{
 ROL= 0,
 PIT ,
 THR ,
 YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
};

//=========================================


#endif


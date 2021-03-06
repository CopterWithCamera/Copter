#ifndef __ANO_OF_H_
#define __ANO_OF_H_

#include "stm32f4xx.h"


//以下为全局变量，在其他文件中，引用本h文件，即可在其他文件中访问到以下变量

extern uint8_t 	OF_QUA,OF_LIGHT;	//光流信息质量：QUA	//光照强度：LIGHT

extern int8_t	OF_DX,OF_DY;		//原始光流信息，具体意义见光流模块手册
extern int16_t	OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;		//融合后的光流信息，具体意义见光流模块手册（单位cm/s）
extern uint16_t	OF_ALT,OF_ALT2;		//原始高度信息和融合后高度信息（单位cm）

extern int16_t	OF_GYR_X,OF_GYR_Y,OF_GYR_Z;		//原始陀螺仪数据
extern int16_t	OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;	//滤波后陀螺仪数据
extern int16_t	OF_ACC_X,OF_ACC_Y,OF_ACC_Z;		//原始加速度数据
extern int16_t	OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;	//滤波后加速度数据

extern float	OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;		//欧拉角格式的姿态数据
extern float	OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;//四元数格式的姿态数据

//本函数是唯一一个需要外部调用的函数，因为光流模块是串口输出数据
//所以本函数需要在串口接收中断中调用，每接收一字节数据，调用本函数一次
void AnoOF_GetOneByte(uint8_t data);

void flow_data_detect(float T);

extern float OF_DX2_DETECT,		//横滚速度		+ <---  ---> -
			OF_DY2_DETECT,		//俯仰速度		+ <前-- --后> -
			OF_DX2FIX_DETECT,
			OF_DY2FIX_DETECT,
			OF_DX2_DETECT_LPF,
			OF_DY2_DETECT_LPF,
			OF_DX2FIX_DETECT_LPF,
			OF_DY2FIX_DETECT_LPF;

#endif

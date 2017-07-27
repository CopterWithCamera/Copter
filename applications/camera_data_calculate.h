#ifndef _CAMERA_DATA_CALCULATE_H_
#define	_CAMERA_DATA_CALCULATE_H_

#include "stm32f4xx.h"

extern float bias_detect;	//偏移的统计滤波结果
extern float bias_real;
extern float bias_lpf;
extern float speed_d_bias;
extern float speed_d_bias_lpf;
extern float receive_fps;
extern u16 receive_fps_counter;
extern u8 bias_error_flag;		//指示bias的状态   0：正常   1：异常恢复的第一帧（只有偏移没有速度）  2：异常

void Camera_Calculate(void);
void get_fps(void);				//在主线程里2s为周期调用

#endif


#ifndef _CAMERA_DATA_CALCULATE_H_
#define	_CAMERA_DATA_CALCULATE_H_

#include "stm32f4xx.h"

//roll方向
extern float bias_detect;	//偏移的统计滤波结果
extern u8 bias_error_flag;		//指示bias的状态   0：正常   1：异常恢复的第一帧（只有偏移没有速度）  2：异常
extern float bias_real;
extern float bias_lpf;
extern float speed_d_bias;
extern float speed_d_bias_lpf;

//pitch方向
extern float bias_detect_pitch;
extern u8 bias_error_flag_pitch;		//bias_detect_pitch值异常指示		0：正常    1：从异常中恢复回来后的第一帧    2：异常
extern float bias_real_pitch;
extern float bias_lpf_pitch;
extern float speed_d_bias_pitch;
extern float speed_d_bias_lpf_pitch;

//fps
extern float receive_fps;
extern u16 receive_fps_counter;


void Camera_Calculate(void);
void get_fps(void);				//在主线程里2s为周期调用

#endif


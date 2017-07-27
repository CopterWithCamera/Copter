#ifndef _CAMERA_DATA_CALCULATE_H_
#define	_CAMERA_DATA_CALCULATE_H_

#include "stm32f4xx.h"

extern float bias_detect;	//ƫ�Ƶ�ͳ���˲����
extern float bias_real;
extern float bias_lpf;
extern float speed_d_bias;
extern float speed_d_bias_lpf;
extern float receive_fps;
extern u16 receive_fps_counter;
extern u8 bias_error_flag;		//ָʾbias��״̬   0������   1���쳣�ָ��ĵ�һ֡��ֻ��ƫ��û���ٶȣ�  2���쳣

void Camera_Calculate(void);
void get_fps(void);				//�����߳���2sΪ���ڵ���

#endif


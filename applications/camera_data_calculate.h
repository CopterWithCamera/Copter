#ifndef _CAMERA_DATA_CALCULATE_H_
#define	_CAMERA_DATA_CALCULATE_H_

#include "stm32f4xx.h"

//roll����
extern float bias_detect;	//ƫ�Ƶ�ͳ���˲����
extern u8 bias_error_flag;		//ָʾbias��״̬   0������   1���쳣�ָ��ĵ�һ֡��ֻ��ƫ��û���ٶȣ�  2���쳣
extern float bias_real;
extern float bias_lpf;
extern float speed_d_bias;
extern float speed_d_bias_lpf;

//pitch����
extern float bias_detect_pitch;
extern u8 bias_error_flag_pitch;		//bias_detect_pitchֵ�쳣ָʾ		0������    1�����쳣�лָ�������ĵ�һ֡    2���쳣
extern float bias_real_pitch;
extern float bias_lpf_pitch;
extern float speed_d_bias_pitch;
extern float speed_d_bias_lpf_pitch;

//fps
extern float receive_fps;
extern u16 receive_fps_counter;


void Camera_Calculate(void);
void get_fps(void);				//�����߳���2sΪ���ڵ���

#endif


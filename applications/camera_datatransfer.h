#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//���ò�����

//ƫ��
extern float bias;
extern float bias_lpf;
extern float bias_real;

//�Ƕ�
extern float angle;

//�ٶ�
extern float speed;

//����
extern float fps;
extern float processing_fps;
extern float receive_fps;

//========================================================================

#endif

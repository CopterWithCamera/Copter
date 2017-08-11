#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//���ò�����

extern float bias;			//ƫ��
extern float bias_pitch;	//pitchƫ��
extern float angle;			//�Ƕ�


//����
extern float fps;
extern float processing_fps;
extern float receive_T;

//����״̬
extern float tracking_state;

extern float Roll_Image;		//�����Ӧ�ĽǶ�
extern float Pitch_Image;
extern float Yaw_Image;
extern float Height_Image;		//�����Ӧ�ĸ߶�

//========================================================================

#endif

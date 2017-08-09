#ifndef _TRACK_MODE_H
#define	_TRACK_MODE_H

#include "stm32f4xx.h"
#include "include.h"

extern u8 copter_fly_mode;		//��̬����ģʽ
extern u8 copter_height_mode;	//�߶ȿ���ģʽ

void Fly_Mode_Ctrl(float T);	//����ģʽ����
void Height_Mode_Ctrl(float T);	//�߶�ģʽ����

#endif

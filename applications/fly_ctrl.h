#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "stm32f4xx.h"
#include "include.h"

extern float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ
extern u8 ctrl_command;			//��ǰ����ָ��

//��������
extern float height_speed_ctrl;	//������ֱ�ٶ�


void Fly_Ctrl(void);			//��schedule�����
void Ctrl_Mode(float *ch_in);	//��fly_mode�����

#endif

#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "stm32f4xx.h"

#include "parameter.h"

//typedef xyz_f_t _xyz_f_t;
#define _xyz_f_t xyz_f_t

#define THR_TAKE_OFF_LIMIT 550

typedef struct
{
	float m_acc;
	float m_speed;
	float m_height;
	float fusion_acc;
	float fusion_speed;
	float fusion_height;

}_hc_value_st;

extern _hc_value_st hc_value;	//�߶�����ֵ�����ٶȡ��ٶȡ��߶ȣ�
extern float set_height_e;		//�߶�������
extern float thr_take_off;		//��׼����

float auto_take_off_land(float dT,u8 ready);
float Height_Ctrl(float T,u8 mode,float thr,float height,u8 ready,float en);
void h_pid_init(void);
#endif


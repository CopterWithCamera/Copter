#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "stm32f4xx.h"
#include "include.h"

//����Ŀ�������Ϣ
extern float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ
extern u8 All_Out_Switch;
extern u8 my_height_mode;
extern float my_except_height;

//��λ����ر�������λ�����״̬�������������������Ŀ��ƣ�
extern u8 ctrl_command;			//��ǰ����ָ��

//��ʱ�����߳�
void Fly_Ctrl(void);			//��schedule�����
void Ctrl_Mode(float *ch_in);	//��fly_mode�����

//�������봦����
void set_except_height(u8 height);	//���ո߶�����
void set_attitude_calibration(u8);


#endif

#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "stm32f4xx.h"
#include "include.h"

//����Ŀ�������Ϣ
extern float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ
extern u8 All_Out_Switch;
extern u8 my_height_mode;
extern float my_except_height;

//���п���ָ��
extern u8 ctrl_command;			//���з���ָ��
extern u8 height_command;		//���и߶�ָ��

//��ʱ�����߳�

void Fly_Height_Ctrl(float T);	//�߶ȿ��ƺ�����5ms��
void Fly_Ctrl(float T);			//5ms��̬���ƺ���
void Fly_Ctrl_Cam(float T);		//����ͷ����Ƶ����̬���ƺ���
void Fly_Ctrl_Flow(void);		//��������Ƶ����̬���ƺ���

void Ctrl_Mode(float *ch_in);	//��fly_mode�����

//�������봦����
void set_except_height(u8 height);	//���ո߶�����
void set_attitude_calibration(u8);
void set_all_out_switch(u8 cmd);
void set_height_mode(u8 cmd);
void set_fly_mode(u8 cmd);

extern u8 height_mode,	//�߶ȿ���ģʽ		0���ֶ��ظ�		1��������ǰ�߶�		2������ָ��߶ȿظ�		3�����		4������
		roll_speed,		//�ٶȺ������ģʽ	0���ֶ�����		1������ͷ���ݶ���	2���������ݶ���
		pitch_speed,	//�ٶȸ�������ģʽ	0���ֶ�����		1������ͷ���ݶ���	2���������ݶ���
		roll_position,	//λ�ú������		0�����0			1���������ͷ����ƫ��
		pitch_position,	//λ�ø�������		0�����0			1���������ͷ����ƫ��
		yaw_mode;		//����ǿ���			0���ֶ�����


#endif

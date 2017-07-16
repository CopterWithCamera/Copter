#ifndef __ANOTC_BARO_H
#define __ANOTC_BARO_H

#include "stm32f4xx.h"
#include "filter.h"
#include "imu.h"
#include "mymath.h"

#include "parameter.h"

#include "height_ctrl.h"

#include "ultrasonic.h"

//typedef xyz_f_t _xyz_f_t;
#define _xyz_f_t xyz_f_t

/*=====================================================================================================================
						 *****
=====================================================================================================================*/
typedef struct
{
	float b1;
	float b2;
	float b3;
	
	float g1;
	float g2;
	float g3;

}_f_set_st;

typedef struct
{
	float est_acc_old;
	_filter_1_st fusion_acceleration;	//�ںϺ���ٶ�
	_filter_1_st fusion_speed_m;		//�ںϺ���ٶȷ�������ȷ����
	_filter_1_st fusion_speed_me;		//�ںϹ����б��˵����ٶȷ�������ȷ����
	
	_filter_1_st fusion_displacement;	//�ںϵõ���λ�ƣ��߶ȣ���ֵ

}_fusion_st;
extern _fusion_st sonar_fusion;
extern _fusion_st baro_fusion;

typedef struct
{
	float dis_deadzone;			//����������ĸ߶�����
	float displacement;			//λ�ƣ��������ݹ������˲�+ת��ΪmmΪ��λ��
	float displacement_old;		//λ�ƾ�ֵ
	float speed;				//��λ�Ʋ��ʱ��õ����ٶ�
	float speed_old;			//�ٶȾ�ֵ
	float acceleration;			//���ٶȲ��ʱ��õ��ļ��ٶ�
	
}_fusion_p_st;
extern _fusion_p_st baro_p;
extern _fusion_p_st sonar;

typedef struct
{
	u8 item;
	float a;
	float b;
	float c;
}_h_f_set_st;

extern float wz_speed;

float baro_compensate(float dT,float kup,float kdw,float vz,float lim);
void baro_ctrl(float dT,_hc_value_st *);

#endif


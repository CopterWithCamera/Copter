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
	_filter_1_st fusion_acceleration;	//融合后加速度
	_filter_1_st fusion_speed_m;		//融合后的速度分量（不确定）
	_filter_1_st fusion_speed_me;		//融合过程中被滤掉的速度分量（不确定）
	
	_filter_1_st fusion_displacement;	//融合得到的位移（高度）数值

}_fusion_st;
extern _fusion_st sonar_fusion;
extern _fusion_st baro_fusion;

typedef struct
{
	float dis_deadzone;			//设置死区后的高度数据
	float displacement;			//位移（死区数据过滑动滤波+转换为mm为单位）
	float displacement_old;		//位移旧值
	float speed;				//用位移差÷时间得到的速度
	float speed_old;			//速度旧值
	float acceleration;			//用速度差÷时间得到的加速度
	
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


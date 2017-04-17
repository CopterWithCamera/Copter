#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "stm32f4xx.h"
#include "include.h"

extern float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值
extern u8 ctrl_command;			//当前飞行指令

//控制数据
extern float height_speed_ctrl;	//期望垂直速度


void Fly_Ctrl(void);			//在schedule里调用
void Ctrl_Mode(float *ch_in);	//在fly_mode里调用

#endif

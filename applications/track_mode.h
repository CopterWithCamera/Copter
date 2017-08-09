#ifndef _TRACK_MODE_H
#define	_TRACK_MODE_H

#include "stm32f4xx.h"
#include "include.h"

extern u8 copter_fly_mode;		//姿态控制模式
extern u8 copter_height_mode;	//高度控制模式

void Fly_Mode_Ctrl(float T);	//飞行模式控制
void Height_Mode_Ctrl(float T);	//高度模式控制

#endif

#ifndef _POSITION_FUNCTION_H
#define	_POSITION_FUNCTION_H

#include "stm32f4xx.h"
#include "include.h"

void attitude_roll(void);
void attitude_pitch(void);
void attitude_yaw(void);

//位置控制
void position_pitch_zero(void);
void position_roll_zero(void);

void position_pitch(float T);
void position_pitch_clear(void);

void position_roll(float T);
void position_roll_clear(void);

//速度控制
void speed_pitch(void);
void speed_pitch_clear(void);

void speed_roll(void);
void speed_roll_clear(void);

extern float position_pitch_out;		//输出速度期望，单位cm/s，方向 + <前-- --后> -
extern float position_roll_out;			//输出速度期望，单位cm/s，方向 + <---  ---> -

#endif

#ifndef _POSITION_FUNCTION_H
#define	_POSITION_FUNCTION_H

#include "stm32f4xx.h"
#include "include.h"

void attitude_roll(void);
void attitude_pitch(void);
void attitude_yaw(void);

//λ�ÿ���
void position_pitch_zero(void);
void position_roll_zero(void);

void position_pitch(float T);
void position_pitch_clear(void);

void position_roll(float T);
void position_roll_clear(void);

//�ٶȿ���
void speed_pitch(void);
void speed_pitch_clear(void);

void speed_roll(void);
void speed_roll_clear(void);

extern float position_pitch_out;		//����ٶ���������λcm/s������ + <ǰ-- --��> -
extern float position_roll_out;			//����ٶ���������λcm/s������ + <---  ---> -

#endif

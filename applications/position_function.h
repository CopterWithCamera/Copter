#ifndef _POSITION_FUNCTION_H
#define	_POSITION_FUNCTION_H

#include "stm32f4xx.h"
#include "include.h"

void attitude_hand(void);
void position_pitch(float T,u8 en);
void position_roll(float T,u8 en);
void speed_pitch(u8 en);
void speed_roll(u8 en);


#endif

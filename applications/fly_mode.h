#ifndef __FLY_MODE_H
#define __FLY_MODE_H

#include "stm32f4xx.h"
#include "include.h"
#include "parameter.h"

extern u8 mode_state;

void mode_check(float *ch_in);
#endif

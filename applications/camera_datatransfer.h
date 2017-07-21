#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);

//========================================================================

//可用参数表

//偏移
extern float bias;
extern float bias_lpf;
extern float bias_real;

//角度
extern float angle;

//速度
extern float speed;

//参数
extern float fps;
extern float processing_fps;
extern float receive_fps;

//========================================================================

#endif

#ifndef _CAMERA_DATATRANSFER_H_
#define	_CAMERA_DATATRANSFER_H_

#include "stm32f4xx.h"

void Copter_Data_Send(void);
void Copter_Receive_Handle(unsigned char data);


extern float angle;
extern float speed;

extern float length;
extern float Roll_Image;	//采图时对应的Roll

extern float real_length;

#endif

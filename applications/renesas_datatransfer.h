#ifndef _RENESAS_DATATRANSFER_H_
#define	_RENESAS_DATATRANSFER_H_

#include "stm32f4xx.h"

void renesas_data_receive_handle(u8 data);		//接收瑞萨发送的数据
void renesas_data_send(void);					//对瑞萨发送数据

#endif


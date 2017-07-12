#include "camera_datatransfer.h"
#include "usart.h"

void Camera_Data_Send(void)
{
	unsigned char a[2] = {1,2};
	Usart3_Send(a,2);
}
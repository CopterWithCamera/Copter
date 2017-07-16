#include "camera_datatransfer.h"
#include "usart.h"
#include "anotc_baro_ctrl.h"

/*

高度数据：

ultra.relative_height					cm		float
sonar.displacement						mm		float
sonar_fusion.fusion_displacement.out	mm		float

*/

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

unsigned char Data_Buffer[20];

void Send_to_Camera(unsigned char *DataToSend ,u8 data_num)
{
	Usart3_Send(DataToSend,data_num);
}

void Camrea_Send_Height(void)
{
	float tmp_f;
	
	u8 cnt = 0;
	
	//帧头
	Data_Buffer[cnt++] = 0xAA;	
	Data_Buffer[cnt++] = 0xAA;
	
	//功能字
	Data_Buffer[cnt++] = 0x01;	
	
	//内容
	tmp_f = ultra.relative_height*10;	//转为mm单位
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = sonar.displacement;
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = sonar_fusion.fusion_displacement.out;
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	Send_to_Camera(Data_Buffer,cnt);
}


void Camera_Data_Send(void)
{
	Camrea_Send_Height();
}


#include "renesas_datatransfer.h"
#include "fly_ctrl.h"
#include "include.h"
#include "height_function.h"
#include "track_mode.h"
#include "rc.h"
#include "camera_datatransfer.h"
#include "track_mode.h"
#include "anotc_baro_ctrl.h"

u8 tmpbuff[20] = {0};

void renesas_setting_cmd(u8 data)
{
	if(data == 0x01)
	{
		fly_ready = 0;			//锁定
	}
	else if(data == 0x02)
	{
		fly_ready = 1;			//解锁
	}
	else if(data == 0x03)
	{
		All_Out_Switch = 0;		//急停（禁止输出）
	}
	else
	{
		
	}
}

void renesas_height_cmd(u8 * data_array)
{
	switch(data_array[0])
	{
		case 0x01:													//设定期望高度
			height_command = 5;							//切换为指令控高模式
			my_except_height = data_array[1]*10.0f;		//设置期望高度
		break;
		
		case 0x02:													//起飞
			height_command = 4;
		break;
		
		case 0x03:													//降落
			height_command = 3;
		break;
		
		case 0x04:													//手动
			height_command = 1;
		break;
		
		default:
			
		break;
	}
}

void renesas_fly_cmd(u8 data)
{
	if(data == 1)				//悬停
	{
		ctrl_command = 3;
	}
	else if(data == 2)			//前进
	{
		ctrl_command = 4; 
	}
	else if(data == 3)			//后退降落
	{
		ctrl_command = 6;
	}
	else if(data == 4)			//跟踪
	{
		ctrl_command = 5;
	}
	else if(data == 5)			//手动
	{
		ctrl_command = 1;
	}
}

//数据输入接口
void renesas_data_receive_handle(u8 data)
{
	static u8 mode = 0;
	static u8 counter = 0;
	
	switch(mode)
	{
		case 0:
			if(data == 0xBB)
				mode = 1;
			else
				mode = 0;
		break;
			
		case 1:
			if(data == 0xE1)
				mode = 2;
			else
				mode = 0;
		break;
			
		case 2:
			if(data == 0x01)			//设置命令
			{
				mode = 11;
			}
			else if(data == 0x02)		//高度命令（2字节）
			{
				mode = 12;
				counter = 0;
			}
			else if(data == 0x03)		//飞行命令
			{
				mode = 13;
			}
			else
			{
				mode = 0;
			}
		break;
			
		case 11:						//设置命令
			renesas_setting_cmd(data);
			mode = 0;
		break;
		
		case 12:						//高度命令（2字节）
			tmpbuff[counter] = data;
			counter++;
			if(counter>=2)
			{
				counter = 0;
				renesas_height_cmd(tmpbuff);
				mode = 0;
			}
		break;
		
		case 13:						//飞行命令
			renesas_fly_cmd(data);
			mode = 0;
		break;
		
		default:
			mode = 0;
		break;
	}
}

//===============================================================================

u8 RenesasSendBuff[20] = {0};

void renesas_send_state(void)
{
	RenesasSendBuff[0] = 0xBB;
	RenesasSendBuff[1] = 0xEE;
	
	RenesasSendBuff[2] = 0x01;
	
	RenesasSendBuff[3] = fly_ready;
	RenesasSendBuff[4] = tracking_state;
	RenesasSendBuff[5] = All_Out_Switch;	//0：急停   1：输出
	RenesasSendBuff[6] = copter_fly_mode;
	RenesasSendBuff[7] = copter_height_mode;
	RenesasSendBuff[8] = (u8)(sonar_fusion.fusion_displacement.out/10.0f);
	
	Usart1_Send(RenesasSendBuff,9);	//发送到串口1
}

void renesas_data_send(void)
{
	renesas_send_state();
}


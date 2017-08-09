#include "track_mode.h"

#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"
#include "camera_datatransfer.h"
#include "camera_data_calculate.h"
#include "ctrl.h"
#include "mymath.h"
#include "ano_of.h"
#include "position_function.h"
#include "position_function_flow.h"
#include "height_function.h"
#include "fly_ctrl.h"

//====================================================================================================
//====================================================================================================

/*
	//姿态
	
	roll_speed,		//速度横滚控制模式	0：手动控制		1：摄像头数据定点		2：光流数据定点
	pitch_speed,	//速度俯仰控制模式	0：手动控制		1：摄像头数据定点		2：光流数据定点
	roll_position,	//位置横滚控制		0：输出0			1：输出摄像头计算偏移
	pitch_position,	//位置俯仰控制		0：输出0			1：输出摄像头计算偏移
	
	//航向
	
	yaw_mode;		//航向角控制			0：手动控制
*/

u8 copter_fly_mode = 0;			//飞行模式		0：黑圆定点		1：前进找车		2：移动物体跟随		3：降落		10.手动

//悬停
void Copter_Hover(float T)
{
	roll_speed = 2;
	pitch_speed = 2;
	
	roll_position = 1;
	pitch_position = 1;
}

//搜索
void Copter_Search(float T)
{
	
}

//跟随
void Copter_Track(float T)
{
	roll_speed = 2;
	pitch_speed = 2;
	
	roll_position = 1;
	pitch_position = 1;
}

//后退一些用于降落
void Copter_Back_To_Land(float T)
{
	
}

//手动
void Copter_Attitude_Hand(float T)
{
	roll_speed = 0;
	pitch_speed = 0;
	
	roll_position = 0;
	pitch_position = 0;
}


//====================================================================================================

void Fly_Mode_Ctrl(float T)		//飞行模式切换控制函数
{
	static u8 copter_fly_mode_old = 0;
	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	//这里要根据不同的指令对 copter_fly_mode 进行切换
	
	if(ctrl_command != 0)	//有新指令进入时ctrl_command不为0
	{
		if(ctrl_command == 1)
		{
			Copter_Attitude_Hand(T);	//手动
		}
		
		if(ctrl_command == 2)
		{
			Copter_Attitude_Hand(T);	//手动
		}
		
		if(ctrl_command == 3)
		{
			Copter_Hover(T);			//悬停
		}
		
		if(ctrl_command == 4)
		{
			Copter_Attitude_Hand(T);	//手动
		}

		if(ctrl_command == 5)
		{
			Copter_Attitude_Hand(T);	//手动
		}
		
		if(ctrl_command == 6)
		{
			Copter_Attitude_Hand(T);	//手动
		}
		
		//清零
		if(ctrl_command<=6)
		{
			ctrl_command = 0;	//外来指令在此清零
		}
	}
	
	//姿态控制模式切换
	switch(copter_fly_mode)
	{	
		case 0:
			Copter_Hover(T);
		break;
		
		case 1:
			Copter_Search(T);
		break;
		
		case 2:
			Copter_Track(T);
		break;
		
		case 3:
			Copter_Back_To_Land(T);
		break;
		
		case 10:
			Copter_Attitude_Hand(T);
		break;
		
		default:
			Copter_Attitude_Hand(T);
		break;
	}
	
	copter_fly_mode_old = copter_fly_mode;
}

//====================================================================================================
//====================================================================================================

/*

	//高度

	height_mode,		//高度控制模式		0：手动控高		1：锁定当前高度		2：根据指令高度控高		3：起飞		4：降落

*/


u8 copter_height_mode = 0;		//高度模式		0：定高			1：起飞			2：降落				10.手动

void Copter_Height_Lock(float T)	//0
{
	height_mode = 1;
}

void Copter_Take_Off(float T)		//1
{
	height_mode = 3;
}

void Copter_Land(float T)			//2
{
	height_mode = 4;
}

void Copter_Height_Hand(float T)	//10
{
	height_mode = 0;
}

//================================================================================================================

void Height_Mode_Ctrl(float T)		//高度模式切换控制函数
{
	static u8 copter_height_mode_old = 0;
	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	//高度控制模式切换
	
	if(height_command != 0)	//有新指令进入时 height_command 不为0
	{
		if(height_command == 1)
		{
			copter_height_mode = 10;	//手动
		}
		
		if(height_command == 2)
		{
			copter_height_mode = 0;		//定高
		}
		
		if(height_command == 3)
		{
			copter_height_mode = 2;		//降落
		}
		
		//清零
		if(height_command <= 3)
		{
			height_command = 0;	//外来指令在此清零
		}
	}
	
	//姿态控制模式切换
	switch(copter_height_mode)
	{	
		case 0:		//定高
			Copter_Height_Lock(T);
		break;
		
		case 1:		//起飞
			Copter_Take_Off(T);
		break;
		
		case 2:		//降落
			Copter_Land(T);
		break;
		
		case 10:	//手动
			Copter_Height_Hand(T);
		break;
		
		default:	//手动
			Copter_Height_Hand(T);
		break;
	}
	
	copter_height_mode_old = copter_height_mode;
}


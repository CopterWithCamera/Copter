#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"

float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值

void fly_ctrl(void)	//调用周期2ms
{
	uint8_t i;
	
	/*
	
	mode_state：
	0：手动
	1：气压计
	2：超声波+气压计
	3：自动
	
	ctrl_command：
	0：正常的手动飞行模式（超声波+气压计定高）
	1：高度锁定
	2：高度锁定+姿态归零
	3：降落模式
	
	*/
	
	//模式0 1 2都是手动飞行的模式，相当于遥控飞机
	//只有切换到模式3时，才会有自动控制介入
	if(mode_state == 0 || mode_state == 1 || mode_state == 2)	//	手动|气压计|超声波+气压计
	{
		
		//=================== filter ===================================
		//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
		//=================== filter =================================== 	
		
		//通道赋值拷贝
		for(i=0;i<CH_NUM;i++)
		{
			CH_ctrl[i] = CH_filter[i];	//CH_filter[i]为经过处理的信号输入值，来源可以是接收机，也可以是数传
		}
	}
	else if(mode_state == 3)	//自动（高度控制已经默认是超声波+气压计定高）
	{
		if(ctrl_command == 0)	//正常的手动飞行模式
		{
			CH_ctrl[0] = CH_filter[0];	//0：横滚
			CH_ctrl[1] = CH_filter[1];	//1：俯仰
			CH_ctrl[2] = CH_filter[2];	//2：油门
			CH_ctrl[3] = CH_filter[3];	//3：航向
		}
		else if(ctrl_command == 1)	//高度锁定
		{
			CH_ctrl[0] = CH_filter[0];	//0：横滚
			CH_ctrl[1] = CH_filter[1];	//1：俯仰
			CH_ctrl[3] = CH_filter[3];	//3：航向
			
			CH_ctrl[2] = 0;	//2：油门（油门位于中值，含义为高度保持）
		}
		else if(ctrl_command == 2)	//高度锁定+姿态归零
		{
			CH_ctrl[0] = 0;	//0：横滚
			CH_ctrl[1] = 0;	//1：俯仰
			CH_ctrl[3] = 0;	//3：航向
			CH_ctrl[2] = 0;	//2：油门（油门位于中值，含义为高度保持）
		}
		else if(ctrl_command == 3)	//匀速降落
		{
			CH_ctrl[0] = CH_filter[0];	//0：横滚
			CH_ctrl[1] = CH_filter[1];	//1：俯仰
			CH_ctrl[3] = CH_filter[3];	//3：航向
			
			CH_ctrl[2] = -100;	//2：油门（油门位于中下）
		}
	}
}


#include "fly_mode.h"
#include "rc.h"

extern float set_height_e;

u8 mode_state;
u8 ctrl_command;
void mode_check(float *ch_in)
{
	/*
	
	mode_state：
	0：手动
	1：气压计
	2：超声波+气压计
	3：自动
	
	AUX1（CH5）：
	低：0
	中：2
	高：3
	
	*/
	
	//根据AUX1通道（第5通道）的数值切换飞行模式
	if(*(ch_in+AUX1) <-350)				//-499 -- -350
	{
		mode_state = 0;	//手动油门
	}
	else if(*(ch_in+AUX1) < -150)		//-350 -- -150
	{
		mode_state = 2;	//超声波+气压计融合
	}
	else if(*(ch_in+AUX1) < 0)			//-150 -- 0
	{
		mode_state = 3;	//自动控制模式，有fly_ctrl.c中代码影响摇杆值
	}
	else if(*(ch_in+AUX1) < 150)		//0 -- 150
	{
		mode_state = 4;
	}
	else if(*(ch_in+AUX1) < 300)		//150 -- 300
	{
		mode_state = 5;
	}
	else								//300 -- 499
	{
		mode_state = 1;	//气压计定高
	}
	
	//根据AUX2通道（第6通道）的数值输入自动控制指令
	if(*(ch_in+AUX2) <-200)			//最低
	{
		ctrl_command = 0;
	}
	else if(*(ch_in+AUX2) <200)		//中间
	{
		ctrl_command = 3;
	}
	else							//最高
	{
		ctrl_command = 2;
	}
	
	//自动回位开关
	if(*(ch_in+AUX3) > 0)			//触发
	{
		set_height_e = 0;	//期望速度差归零
	}
	
}



//旧的模式判断代码，0 -- 手动，1 -- 气压计， 2 -- 超声波
//u8 mode_state_old;
//void mode_check(float *ch_in)
//{
//	//根据AUX1通道（第5通道）的数值切换飞行模式
//	if(*(ch_in+AUX1) <-200)			//最低
//	{
//		mode_state = 0;	//手动油门
//	}
//	else if(*(ch_in+AUX1) >200)		//中间
//	{
//		mode_state = 2;	//超声波+气压计融合
//	}
//	else							//最高
//	{
//		mode_state = 1;	//气压计定高
//	}
//	
//	//===========   ===========
//	mode_state_old = mode_state; //历史模式
//}

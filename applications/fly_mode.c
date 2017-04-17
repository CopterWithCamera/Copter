#include "fly_mode.h"
#include "fly_ctrl.h"
#include "rc.h"
#include "ultrasonic.h"

u8 mode_state;
void mode_check(float *ch_in)
{
	/*
	mode_state：
	0：手动
	1：气压计
	2：超声波+气压计
	3：自动
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
		if(ultra.measure_ok == 1)		//只有在超声波传感器有数据时才允许自动控制（严重依赖超声波）
		{
			mode_state = 3;	//自动控制模式，有fly_ctrl.c中代码影响摇杆值
		}
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
	
	//fly_ctrl用的飞行模式控制，在mode_state=3时起作用
	Ctrl_Mode(ch_in);
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

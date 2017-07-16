#include "fly_mode.h"
#include "fly_ctrl.h"
#include "rc.h"
#include "ultrasonic.h"

u8 mode_state;
void mode_check(float *ch_in)
{
	/*
	mode_state:
	0：手动
	1：气压计
	2：超声波+气压计
	3：自动
	*/
	
	//根据AUX1通道（第5通道）的数值切换飞行模式
	if(*(ch_in+AUX1) < -150)			//-499 -- -150
	{
		mode_state = 0;	//手动油门
	}
	else if(*(ch_in+AUX1) < 150)		//-150 -- +150
	{
		mode_state = 2;	//超声波+气压计融合
	}
	else								//+150 -- +499
	{
		if(ultra.measure_ok == 1)		//只有在超声波传感器有数据时才允许自动控制（严重依赖超声波）
		{
			mode_state = 3;	//自动控制模式，有fly_ctrl.c中代码影响摇杆值
		}
	}
}


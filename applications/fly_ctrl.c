#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"

//定义辅助通道对应飞行模式的宏

#define	FUNCTION_1					hand_ctrl()
#define FUNCTION_2					height_lock()
#define FUNCTION_3					fly_ctrl_land()


//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter ===================================

float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值

//手飞
void hand_ctrl(void)
{
	//手飞模式下俯仰和横滚加死区
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[2] = CH_filter[2];	//2：油门 THR
	CH_ctrl[3] = CH_filter[3];	//3：航向 YAW
}

//高度锁定
void height_lock(void)
{
	CH_ctrl[0] = CH_filter[0];	//0：横滚
	CH_ctrl[1] = CH_filter[1];	//1：俯仰
	CH_ctrl[3] = CH_filter[3];	//3：航向

	CH_ctrl[2] = 0;	//2：油门（油门位于中值，含义为高度保持）

}

//高度姿态锁定
void height_attitude_lock(void)
{
	CH_ctrl[0] = 0;	//0：横滚
	CH_ctrl[1] = 0;	//1：俯仰
	CH_ctrl[3] = 0;	//3：航向
	CH_ctrl[2] = 0;	//2：油门（油门位于中值，含义为高度保持）
}

//降落
float height_speed_ctrl = 0;
void fly_ctrl_land(void)	//调用周期2ms
{
	static u16 lock_time = 0;
	
	//降落函数只可在起飞后调用
	
	//从其他模式切换到降落模式
	if(ctrl_command != ctrl_command_old)
	{
		set_height_e = 0;	//期望速度差归零
	}
	
	//俯仰、横滚、航向轴正常
	CH_ctrl[0] = CH_filter[0];	//0：横滚
	CH_ctrl[1] = CH_filter[1];	//1：俯仰
	CH_ctrl[3] = CH_filter[3];	//3：航向

//	//摇杆控制（转化为速度期望）
//	CH_ctrl[2] = -60;	//-50对应的大约是0.3m/s，-100大约是0.6m/s，具体数值通过实验确定
	
	//设定期望垂直速度
	height_speed_ctrl = -400;	//单位mm/s
	CH_ctrl[2] = 0;				//油门值拉最低，转速输出函数检测到油门低时才允许螺旋桨停转
	
	if(ultra.relative_height < 5)	//当前飞机在地面时超声波数据是3-4cm（和具体机型有关）
	{
		height_speed_ctrl = -2000;	//切换为最大下降速度（快速通过积分让螺旋桨停转）
		
		lock_time++;			//计数器累加
		if(lock_time > 1000)	//2s
		{
			fly_ready = 0;	//锁定
		}
	}
	
}

//起飞
u8 takeoff_allow = 0;	//允许起飞标志，允许时可以运行起飞控制函数；1 -- 可以起飞，0 -- 起飞完成或不能起飞
void fly_ctrl_takeoff(void)	//调用周期2ms
{
	static u16 counter = 0;
	
	if(fly_ready == 1)	//已经解锁后，起飞代码才能执行
	{
		if(takeoff_allow == 1)	//允许起飞
		{
			if(thr_take_off_f == 0)
			{
				thr_take_off_f = 1;	//起飞标志位置1（高度控制代码只有在起飞标志位置1时才会运行）
				thr_take_off = 500; //直接赋值起飞基准油门
			}
			
			height_speed_ctrl = 500;	//500mm/s上升
			
			if(ultra.relative_height > 35)	//超过30cm（传感器离地面4-5cm）
			{
				counter++;	//每2ms累加1次
				
				if(counter > 150)	//0.3s 认为已经稳定超过30cm，排除错误数据影响
				{
					takeoff_allow = 0;	//起飞完毕
					counter = 0;
				}
			}
			else
			{
				counter = 0;
			}
		}
		else	//不允许起飞（已经在飞行）或起飞流程执行完成
		{
			height_lock();	//高度锁定模式，定高等待
		}
	}
}

//========================================================================================================

//识别控制指令（在mode_check函数中调用，处理辅助通道数值）
u8 ctrl_command;
u8 ctrl_command_old;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{
	//更新历史模式
	ctrl_command_old = ctrl_command;
	
	//根据AUX2通道（第6通道）的数值输入自动控制指令
	if(*(ch_in+AUX2) <-200)		//三挡开关在最上
	{
		ctrl_command = 1;
	}
	else if(*(ch_in+AUX2) <200)		//三挡开关在中间
	{
		ctrl_command = 2;	
	}
	else							//三挡开关在最下
	{
		ctrl_command = 3;	
	}
	
	//自动回位开关
	if(*(ch_in+AUX3) > 0)			//触发
	{
		set_height_e = 0;	//期望速度差归零
	}
	
	//
	if(*(ch_in+AUX4) > 0)			//SWA,输出使能开关，只有在开关播到上方时输出才能被输出到PWM
	{
		All_Out_Switch = 0;
	}
	else
	{
		All_Out_Switch = 1;
	}
}

/* ************************************************************
	飞行自动控制函数

	根据ctrl_command调用不同的自动控制函数

	mode_state：
	0：手动				1：气压计
	2：超声波+气压计		3：自动

	ctrl_command：
	0：正常的手动飞行模式（超声波+气压计定高）		1：高度锁定
	2：高度锁定+姿态归零							3：降落模式
												
*************************************************************** */

void Fly_Ctrl(void)	
{
	
	//调用周期5ms
	
	if(mode_state != 3)
	{
		//只有自动模式才会执行自动控制代码
		return;
	}
	
	//在上锁时认为已经降落，清除一些标志位，为下次起飞做准备
	if(fly_ready == 0)
	{
		takeoff_allow = 1;	//允许起飞（只有没有离地前才允许执行起飞函数）
	}
	
	switch(ctrl_command)
	{
		case 1:
			FUNCTION_1;
		break;
		
		case 2:
			FUNCTION_2;
		break;
		
		case 3:
			FUNCTION_3;
		break;
		
		default:
			hand_ctrl();	//默认（意外）情况下手飞
		break;
	}

}


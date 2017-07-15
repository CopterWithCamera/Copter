#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"

//定义辅助通道对应飞行模式的宏

#define	FUNCTION_1					hand_ctrl
#define FUNCTION_2					height_lock
#define FUNCTION_3					height_lock_displacement


//=================== filter ===================================
//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
//=================== filter ===================================

float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值

u8 my_height_mode = 0;	//模式使用的定高模式
						//0：油门
						//1：期望高度
						//2：期望速度
						
float my_except_height = 0;//期望高度

//手飞
void hand_ctrl(void)
{
	my_height_mode = 0;		//输入油门值
	
	//手飞模式下俯仰和横滚加死区
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[2] = CH_filter[2];	//2：油门 THR
	CH_ctrl[3] = CH_filter[3];	//3：航向 YAW
}

//高度锁定（遥控器归中值）
void height_lock(void)
{
	my_height_mode = 0;		//输入油门值
	
	CH_ctrl[0] = CH_filter[0];	//0：横滚
	CH_ctrl[1] = CH_filter[1];	//1：俯仰
	CH_ctrl[3] = CH_filter[3];	//3：航向

	CH_ctrl[2] = 0;	//2：油门（油门位于中值，含义为高度保持）

}

//直接输入期望高度定高
void height_lock_displacement()
{
	static u8 flag;
	my_height_mode = 1;		//输入目标高度差
	
	if(!flag)
	{
		flag = 1;
		my_except_height = sonar_fusion.fusion_displacement.out;	//读取当前高度
	}
	
	CH_ctrl[0] = CH_filter[0];	//0：横滚
	CH_ctrl[1] = CH_filter[1];	//1：俯仰
	CH_ctrl[3] = CH_filter[3];	//3：航向
}

//高度姿态锁定
void height_attitude_lock(void)
{
	my_height_mode = 0;
	
	CH_ctrl[0] = 0;	//0：横滚
	CH_ctrl[1] = 0;	//1：俯仰
	CH_ctrl[3] = 0;	//3：航向
	CH_ctrl[2] = 0;	//2：油门（油门位于中值，含义为高度保持）
}


//========================================================================================================




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

void Fly_Ctrl(void)		//调用周期5ms
{
	static u8 height_lock_flag = 0;
	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	//==================== 新飞行控制逻辑 ==========================
	
	/*
	
	飞行控制的主要任务有两个：
	
	1.姿态控制，处理横滚、俯仰、航向三轴
		CH_ctrl[0] = CH_filter[0];	//0：横滚
		CH_ctrl[1] = CH_filter[1];	//1：俯仰
		CH_ctrl[3] = CH_filter[3];	//3：航向
	
	2.高度控制
		my_height_mode = 0		油门输入模式
		使用 CH_filter[THR] 输入油门值，取值范围 -500 -- +500
	
		my_height_mode = 1		期望高度模式
		使用 my_except_height 变量控制高度，单位为mm	
	
	*/
	
	/* ********************* 高度控制 ********************* */
	
	if(ctrl_command == 0)
	{
		//摇杆控高
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2：油门 THR
		height_lock_flag = 0;
	}
	else if(ctrl_command == 1)
	{
		//期望高度控制高度
		my_height_mode = 1;
		
		if(height_lock_flag == 0)
		{
			height_lock_flag = 1;
			
			//设置期望高度
			my_except_height = sonar_fusion.fusion_displacement.out;	//读取当前高度
		}
	}
	else if(ctrl_command == 2)
	{
		//期望高度控制高度
		my_height_mode = 1;

		//设置期望高度
		my_except_height = 100;		//下降到100mm = 10cm
	}
	else if(ctrl_command == 3)
	{
		
	}
	else if(ctrl_command == 4)
	{
		
	}
	else if(ctrl_command == 5)
	{
		
	}
	else	//应急模式用手动控制油门
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2：油门 THR
		
		height_lock_flag = 0;
	}
	
	
	/* ********************* 姿态控制 ********************* */
	
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[3] = CH_filter[3];	//3：航向 YAW
	

}

//识别控制指令（在mode_check函数中调用，处理辅助通道数值）
u8 ctrl_command;
u8 ctrl_command_old;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{	
	//更新历史模式
	ctrl_command_old = ctrl_command;
	
	//根据AUX2通道（第6通道）的数值输入自动控制指令
	if(*(ch_in+AUX2) < -350)			//-499 -- -350
	{
		ctrl_command = 0;
	}
	else if(*(ch_in+AUX2) < -150)		//-350 -- -150
	{
		ctrl_command = 1;
	}
	else if(*(ch_in+AUX2) < 0)			//-150 -- 0
	{
		ctrl_command = 2;
	}
	else if(*(ch_in+AUX2) < 150)		//0 -- 150
	{
		ctrl_command = 3;
	}
	else if(*(ch_in+AUX2) < 350)		//150 -- 350				
	{
		ctrl_command = 4;
	}
	else								//350 -- 499
	{
		ctrl_command = 5;
	}
	
	//自动回位开关
	if(*(ch_in+AUX3) > 0)			//触发
	{
		set_height_e = 0;	//期望速度差归零
	}
	
	//急停功能
	//All_Out_Switch = 0时急停，All_Out_Switch = 1时正常运行
	if(*(ch_in+AUX4) > 0)			//SWA,输出使能开关，只有在开关播到上方时输出才能被输出到PWM
	{
		//禁止输出
		All_Out_Switch = 0;
	}
	else
	{
		//允许输出
		All_Out_Switch = 1;
	}
}

//==================== 旧飞行控制逻辑 ==========================
	
//	switch(ctrl_command)
//	{
//		case 1:
//			FUNCTION_1();
//		break;
//		
//		case 2:
//			FUNCTION_2();
//		break;
//		
//		case 3:
//			FUNCTION_3();
//		break;
//		
//		default:
//			hand_ctrl();	//意外情况下手飞
//		break;
//	}


#include "fly_ctrl.h"
#include "rc.h"
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

float CH_ctrl[CH_NUM];	//具体输入给ctrl的遥控器值
float my_except_height = 0;//期望高度
u8 my_height_mode = 0;	//模式使用的定高模式
						//0：油门
						//1：期望高度
						//2：期望速度

//========================================================================================
//========================================================================================
//										指令接收
//========================================================================================
//========================================================================================

void set_except_height(u8 height)	//地面站传入高度数据，单位cm，取值范围0cm-255cm
{
	my_except_height = height * 10;
}

void set_attitude_calibration(u8 cmd)	//姿态仪调整指令
{
	switch(cmd)
	{
		//前后 -- 俯仰 -- y轴
		
		case 0x01:	//前
			mpu6050.vec_3d_cali.y = mpu6050.vec_3d_cali.y + 0.1f;
		break;
		
		case 0x02:	//后
			mpu6050.vec_3d_cali.y = mpu6050.vec_3d_cali.y - 0.1f;
		break;
		
		//左右 -- 横滚 -- x轴
		
		case 0x03:	//左
			mpu6050.vec_3d_cali.x = mpu6050.vec_3d_cali.x + 0.1f;
		break;
		
		case 0x04:	//右
			mpu6050.vec_3d_cali.x = mpu6050.vec_3d_cali.x - 0.1f;
		break;
		
		//存储
		
		case 0x05:	//存储
			flash_save_en_cnt = 1;	//存储使能
		break;
		
		//清零
		
		case 0x06:	//清零（不存储）
			mpu6050.vec_3d_cali.x = 0;
			mpu6050.vec_3d_cali.y = 0;
		break;
		
		default:
			
		break;
	}
}

void set_all_out_switch(u8 cmd)		//电机输出控制指令
{
	switch(cmd)	//第一个数据（u8）
	{
		case 0x01:	//急停
			All_Out_Switch = 0;	//禁止输出
		break;
			
		case 0x02:	//解急停
			All_Out_Switch = 1;	//允许输出
		break;
			
		default:
			
		break;
	}
}

/************************************************************************************
						多频率联合自动控制函数

	mode_state：	0：手动		1：气压计	2：超声波+气压计		3：自动

*************************************************************************************/

u8	height_mode = 0,	//高度控制模式		0：手动控高		1：锁定当前高度		2：根据指令高度控高		3：起飞				4：降落
	roll_speed = 0,		//速度横滚控制模式	0：手动控制		1：摄像头数据定点												4：光流数据定点
	pitch_speed = 0,	//速度俯仰控制模式	0：手动控制		1：摄像头数据定点	2：摄像头数据前进		3：摄像头数据后退	4：光流数据定点		5：光流数据前进		6：光流数据后退
	roll_position = 0,	//位置横滚控制		0：输出0			1：输出摄像头计算偏移
	pitch_position = 0;	//位置俯仰控制		0：输出0			1：输出摄像头计算偏移

//飞行模式切换控制函数
void Fly_Mode_Ctrl(float T)
{
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	switch(height_command)
	{
		case 0:
			height_mode = 0;
		break;
		
		case 1:
			height_mode = 1;
		break;
		
		case 2:
			height_mode = 4;
		break;
		
		default:
			height_mode = 0;
		break;
	}
	
	switch(ctrl_command)
	{
		case 0:
			roll_position = 0;	//手飞
			roll_speed = 0;
			pitch_position = 0;
			pitch_speed = 0;
		break;
		
		case 1:
			pitch_position = 0;	//俯仰手飞
			pitch_speed = 0;
			roll_position = 1;	//横滚自动
			roll_speed = 1;
		break;
		
		case 2:
			pitch_position = 1;	//俯仰自动
			pitch_speed = 1;
			roll_position = 1;	//横滚自动
			roll_speed = 1;
		break;
		
		case 3:
			pitch_position = 0;	//俯仰手动
			pitch_speed = 0;
			roll_position = 1;	//横滚光流自动
			roll_speed = 4;
		break;
		
		case 4:
			pitch_position = 1;	//俯仰光流自动
			pitch_speed = 4;
			roll_position = 1;	//横滚光流自动
			roll_speed = 4;
		break;
		
		case 5:
			
		break;
		
		case 6:
			
		break;
		
		default:
			
		break;
	}
	
	
}

void Fly_Height_Ctrl(float T)	//高度控制函数
{
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	switch(height_mode)
	{
		case 0:
			hand();			//手动
		break;
		
		case 1:
			height_lock();	//锁定当前高度
		break;
		
		case 2:
			height_hold();	//根据高度期望控高
		break;
		
		case 3:
			take_off(T);	//起飞
		break;
		
		case 4:
			land();			//降落
		break;
		
		default:
			hand();
		break;
	}
	
	//函数清零
	if(height_mode != 1)
		height_lock_clear();
}

void Fly_Ctrl(float T)		//调用周期5ms
{
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	/* ********************* 综合控制 ********************* */
	
	if(roll_speed == 0 && pitch_speed == 0)
	{
		attitude_hand();
	}
	
	if( roll_position == 0 )
	{
		position_roll_zero();
	}
	
	if( pitch_position == 0 )
	{
		position_pitch_zero();
	}
}

void Fly_Ctrl_Cam(float T)		//调用周期与camera数据相同
{	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	//位置控制
	
	if( roll_position == 1 )	//摄像头位置输出
	{
		position_roll(T);
	}
	
	if( pitch_position == 1 )	//摄像头位置输出
	{
		position_pitch(T);
	}
	
	//速度控制
	
	if( roll_speed == 1 )	//摄像头定点
	{
		speed_roll();
	}
	
	if( pitch_speed == 1 )	//摄像头定点
	{
		speed_pitch();
	}
	
	//函数清零
	
	if( roll_position == 0 )
	{
		position_roll_clear();
	}
	
	if( pitch_position == 0 )
	{
		position_pitch_clear();
	}
	
	if( roll_speed == 0 )
	{
		speed_roll_clear();
	}
	
	if( pitch_speed == 0 )
	{
		speed_pitch_clear();
	}
}

void Fly_Ctrl_Flow(void)		//调用周期与camera数据相同
{	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
	if(roll_speed == 2)	//光流定点
	{
		speed_flow_roll();
	}
	
	if(pitch_speed == 2)	//光流定点
	{
		speed_flow_pitch();
	}
	
	//函数清零
	
	if(roll_speed == 0)	//光流定点
	{
		speed_flow_roll_clear();
	}
	
	if(pitch_speed == 0)	//光流定点
	{
		speed_flow_pitch_clear();
	}
}


//========================================================================================
//========================================================================================
//				   识别控制指令（在mode_check函数中调用，处理辅助通道数值）
//========================================================================================
//========================================================================================
u8 ctrl_command;
u8 height_command;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{
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
	
	//根据AUX3通道控制高度
	if(*(ch_in+AUX3) < -150)	
	{
		height_command = 0;
	}
	else if(*(ch_in+AUX3) < 150)
	{
		height_command = 1;
	}
	else
	{
		height_command = 2;
	}
	
	//电机输出使能
	//All_Out_Switch = 0时急停，All_Out_Switch = 1时正常运行
	//*(ch_in+AUX4)>0时开关被扳动到下方，发出禁止输出指令
	static u8 out_flag = 0;
	static u8 stop_flag = 0;
	if(*(ch_in+AUX4) > 0)			//SWA,输出使能开关，只有在开关播到上方时输出才能被输出到PWM
	{
		out_flag = 0;	//清out_flag
		
		if(stop_flag == 0)
		{
			stop_flag = 1;
			All_Out_Switch = 0;	//禁止输出
		}
		
	}
	else
	{
		stop_flag = 0;	//清stop_flag
		
		if(out_flag == 0)
		{
			out_flag = 1;
			All_Out_Switch = 1;	//允许输出
		}
	}
}

//****************************************************************************************

//========================================================================================
//========================================================================================
//	飞行自动控制函数
//
//	根据ctrl_command调用不同的自动控制函数
//
//	mode_state：
//	0：手动				1：气压计
//	2：超声波+气压计		3：自动
//
//	height_command：
//	0：手动控高			1：定高
//	2：降落
//========================================================================================
//========================================================================================

//void Fly_Ctrl(void)		//调用周期5ms
//{
//	//只有自动模式才会执行自动控制代码
//	if(mode_state != 3)
//	{
//		return;
//	}

////===========================================================
////==================== 飞行控制逻辑 ==========================
////===========================================================
//	
//	/*
//	
//	飞行控制的主要任务有两个：
//	
//	1.姿态控制，处理横滚、俯仰、航向三轴
//		CH_ctrl[0] = CH_filter[0];	//0：横滚
//		CH_ctrl[1] = CH_filter[1];	//1：俯仰
//		CH_ctrl[3] = CH_filter[3];	//3：航向
//	
//	2.高度控制
//		my_height_mode = 0		油门输入模式
//		使用 CH_filter[THR] 输入油门值，取值范围 -500 -- +500
//	
//		my_height_mode = 1		期望高度模式
//		使用 my_except_height 变量控制高度，单位为mm	
//	
//	*/
//	
///* ********************* 高度控制 ********************* */
//	
//	if(height_command == 0)
//	{
//		hand();
//	}
//	
//	if(height_command == 1)
//	{
//		height_lock(1);	//锁定高度
//	}
//	else
//	{
//		height_lock(0);	//清除标志位
//	}
//	
//	if(height_command == 2)
//	{
//		land();	//降落模式
//	}
//	
//	//意外状况处理
//	if(height_command > 2)	//不应该出现的情况
//	{
//		my_height_mode = 0;
//		CH_ctrl[2] = CH_filter[2];	//2：油门 THR
//	}
//	
///* ********************* 姿态控制 ********************* */
//	
//	if(ctrl_command == 0)
//	{
//		attitude_hand();
//	}
//	
//	if(ctrl_command == 1)
//	{
//		attitude_hand();
//	}
//	
//	if(ctrl_command == 2)
//	{
//		attitude_hand();
//	}

//	//意外状况处理
//	if(ctrl_command > 5)
//	{
//		attitude_hand();
//	}
//	
//}

//	if(height_command == 0)
//	{
//		hand();
//	}
//	
//	if(height_command == 1)
//	{
//		height_lock(1);	//锁定高度
//	}
//	else
//	{
//		height_lock(0);	//清除标志位
//	}
//	
//	if(height_command == 2)
//	{
//		land();	//降落模式
//	}

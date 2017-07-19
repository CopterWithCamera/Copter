#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"
#include "camera_datatransfer.h"
#include "ctrl.h"

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

void set_except_height(u8 height)	//传入高度数据，单位cm，取值范围0cm-255cm
{
	my_except_height = height * 10;
}

void set_attitude_calibration(u8 cmd)
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
		
		default:
			
		break;
	}
}


//========================================================================================
//========================================================================================
//										高度控制
//========================================================================================
//========================================================================================

//手飞模式
void hand(void)
{
	//=================== filter ===================================
	//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
	//=================== filter ===================================

	//摇杆控高
	my_height_mode = 0;
	CH_ctrl[2] = CH_filter[2];	//2：油门 THR
	
	if(NS==0) //丢失信号
	{
		CH_ctrl[2] = LIMIT(CH_ctrl[2],-499,0);	//保持当前油门值，但不能超过半油门，500这个数在定高代码里代表悬停
												//也就是说定高模式丢信号时只能悬停或下降（依照丢信号前状态）
	}
	
	//依旧由油门控制Thr_Low位
	//thr取值范围0-1000，改为CH_filter后取值范围+-500
	if( CH_ctrl[2] < -400 )	//油门低判断（用于 ALL_Out 里的最低转速保护 和 ctrl2 里的Yaw轴起飞前处理）
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
}

//锁定当前高度（进入模式时锁定当前高度，并接受上位机位置控制）
void height_lock(u8 en)	//en -- 模式启用标志位，用于判断此模式是否被使用
{
	static u8 height_lock_flag = 0;
	
	if(en)
	{
		//高度控制模式被启用
		my_height_mode = 1;
		
		Thr_Low = 0;	//油门低标志置0，防止螺旋桨意外停转
		
		if(height_lock_flag == 0)
		{
			height_lock_flag = 1;
			
			//设置期望高度
			my_except_height = sonar_fusion.fusion_displacement.out;	//读取当前高度
			
			if( my_except_height < 150)		//容错处理，防止期望高度过低
				my_except_height = 150;
		}
		
	}
	else
	{
		height_lock_flag = 0;
	}
}

//下降到15cm
void falling_to_15cm(void)	//这个高度相当于降落了（起落架大约比15cm短一点点）
{
	Thr_Low = 1;	//油门低标志为1，允许螺旋桨停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 150;		//下降到100mm = 10cm
}

//上升到50cm
void rising_to_50cm(void)
{
	Thr_Low = 0;	//油门低标志置0，防止螺旋桨意外停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 500;		//下降到 150cm
}

//========================================================================================
//========================================================================================
//									姿态（位置）控制函数
//					可以使用的参数：float length、float angle、float speed;
//========================================================================================
//========================================================================================

//手动控制姿态
void attitude_hand(void)
{
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[3] = CH_filter[3];	//3：航向 YAW
}

//横滚角乒乓控制
void attitude_pingpong(void)
{
	//横滚自动控制
	if(length > 20)
	{
		CH_ctrl[0] = -5;
	}
	else if(length < -20)
	{
		CH_ctrl[0] = 5;
	}
	else
	{
		CH_ctrl[0] = 0;
	}
	
	//俯仰和航向手动控制
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[3] = CH_filter[3];	//3：航向 YAW
}

//根据偏移量进行单P调整
void attitude_single_p(void)
{

}

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
//	ctrl_command：
//	0：正常的手动飞行模式（超声波+气压计定高）		1：高度锁定
//	2：高度锁定+姿态归零							3：降落模式												
//========================================================================================
//========================================================================================

void Fly_Ctrl(void)		//调用周期5ms
{	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}

//===========================================================
//==================== 飞行控制逻辑 ==========================
//===========================================================
	
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
	
	if(height_command == 0)
	{
		hand();
	}
	
	if(height_command == 1)
	{
		height_lock(1);	//锁定高度
	}
	else
	{
		height_lock(0);	//清除标志位
	}
	
	if(height_command == 2)
	{
		falling_to_15cm();
	}
	
	//意外状况处理
	if(height_command > 2)	//不应该出现的情况
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2：油门 THR
	}
	
/* ********************* 姿态控制 ********************* */
	
	//指令1
	if(ctrl_command == 0)
	{
		attitude_hand();
	}
	
	//指令2
	if(ctrl_command == 1)
	{
		attitude_pingpong();	//横滚角乒乓控制
	}
	
	//指令3
	if(ctrl_command == 2)
	{
		attitude_hand();
	}
	
	//指令4
	if(ctrl_command == 3)
	{
		attitude_hand();
	}
	
	//指令5
	if(ctrl_command == 4)
	{
		attitude_hand();
	}
	
	//指令6
	if(ctrl_command == 5)
	{
		attitude_hand();
	}
	
	//意外状况处理
	if(ctrl_command > 5)
	{
		attitude_hand();
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


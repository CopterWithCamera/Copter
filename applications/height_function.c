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
#include "height_function.h"

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
	CH_ctrl[THR] = CH_filter[THR];	//2：油门 THR
	
	if(NS==0) //丢失信号
	{
		CH_ctrl[THR] = LIMIT(CH_ctrl[THR],-499,0);	//保持当前油门值，但不能超过半油门，500这个数在定高代码里代表悬停
												//也就是说定高模式丢信号时只能悬停或下降（依照丢信号前状态）
	}
	
	//依旧由油门控制Thr_Low位
	//thr取值范围0-1000，改为CH_filter后取值范围+-500
	if( CH_ctrl[THR] < -400 )	//油门低判断（用于 ALL_Out 里的最低转速保护 和 ctrl2 里的Yaw轴起飞前处理）
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
			
			#if (HEIGHT_SOURCE == 1)
				my_except_height = sonar_fusion.fusion_displacement.out;	//读取当前高度
			#elif (HEIGHT_SOURCE == 2)
				my_except_height = sonar.displacement;						//读取当前高度
			#endif
			
			if( my_except_height < 150)		//容错处理，防止期望高度过低
				my_except_height = 150;
		}
	}
	else
	{
		height_lock_flag = 0;
	}
}

//降落
void land(void)	//这个高度相当于降落了（起落架大约比15cm短一点点）
{
	Thr_Low = 1;	//油门低标志为1，允许螺旋桨停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 100;		//下降到100mm = 10cm
	
	if(sonar.displacement < 140)	//小于14cm
	{
		fly_ready = 0;	//锁定
	}
}

//********************************************************************************************

////下降到15cm
//void falling_to_15cm(void)	//这个高度相当于降落了（起落架大约比15cm短一点点）
//{
//	Thr_Low = 1;	//油门低标志为1，允许螺旋桨停转
//	
//	//期望高度控制高度
//	my_height_mode = 1;

//	//设置期望高度
//	my_except_height = 150;		//下降到150mm = 15cm
//}

////上升到50cm
//void rising_to_50cm(void)
//{
//	Thr_Low = 0;	//油门低标志置0，防止螺旋桨意外停转
//	
//	//期望高度控制高度
//	my_height_mode = 1;

//	//设置期望高度
//	my_except_height = 500;		//上升到50cm
//}

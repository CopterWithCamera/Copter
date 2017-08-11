#include "height_function.h"
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
#include "track_mode.h"


//========================================================================================
//========================================================================================
//										高度控制
//========================================================================================
//========================================================================================

//0.手飞模式
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

//1.1锁定当前高度（进入模式时锁定当前高度，并接受上位机位置控制）
u8 height_lock_flag = 0;
void height_lock()	//en -- 模式启用标志位，用于判断此模式是否被使用
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

//1.2定高清零
void height_lock_clear(void)
{
	height_lock_flag = 0;
}

//2.根据指令控高
void height_hold(void)	//
{
	Thr_Low = 0;	//油门低标志为0
	
	//期望高度控制高度
	my_height_mode = 1;

	//防止期望高度过低
	if(my_except_height < 150)
		my_except_height = 150;		//15cm
}

//3.本函数输出自动起飞期间的油门值，输出值取值范围（-499 -- +499）
//此模式按时间运行，有延时功能
u8 auto_take_off;	//自动起飞标志位
void take_off(float dT)	//dT单位是s
{
	static float thr_auto = 0.0f;
	static float time_counter = 0;
	
	my_height_mode = 0;		//摇杆控制高度
	
	if(fly_ready==0)	//没解锁时期望高度为当前高度
	{
		Thr_Low = 1;	//油门低标志为1，允许停转
		CH_ctrl[THR] = -499;	//油门最低
		auto_take_off = 0;	//解锁之前自动起飞标志位为0（只有上锁状态才能让auto_take_off归零，防止起飞期间反复调用此函数）
		return;
	}
	
	Thr_Low = 0;	//油门低标志为0，最低转速保护
	
	if( auto_take_off == 0)
	{
		auto_take_off = 2;	//已经解锁，且没有开始自动起飞则开始自动起飞，数值1位预留的起飞前准备模式
	}
	
	#if defined(__COPTER_NO1)
	
		if(auto_take_off == 2)	//看来这个变量的意思是要把油门一下子拉到200（范围是-500 -- +500），这个数是70%的油门
		{
			thr_auto = 300;		//设定起飞油门
			auto_take_off = 3;	//开始控制油门
		}
		else if(auto_take_off == 3)	//然后根据时间一点点的让油门下降（这个函数调用频率是2ms，0.002*200 = 0.4，500*0.4=200），1s之后这个油门清零
		{
			if(thr_auto > 0.0f)
			{
				thr_auto -= 180 *dT;	//油门缓慢缩小，在2ms调用周期下1.666s中后此变量归零
			}
			else
			{
				auto_take_off = 4;
				time_counter = 1.0;
			}
		}
		else if(auto_take_off == 4)		//1s整，然后锁定当前高度
		{
			thr_auto = 50;	//thr的死区在+-40，40以上才有效
			
			time_counter -= dT;
			
			if(time_counter<0)
			{
				auto_take_off = 5;
			}
		}
		else
		{
			height_command = 2;		//给出定高指令
		}
		
		thr_auto = LIMIT(thr_auto,0,300);	//0代表悬停，300是限制最高值
	
	#elif defined(__COPTER_NO2)
		
		if(auto_take_off == 2)	//看来这个变量的意思是要把油门一下子拉到200（范围是-500 -- +500），这个数是70%的油门
		{
			thr_auto = 350;		//设定起飞油门
			auto_take_off = 3;	//开始控制油门
		}
		else if(auto_take_off == 3)	//然后根据时间一点点的让油门下降（这个函数调用频率是2ms，0.002*200 = 0.4，500*0.4=200），1s之后这个油门清零
		{
			if(thr_auto > 0.0f)
			{
				thr_auto -= 180 *dT;	//油门缓慢缩小，在2ms调用周期下1.666s中后此变量归零
			}
			else
			{
				auto_take_off = 4;
				time_counter = 1.0;
			}
		}
		else if(auto_take_off == 4)		//1s整，然后锁定当前高度
		{
			thr_auto = 100;	//thr的死区在+-40，40以上才有效
			
			time_counter -= dT;
			
			if(time_counter<0)
			{
				auto_take_off = 5;
			}
		}
		else
		{
			height_command = 2;		//给出定高指令
		}
		
		thr_auto = LIMIT(thr_auto,0,300);	//0代表悬停，300是限制最高值
	
	#endif

	CH_ctrl[THR] = thr_auto;	//输出值传给油门
	
	mydata.d14 = (u16)thr_auto;
}

//4.降落
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

#include "position_function_flow.h"

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


//速度控制
static float speed_integration_pitch = 0.0f;	//积分变量
void speed_flow_pitch()
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	static float speed_error_old = 0.0f;	//old变量
	static u8 d_stop_flag = 0;		//停止d运算的标志位，表示speed_error_old数值无效
	s32 out_tmp;
	
	//接口：except_speed      + <-- --> -      单位cm/s
	
	/*
		speed_d_bias_pitch			速度值			+ <前---  ---后> -
		speed_d_bias_lpf_pitch		lpf值			+ <前---  ---后> -
	
		CH_filter[1]				遥控器俯仰输入	- <前---  ---后> +
	*/
	
	//模式使能
	
	//except_speed_pitch      + <-- --> -      单位cm/s
	
	except_speed = position_pitch_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	except_speed = LIMIT(except_speed,-30,30);			//限幅（速度调整要求平稳）
	
	if( OF_QUA < 20 )		//数据可信度低
	{
		// bias_detect（水平偏差）值异常处理
		
		//偏移过大，使用乒乓控制，系数对应 param_A param_B
		//根据期望值处理
		
		if( except_speed > 1.0f )	//速度期望向前
		{
			p_out = -20.0f * pid_setup.groups.ctrl6.kp;	//前飞
		}
		else if(except_speed < -1.0f)	//速度期望向后
		{
			p_out =  20.0f * pid_setup.groups.ctrl6.ki; //后飞
		}
		else						
		{
			p_out = 0.0f;	//中间设置死区
		}

		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		d_stop_flag = 1;	//表示speed_error_old无效，无法进行d运算
	}
	else
	{
		//bias_detect值正常
		
		speed_error = except_speed - OF_DY2FIX_DETECT;			//计算error   speed_error值
																//error   负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
																//		  正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速
		
		//p
		p_out = - speed_error * user_parameter.groups.param_D; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * user_parameter.groups.param_E; //user_parameter.groups.self_def_1.ki;
		speed_integration_pitch = LIMIT(speed_integration_pitch,-40.0f,40.0f);
		
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_pitch = 0;
//		}
		
		i_out = - speed_integration_pitch;
		
		//d
		//error    +   （应该向左加速）<-- --> （应该向右加速）   -
		//error - error_old   正：更需要向左加速
		//					  负：没那么需要向左加速了
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.param_F; //user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//限制输出幅度为+-70，允许d引起刹车动作
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//输出整合
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float变量安全隔离
	out_tmp = (s32)(out*100.0f);	//放大100倍，保留小数点后2位精度
	out_tmp = LIMIT(out_tmp,-15000,15000);	//限幅
	out = ((float)out_tmp) / 100.0f;	//缩小100倍，回归float

	CH_ctrl[1] = out;	//根据经验值，CH_ctrl的输入值应该在50-100之间

}

void speed_flow_pitch_clear(void)
{
	speed_integration_pitch = 0.0;
}

//速度控制
//接口：except_speed      + <-- --> -      单位cm/s
static float speed_integration_roll = 0.0f;	//积分变量
void speed_flow_roll()
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old变量
	static u8 d_stop_flag = 0;		//停止d运算的标志位，表示speed_error_old数值无效
	s32 out_tmp;
	
	/*
		CH_filter[0]			遥控器横滚输入	- <---  ---> +
	
		speed_d_bias			速度值			+ <---  ---> -
		speed_d_bias_lpf		lpf值			+ <---  ---> -
	
		CH_ctrl[0]	横滚输出						- <---  ---> +		左负右正（负数向左有加速度，正数向右有加速度）
	*/
	
	//模式使能
	
	//except_speed      + <-- --> -      单位cm/s
	
	except_speed = position_roll_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	except_speed = LIMIT(except_speed,-30,30);			//限幅（速度调整要求平稳）
	
	if( OF_QUA < 20 )
	{
		// bias_detect（水平偏差）值异常处理
		
		// 根据期望速度进行处理，不适用速度差（因为速度反馈无效）
		
		//使用乒乓控制，系数对应 param_A param_B
		
		if( except_speed > 1.0f )	//速度期望向左
		{
			p_out = -30.0f * user_parameter.groups.param_A;	//左飞
		}
		else if(except_speed < -1.0f)	//速度期望向右
		{
			p_out =  30.0f * user_parameter.groups.param_B;	//右飞
		}
		else						
		{
			p_out = 0.0f;	//中间设置死区
		}

		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		d_stop_flag = 1;	//表示speed_error_old无效，无法进行d运算
	}
	else
	{
		//bias_detect值正常
		
		speed_error = except_speed - OF_DX2FIX_DETECT;	//计算error   speed_error值
															//error   正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速		负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
		
		//p
		p_out = - speed_error * user_parameter.groups.param_G;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.param_H;
		speed_integration_roll = LIMIT(speed_integration_roll,-40.0f,40.0f);
		
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_roll = 0;
//		}
		
		i_out = - speed_integration_roll;
		
		//d
		//error    +   （应该向左加速）<-- --> （应该向右加速）   -
		//error - error_old   正：更需要向左加速
		//					  负：没那么需要向左加速了
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.param_I;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//限制输出幅度为+-70，允许d引起刹车动作
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//输出整合
	out = p_out + i_out + d_out;			// - <-- --> +
	out = LIMIT(out,-150.0f,150.0f);
	
	//float变量安全隔离
	out_tmp = (s32)(out*100.0f);	//放大100倍，保留小数点后2位精度
	out_tmp = LIMIT(out_tmp,-15000,15000);	//限幅
	out = ((float)out_tmp) / 100.0f;	//缩小100倍，回归float

	CH_ctrl[0] = out;	//根据经验值，CH_ctrl的输入值应该在50-100之间

}

void speed_flow_roll_clear(void)
{
	speed_integration_roll = 0.0;
}


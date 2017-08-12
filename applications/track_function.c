#include "track_function.h"

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

//前进时用的pitch控制
void forward_pitch(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old变量
	static u8 d_stop_flag = 0;		//停止d运算的标志位，表示speed_error_old数值无效
	s32 out_tmp;
	
	static u8 lost_circle_flag = 0;	//如果出现超出状态，就是已经丢失圆，准备进入跟随模式
	
	/*
		speed_d_bias_pitch			速度值			+ <前---  ---后> -
		speed_d_bias_lpf_pitch		lpf值			+ <前---  ---后> -
	
		CH_filter[1]				遥控器俯仰输入	- <前---  ---后> +
	*/

	//模式使能
	
	//except_speed_pitch      + <-- --> -      单位cm/s
	
	except_speed = 10;	//给一个比较合适的前进初速度
	except_speed = LIMIT(except_speed,-15,15);			//限幅（速度调整要求平稳）
	
	if( bias_error_flag != 0 )
	{
		// bias_detect（水平偏差）值异常处理
		
		//向前飘动
		
		p_out = 0.0f;	//向前很低的倾角
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		d_stop_flag = 1;	//表示speed_error_old无效，无法进行d运算
		
		lost_circle_flag = 1;	//在向前飘模式出现前后超出都认为是已经开始匹配小车
	}
	else
	{
		if(lost_circle_flag)
		{
			//在丢失圆后又看到东西
			ctrl_command = 5;		//进入跟随模式
		}
		
		//bias_detect值正常
		
		speed_error = except_speed - speed_d_bias_lpf_pitch;	//计算error   speed_error值
																//error   负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
																//		  正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速
		
		//p
		p_out = - speed_error * pid_setup.groups.ctrl4.kp; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * pid_setup.groups.ctrl4.ki; //user_parameter.groups.self_def_1.ki;
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
			d_out = -(speed_error - speed_error_old) * pid_setup.groups.ctrl4.kd; //user_parameter.groups.self_def_1.kd;
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

//前进时用的roll控制
void forward_roll(void)
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
	
	except_speed = LIMIT(except_speed,-15,15);			//限幅（速度调整要求平稳）
	
	if( bias_error_flag != 0 )
	{
		// bias_detect（水平偏差）值异常处理
		
		// 使用乒乓控制，系数对应 param_A param_B

		// 此时已经丢失视野，放弃roll调整
		
		p_out = 0.0f;
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		d_stop_flag = 1;	//表示speed_error_old无效，无法进行d运算
	}
	else
	{
		//bias_detect值正常
		
		speed_error = except_speed - speed_d_bias_lpf;	//计算期望速度差   speed_error值   - <-- --> +
														//error   正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速		负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
		
		//p
		p_out = - speed_error * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.self_def_1.ki;
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
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
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

	CH_ctrl[0] = out;	//根据经验值，CH_ctrl的输入值应该在50-100之间
}


//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

//后退时用的pitch控制
void backward_pitch(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old变量
	static u8 d_stop_flag = 0;		//停止d运算的标志位，表示speed_error_old数值无效
	s32 out_tmp;
	
	/*
		speed_d_bias_pitch			速度值			+ <前---  ---后> -
		speed_d_bias_lpf_pitch		lpf值			+ <前---  ---后> -
	
		CH_filter[1]				遥控器俯仰输入	- <前---  ---后> +
	*/

	//模式使能
	
	//except_speed_pitch      + <-- --> -      单位cm/s
	
	except_speed = -5;	//给一个比较合适的前进初速度
	except_speed = LIMIT(except_speed,-15,15);			//限幅（速度调整要求平稳）
	
	if( bias_error_flag != 0 )
	{
		// bias_detect（水平偏差）值异常处理
		
		//向前飘动
		
		p_out = 0.0f;	//滑行后退
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		d_stop_flag = 1;	//表示speed_error_old无效，无法进行d运算

	}
	else
	{
		//bias_detect值正常
		
		speed_error = except_speed - speed_d_bias_lpf_pitch;	//计算error   speed_error值
																//error   负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
																//		  正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速
		
		//p
		p_out = - speed_error * pid_setup.groups.ctrl4.kp; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * pid_setup.groups.ctrl4.ki; //user_parameter.groups.self_def_1.ki;
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
			d_out = -(speed_error - speed_error_old) * pid_setup.groups.ctrl4.kd; //user_parameter.groups.self_def_1.kd;
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

//前进时用的roll控制
void backward_roll(void)
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
	
	except_speed = LIMIT(except_speed,-15,15);			//限幅（速度调整要求平稳）
	
	if( bias_error_flag != 0 )
	{
		// bias_detect（水平偏差）值异常处理
		
		// 使用乒乓控制，系数对应 param_A param_B

		// 此时已经丢失视野，放弃roll调整
		
		p_out = 0.0f;
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		d_stop_flag = 1;	//表示speed_error_old无效，无法进行d运算
	}
	else
	{
		//bias_detect值正常
		
		speed_error = except_speed - speed_d_bias_lpf;	//计算期望速度差   speed_error值   - <-- --> +
														//error   正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速		负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
		
		//p
		p_out = - speed_error * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.self_def_1.ki;
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
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
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

	CH_ctrl[0] = out;	//根据经验值，CH_ctrl的输入值应该在50-100之间
}


//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************


//Pitch位置控制
static float position_integration_pitch = 0;
void position_track_pitch(float T)	//与摄像头采集数据同频调用
{
	float p_out,i_out,d_out,out;
	static u16 bias_error_counter_pitch = 0;

	/*
		bias_pitch				原始值					+ <前---  ---后> -
		bias_detect_pitch 		原始值的统计滤波结果		+ <前---  ---后> -
		bias_error_flag_pitch	偏移值无效指示			0：偏移值正常	1：偏移值异常
		bias_real_pitch			校正值					+ <前---  ---后> -
		bias_lpf_pitch			校正值过低通滤波器		+ <前---  ---后> -
	
		CH_ctrl[1]				俯仰输出					- <---  ---> +		前负后正（负数向前有加速度，正数向后有加速度）
	*/
	
	if( bias_error_flag_pitch != 0 )
	{
		//偏移过大，使用乒乓控制
		
		if(	( bias_error_counter_pitch * T ) > 20.0f )	//时间超过5s
		{
			p_out = 0.0f;	//不再控制
			i_out = 0.0f;
			d_out = 0.0f;
		}
		else
		{
			bias_error_counter_pitch++;	//继续记录超出控制时间
			
			if( bias_detect_pitch < -30.0f )
			{
				//后偏过大
				p_out = -5;		//前飞
			}
			else if( bias_detect_pitch > 30.0f )
			{
				//前偏过大
				p_out =  5;		//后飞
			}
			else
			{
				p_out = 0.0f;
			}
			
//			p_out = bias_lpf_pitch / 2.0f * pid_setup.groups.ctrl5.kp;

			i_out = 0.0f;
			d_out = 0.0f;
		}
		
		position_integration_pitch = 0.0f;		//积分清零
		
	}
	else
	{
		//正常值
		
		bias_error_counter_pitch = 0;
		
		//不用计算error，因为期望为0
		
		//p
		p_out = bias_lpf_pitch * pid_setup.groups.ctrl5.kp; //user_parameter.groups.self_def_2.kp;
		
		//i
		position_integration_pitch += bias_lpf_pitch * pid_setup.groups.ctrl5.ki; //user_parameter.groups.self_def_2.ki;
		position_integration_pitch = LIMIT(position_integration_pitch,-10.0f,10.0f);
		
		//接近中心位置I清零
		if(ABS(bias_lpf_pitch) < 10)
		{
			position_integration_pitch = 0.0f;
		}
		
		i_out = position_integration_pitch;
		
		//d
		d_out = speed_d_bias_lpf_pitch * pid_setup.groups.ctrl5.kd; //user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf 左正右负
		d_out = LIMIT(d_out,-10.0f,10.0f);	//限制输出幅度为+-70，允许d引起刹车动作
	}
	
	//输出整合
	//PID输出 out： - <前-- --后> +
	out = p_out + i_out + d_out;
	//out = LIMIT(out,-15.0f,15.0f);
	
	position_pitch_out = -out;	//	+ <前-- --后> -
}

//Roll位置控制
static float position_integration_roll = 0;
void position_track_roll(float T)
{
	float p_out,i_out,d_out,out;
	
	static u16 bias_error_counter;

	/*
		bias		原始值					+ <---  ---> -
		bias_detect 原始值的统计滤波结果		+ <---  ---> -
		bias_real	校正值					+ <---  ---> -
		bias_lpf	校正值过低通滤波器		+ <---  ---> -
	
		CH_ctrl[0]	横滚输出					- <---  ---> +		左负右正（负数向左有加速度，正数向右有加速度）
	*/
	
	if( bias_error_flag != 0 )
	{
		//偏移过大，使用乒乓控制
		
		if( bias_error_counter * T >20.0f )	//超时控制持续时间大于5s
		{
			p_out = 0.0f;	//不再控制
			i_out = 0.0f;
			d_out = 0.0f;
		}
		else
		{
			bias_error_counter++;
			
			if( bias_detect < -50.0f )	//根据偏移方向给出指令
			{
				//右偏过大
				p_out = -5;		//左飞
			}
			else if( bias_detect > 50.0f )
			{
				//左偏过大
				p_out =  5;		//右飞
			}
			else
			{
				//容错
				p_out = 0.0f;
			}
			
//			p_out = bias_lpf / 2.0f * user_parameter.groups.self_def_2.kp;

			i_out = 0.0f;
			d_out = 0.0f;
		}
		
		position_integration_roll = 0.0f;	//积分清零
	}
	else
	{
		//正常值
		
		bias_error_counter = 0;
		
		//不用计算error，因为期望为0
		
		//p
		p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
		
		//i
		position_integration_roll += bias_lpf * user_parameter.groups.self_def_2.ki;
		position_integration_roll = LIMIT(position_integration_roll,-10.0f,10.0f);
		
		//接近中心位置I清零
		if(ABS(bias_lpf) < 10)
		{
			position_integration_roll = 0.0f;
		}
		
		i_out = position_integration_roll;
		
		//d
		d_out = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf 左正右负
		d_out = LIMIT(d_out,-10.0f,10.0f);	//限制输出幅度为+-70，允许d引起刹车动作
	}
	
	//输出整合
	//PID输出 out： - <-- --> +
	out = p_out + i_out + d_out;
	//out = LIMIT(out,-15.0f,15.0f);
	
	position_roll_out = -out;	//	+ <---  ---> -

}

//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************


//追踪用速度控制
//接口：except_speed      + <-- --> -      单位cm/s
void speed_track_pitch(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old变量
	static u8 d_stop_flag = 0;		//停止d运算的标志位，表示speed_error_old数值无效
	s32 out_tmp;
	
	/*
		speed_d_bias_pitch			速度值			+ <前---  ---后> -
		speed_d_bias_lpf_pitch		lpf值			+ <前---  ---后> -
	
		CH_filter[1]				遥控器俯仰输入	- <前---  ---后> +
	*/

	//模式使能
	
	//except_speed_pitch      + <-- --> -      单位cm/s
	
	except_speed = position_pitch_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	except_speed = LIMIT(except_speed,-15,15);			//限幅（速度调整要求平稳）
	
	if( bias_error_flag != 0 )
	{
		// bias_detect（水平偏差）值异常处理
		
		//偏移过大，使用乒乓控制，系数对应 param_A param_B
		
		if( except_speed > 1.0f )	//速度期望向前
		{
			p_out = -20.0f * pid_setup.groups.ctrl6.kp; //user_parameter.groups.param_A;	//前飞
		}
		else if(except_speed < -1.0f)	//速度期望向后
		{
			p_out =  20.0f * pid_setup.groups.ctrl6.ki; //user_parameter.groups.param_B;	//后飞
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
		
		speed_error = except_speed - speed_d_bias_lpf_pitch;	//计算error   speed_error值
																//error   负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
																//		  正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速
		
		//p
		p_out = - speed_error * pid_setup.groups.ctrl4.kp; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * pid_setup.groups.ctrl4.ki; //user_parameter.groups.self_def_1.ki;
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
			d_out = -(speed_error - speed_error_old) * pid_setup.groups.ctrl4.kd; //user_parameter.groups.self_def_1.kd;
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

//追踪速度控制
//接口：except_speed      + <-- --> -      单位cm/s
void speed_track_roll(void)
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
	
	except_speed = LIMIT(except_speed,-15,15);			//限幅（速度调整要求平稳）
	
	if( bias_error_flag != 0 )
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
		
		speed_error = except_speed - speed_d_bias_lpf;	//计算期望速度差   speed_error值   - <-- --> +
														//error   正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速		负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
		
		//p
		p_out = - speed_error * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.self_def_1.ki;
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
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
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

	CH_ctrl[0] = out;	//根据经验值，CH_ctrl的输入值应该在50-100之间
}



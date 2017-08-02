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
		
		//清零
		
		case 0x06:	//清零（不存储）
			mpu6050.vec_3d_cali.x = 0;
			mpu6050.vec_3d_cali.y = 0;
		break;
		
		default:
			
		break;
	}
}

void set_all_out_switch(u8 cmd)
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

//下降到15cm
void falling_to_15cm(void)	//这个高度相当于降落了（起落架大约比15cm短一点点）
{
	Thr_Low = 1;	//油门低标志为1，允许螺旋桨停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 150;		//下降到150mm = 15cm
}

//上升到50cm
void rising_to_50cm(void)
{
	Thr_Low = 0;	//油门低标志置0，防止螺旋桨意外停转
	
	//期望高度控制高度
	my_height_mode = 1;

	//设置期望高度
	my_except_height = 500;		//上升到50cm
}

//=====================================================================================================================================
//=====================================================================================================================================
//
//									                     姿态（位置）控制函数
//
//					可以使用的参数：	float bias、float bias_detect、float bias_real、float bias_lpf	左正右负（数值为正则偏左，数值为负则偏右）
//									float angle、
//									float speed
//
//					控制输出：		CH_ctrl[0]	横滚输出							左负右正（负数向左有加速度，正数向右有加速度）
//
//=====================================================================================================================================
//=====================================================================================================================================

/***********************************************

					位置控制函数

	输入：	en		模式启用标志			0：停用		1：启用
			mode	位置控制工作模式		0：定点		1：前进		2：后退
			
	输出：	position_roll_out			输出roll方向速度期望，单位cm/s			左正右负
			position_pitch_out			输出pitch方向速度期望，单位cm/s			前正后负
			position_mode_out			0：定点		1：前进		2：后退

************************************************/
float position_roll_out = 0.0f;
float position_pitch_out = 0.0f;
u8 position_mode_out = 0;
void position_ctrl(u8 en,u8 mode)
{
	static float position_integration_roll = 0;
	static float position_integration_pitch = 0;
	
	position_mode_out = mode;
	
	if(en == 0)
	{
		position_integration_roll = 0;
		position_integration_pitch = 0;
	}
		
	/*
		bias_error_flag		bias_detect数据值异常指示		0：正常		1：从异常中恢复回来后的第一帧    2：异常
		bias		原始值					+ <---  ---> -
		bias_detect 原始值的统计滤波结果		+ <---  ---> -
		bias_real	校正值					+ <---  ---> -
		bias_lpf	校正值过低通滤波器		+ <---  ---> -
	
		bias_error_flag_pitch;	bias_detect_pitch值异常指示		0：正常		1：从异常中恢复回来后的第一帧    2：异常
		bias_pitch				原始值						前 
		bias_detect_pitch;		原始值的统计滤波结果			/\   +
		bias_real_pitch;		校正值					    ||
		bias_lpf_pitch;			校正值过低通滤波器			\/	 -
															后
	
		CH_ctrl[0]	横滚输出					- <---  ---> +		左负右正（负数向左有加速度，正数向右有加速度）
		CH_ctrl[1]	俯仰输出					前 -   后 +
	*/

	//***************************************************
	//pitch方向
	
	float p_out_pitch,i_out_pitch,d_out_pitch,out_pitch;
	
	if(mode == 0)
	{
		//定点（PID）
		
		if( bias_error_flag_pitch != 0 )
		{
			//偏移过大，使用乒乓控制
			
			if( bias_detect_pitch < -30.0f )
			{
				//右偏过大
				p_out_pitch = -100;		//左飞
			}
			else if( bias_detect_pitch > 30.0f )
			{
				//左偏过大
				p_out_pitch =  100;		//右飞
			}

			i_out_pitch = 0.0f;
			d_out_pitch = 0.0f;
		}
		else
		{
			//正常值
			
			//不用计算error，因为期望为0
			
			//p
			p_out_pitch = bias_lpf_pitch * pid_setup.groups.ctrl5.kp;
			
			//i
			position_integration_pitch += bias_lpf * pid_setup.groups.ctrl5.ki;
			position_integration_pitch = LIMIT(position_integration_pitch,-10.0f,10.0f);
			i_out_pitch = position_integration_pitch;
			
			//d
			d_out_pitch = speed_d_bias_lpf * pid_setup.groups.ctrl5.kd;		//speed_d_bias_lpf 左正右负
			d_out_pitch = LIMIT(d_out_pitch,-10.0f,10.0f);	//限制输出幅度为+-70，允许d引起刹车动作
		}
		
		//输出整合
		//PID输出 out： - <-- --> +
		out_pitch = p_out_pitch + i_out_pitch + d_out_pitch;
		out_pitch = LIMIT(out_pitch,-15.0f,15.0f);
		
		//输出给 position_pitch_out
		//输出的值应该在-15到+15之间
		position_pitch_out = -out_pitch;
	}
	else
	{
		//前进或后退
		
		if(mode == 1)	//前进
		{
			position_pitch_out = 100;
		}
		else
		{
			position_pitch_out = -100;
		}
	}
	
	
	
	//***************************************************
	//roll方向
	
	float p_out_roll,i_out_roll,d_out_roll,out_roll;
	
	if( bias_error_flag != 0 )
	{
		//偏移过大，使用乒乓控制
		
		if( bias_detect < -50.0f )
		{
			//右偏过大
			p_out_roll = -5;		//左飞
		}
		else if( bias_detect > 50.0f )
		{
			//左偏过大
			p_out_roll =  5;		//右飞
		}

		i_out_roll = 0.0f;
		d_out_roll = 0.0f;
	}
	else
	{
		//正常值
		
		//不用计算error，因为期望为0
		
		//p
		p_out_roll = bias_lpf * user_parameter.groups.self_def_2.kp;
		
		//i
		position_integration_roll += bias_lpf * user_parameter.groups.self_def_2.ki;
		position_integration_roll = LIMIT(position_integration_roll,-10.0f,10.0f);
		i_out_roll = position_integration_roll;
		
		//d
		d_out_roll = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf 左正右负
		d_out_roll = LIMIT(d_out_roll,-10.0f,10.0f);	//限制输出幅度为+-70，允许d引起刹车动作
	}
	
	//输出整合
	//PID输出 out： - <-- --> +
	out_roll = p_out_roll + i_out_roll + d_out_roll;
	out_roll = LIMIT(out_roll,-15.0f,15.0f);

	
	//输出给 position_roll_out
	//输出的值应该在-15到+15之间
	position_roll_out = -out_roll;

}

/**************************************************************************************

				速度控制环

	输入：	pitch_mode				0：定点，根据except_speed_pitch控制		1：前进（恒定10cm/s）	2：后退（恒定-10cm/s）
			except_speed_pitch		pitch方向期望速度，单位cm/s，前正后负
			except_speed_roll		roll方向期望速度，单位cm/s，左正右负
			
	输出：	CH_ctrl[0]				roll方向角度期望（左负右正）
			CH_ctrl[1]				pitch方向角度期望（前负后正）
			CH_ctrl[3]				yaw方向角速度期望（左负右正）


**************************************************************************************/
void speed_ctrl(u8 en)
{
	static float speed_error_integration_pitch = 0.0f;
	static float speed_error_integration_roll = 0.0f;
	
	//非此模式时的清零处理
	if(!en)
	{
		speed_error_integration_pitch = 0;
		speed_error_integration_roll = 0;
		
		return;
	}
	
	//来自position_ctrl的数据接口
	u8 pitch_mode = position_mode_out;
	float except_speed_pitch = position_pitch_out;
	float except_speed_roll = position_roll_out;
	
	//*********************************************************
	
	//pitch方向
	
	float speed_error_pitch = 0.0f;
	static float speed_error_old_pitch = 0.0f;

	float p_out_pitch = 0.0f, i_out_pitch = 0.0f, d_out_pitch = 0.0f, out_pitch = 0.0f;
	
	
	/*
								   前               				前
		speed_error_pitch：		   /\  +         CH_ctrl[PIT]：   	/\  -
								   ||								||
								   ||								||
								   \/  -							\/  +
								   后								后
	*/
	
	if(pitch_mode == 0)
	{
		//悬停
		
		//期望输入（单位是cm/s）
		//except_speed_pitch = 0.0f;	//-( my_deathzoom( ( CH_filter[RIT] ) , 0, 30 ) / 5.0f );
		except_speed_pitch = my_deathzoom( except_speed_pitch , 0, 1 );		//设置+-1的死区
		except_speed_pitch = LIMIT(except_speed_pitch , -15, 15);			//限幅 -15 -- +15
		
	}
	else
	{
		//前进或后退（恒速）
		//数据可用时控制速度，不可用时飘过去
		
		if(pitch_mode == 1)	//前进
		{
			except_speed_pitch = 10;	//向前的期望速度为10cm/s
		}
		else	//pitch_mode = 2  后退
		{
			except_speed_pitch = -10;	//向后的期望速度为-10cm/s
		}
	}
	
	//计算error
	//在摄像头无数据时speed_d_bias_lpf_pitch为0
	speed_error_pitch = except_speed_pitch - speed_d_bias_lpf_pitch;	//计算error   speed_error值
																		//error   负：期望向前速度小于当前向后速度，期望向前速度比较小，应该向前加速
																		//		  正：期望向前速度大于当前向后速度，期望向前速度比较大，应该向后加速
	
	if(bias_error_flag_pitch != 0)
	{
		//速度反馈值不可信
		
		//使用乒乓控制，系数对应 pid_setup.groups.ctrl6.kp、pid_setup.groups.ctrl6.ki（根据期望速度方向设置加速度方向）
		
		//PID输出为正代表需要向前有加速度
		
		if(pitch_mode == 0)	//定点
		{
			
			if( speed_error_pitch > 0.0f)	//需要向前的加速度
			{
				p_out_pitch = 20.0f * pid_setup.groups.ctrl6.kp;	//前飞
				
			}
			else							//需要向后的加速度
			{
				p_out_pitch =  -20.0f * pid_setup.groups.ctrl6.ki;	//后飞
			}
			
		}
		else
		{
			//前进或后退
			
			if(pitch_mode == 1)	//前进
			{
				p_out_pitch = 2 * pid_setup.groups.ctrl6.kp;	//前飘
			}
			else	//后退
			{
				p_out_pitch = -2 * pid_setup.groups.ctrl6.ki;	//后飘
			}

		}
		
		i_out_pitch = 0.0f;
		d_out_pitch = 0.0f;

		speed_error_old_pitch = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
	}
	else
	{
		//p
		p_out_pitch = speed_error_pitch * pid_setup.groups.ctrl4.kp;
		
		//i
		speed_error_integration_pitch += speed_error_pitch * pid_setup.groups.ctrl4.ki;
		speed_error_integration_pitch = LIMIT(speed_error_integration_pitch,-40.0f,40.0f);
		i_out_pitch = speed_error_integration_pitch;
		
		//d
		//error    +   （应该向左加速）<-- --> （应该向右加速）   -
		//error - error_old   正：向左的期望速度差变大了，需要向左加速
		//					  负：向左的期望速度差变小了，可以放缓向左加速/向右加速
		d_out_pitch = (speed_error_pitch - speed_error_old_pitch) * pid_setup.groups.ctrl4.kd;
		d_out_pitch = LIMIT(d_out_pitch,-70.0f,70.0f);												//限制输出幅度为+-70，允许d引起刹车动作
		
		speed_error_old_pitch = speed_error_pitch;
	}
	
	//整合输出
	out_pitch = p_out_pitch + i_out_pitch + d_out_pitch;
	out_pitch = LIMIT(out_pitch,-150.0f,150.0f);			//单位是0.1°
	
	CH_ctrl[1] = -out_pitch;		//my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	
	//*********************************************************
	
	//roll方向
	
	float speed_error_roll = 0.0f;
	static float speed_error_old_roll = 0.0f;
	float p_out_roll = 0.0f, i_out_roll = 0.0f, d_out_roll = 0.0f, out_roll = 0.0f;
	
	
	//期望输入（单位是cm/s）
	//except_speed_roll = 0.0f;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	except_speed_roll = my_deathzoom( except_speed_roll , 0, 1 );	//设置+-1的死区
	except_speed_roll = LIMIT(except_speed_roll , -15, 15);			//限幅 -15 -- +15
	
	//计算error
	speed_error_roll = except_speed_roll - speed_d_bias_lpf;	//计算error   speed_error值
																//error   负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
																//		  正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速
	
	//PID输出左正右负 + <-- --> -
	
	if( bias_error_flag != 0 )
	{
		//速度反馈值不可信
		
		//使用乒乓控制，系数对应 param_A param_B（根据期望速度方向设置加速度方向）
		
		//PID输出为正代表需要向左有加速度
		
		if( speed_error_roll > 0.0f)	//需要向左的加速度
		{
			p_out_roll = 40.0f * user_parameter.groups.param_A;	//左飞
		}
		else							//需要向右的加速度
		{
			p_out_roll =  -40.0f * user_parameter.groups.param_B;	//右飞
		}

		i_out_roll = 0.0f;
		d_out_roll = 0.0f;
		
		speed_error_old_roll = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
		
	}
	else
	{
		//bias_detect值正常
		
		//PID输出数值为正代表需要向左有加速度
		
		//p
		p_out_roll = speed_error_roll * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_error_integration_roll += speed_error_roll * user_parameter.groups.self_def_1.ki;
		speed_error_integration_roll = LIMIT(speed_error_integration_roll,-40.0f,40.0f);
		i_out_roll = speed_error_integration_roll;
		
		//d
		//error    +   （应该向左加速）<-- --> （应该向右加速）   -
		//error - error_old   正：向左的期望速度差变大了，需要向左加速
		//					  负：向左的期望速度差变小了，可以放缓向左加速/向右加速
		d_out_roll = (speed_error_roll - speed_error_old_roll) * user_parameter.groups.self_def_1.kd;
		d_out_roll = LIMIT(d_out_roll,-70.0f,70.0f);	//限制输出幅度为+-70，允许d引起刹车动作
		
		speed_error_old_roll = speed_error_roll;
	}

	//输出整合
	out_roll = p_out_roll + i_out_roll + d_out_roll;
	out_roll = LIMIT(out_roll,-150.0f,150.0f);			//单位是0.1°

	CH_ctrl[0] = -out_roll;	//CH_ctrl   - <-- --> +
								//out_roll	+ <-- --> -
								//接口需要加负号
	
	//*********************************************************
	
	//俯仰和航向手动控制
	CH_ctrl[3] = CH_filter[3];								//3：航向 YAW
	
}





//手动控制姿态
void attitude_hand(void)
{
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[YAW] = CH_filter[YAW];	//3：航向 YAW
}


//********************************************************************************************************************
//													测试函数
//********************************************************************************************************************

//横滚角乒乓控制
void attitude_pingpong(void)
{
	/*
	
		参数表：
		user_parameter.groups.self_def_1	横滚方向PID		对应地面站PID13
	
	*/
	
	//横滚自动控制
	if(bias_real > 14)	//偏左
	{
		CH_ctrl[0] =  40 * user_parameter.groups.self_def_1.kp;	//横滚方向kp	向右调整（为正）
	}
	else if(bias_real < -14)
	{
		CH_ctrl[0] = -50 * user_parameter.groups.self_def_1.ki;	//向左调整（为负）
	}
	else
	{
		CH_ctrl[0] = 0;
	}
	
	//俯仰和航向手动控制
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	CH_ctrl[3] = CH_filter[3];								//3：航向 YAW
}

//近地面姿态锁定
void land_attitude(void)
{
	CH_ctrl[ROL] = 0;	//0：横滚 ROL
	CH_ctrl[PIT] = 0;	//1：俯仰 PIT
	CH_ctrl[YAW] = 0;	//3：航向 YAW
}

//位置控制（对接到速度控制）
float position_except_speed = 0.0f;		//输出速度期望，单位cm/s，方向 + <-- --> -
void position_to_speed_pid(u8 en)
{
	float p_out,i_out,d_out,out;
	static float position_roll_integration = 0;
	
	if(en)
	{
		
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
			
			if( bias_detect < -50.0f )
			{
				//右偏过大
				p_out = -5;		//左飞
			}
			else if( bias_detect > 50.0f )
			{
				//左偏过大
				p_out =  5;		//右飞
			}

			i_out = 0.0f;
			d_out = 0.0f;
		}
		else
		{
			//正常值
			
			//不用计算error，因为期望为0
			
			//p
			p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
			
			//i
			position_roll_integration += bias_lpf * user_parameter.groups.self_def_2.ki;
			position_roll_integration = LIMIT(position_roll_integration,-10.0f,10.0f);
			i_out = position_roll_integration;
			
			//d
			d_out = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf 左正右负
			d_out = LIMIT(d_out,-10.0f,10.0f);	//限制输出幅度为+-70，允许d引起刹车动作
		}
		
		//输出整合
		//PID输出 out： - <-- --> +
		out = p_out + i_out + d_out;
		out = LIMIT(out,-15.0f,15.0f);
		
		//输出给 position_except_speed
		//输出的值应该在-15到+15之间
		position_except_speed = -out;

	}
	else
	{
		position_roll_integration = 0;
	}
}

//速度控制
//接口：except_speed      + <-- --> -      单位cm/s
void speed_pid(u8 en)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	static float roll_speed_integration = 0.0f;	//积分变量
	static float speed_error_old = 0.0f;	//old变量
	s32 out_tmp;
	
	/*
		CH_filter[0]			遥控器横滚输入	- <---  ---> +
	
		speed_d_bias			速度值			+ <---  ---> -
		speed_d_bias_lpf		lpf值			+ <---  ---> -
	
		CH_ctrl[0]	横滚输出						- <---  ---> +		左负右正（负数向左有加速度，正数向右有加速度）
	*/
	
	if(en)
	{
		//模式使能
		
		//except_speed      + <-- --> -      单位cm/s
		
		except_speed = position_except_speed;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
		
		except_speed = my_deathzoom( except_speed , 0, 1 );	//设置+-1的死区
		except_speed = LIMIT(except_speed,-15,15);			//限幅
		
		if( bias_error_flag != 0 )
		{
			// bias_detect（水平偏差）值异常处理
			
			//偏移过大，使用乒乓控制，系数对应 param_A param_B
			
			if( bias_detect < -50.0f )
			{
				//右偏过大
				p_out = -40.0f * user_parameter.groups.param_A;	//左飞
			}
			else if( bias_detect > 50.0f )
			{
				//左偏过大
				p_out =  40.0f * user_parameter.groups.param_B;	//右飞
			}

			i_out = 0.0f;
			d_out = 0.0f;
			
			speed_error_old = 0;	// speed_error_old 清零（在一定程度上减小对d的影响）
			
		}
		else
		{
			//bias_detect值正常
			
			speed_error = except_speed - speed_d_bias_lpf;	//计算error   speed_error值
															//error   负：期望向左速度小于当前向左速度，期望向左速度比较小，应该向右加速
															//		  正：期望向左速度大于当前向左速度，期望向左速度比较大，应该向左加速
			
			//p
			p_out = - speed_error * user_parameter.groups.self_def_1.kp;
			
			//i
			roll_speed_integration += speed_error * user_parameter.groups.self_def_1.ki;
			roll_speed_integration = LIMIT(roll_speed_integration,-40.0f,40.0f);
			i_out = - roll_speed_integration;
			
			//d
			//error    +   （应该向左加速）<-- --> （应该向右加速）   -
			//error - error_old   正：更需要向左加速
			//					  负：没那么需要向左加速了
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//限制输出幅度为+-70，允许d引起刹车动作
			
			speed_error_old = speed_error;
		}
		
		//输出整合
		out = p_out + i_out + d_out;
		out = LIMIT(out,-150.0f,150.0f);
		
		//float变量安全隔离
		out_tmp = (s32)(out*100.0f);	//放大100倍，保留小数点后2位精度
		out_tmp = LIMIT(out_tmp,-15000,15000);	//限幅
		out = ((float)out_tmp) / 100.0f;	//缩小100倍，回归float

		CH_ctrl[0] = out;

		//俯仰和航向手动控制
		CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
		CH_ctrl[3] = CH_filter[3];								//3：航向 YAW
	}
	else
	{
		//非此模式执行清零
		roll_speed_integration = 0.0;
	}
}

//位置控制
void position_pid(u8 en)
{
	float p_out,i_out,d_out,out;
	static float roll_integration = 0;
	s32 out_tmp;
	
	if(en)
	{
		
		/*
			bias		原始值					+ <---  ---> -
			bias_detect 原始值的统计滤波结果		+ <---  ---> -
			bias_real	校正值					+ <---  ---> -
			bias_lpf	校正值过低通滤波器		+ <---  ---> -
		
			CH_ctrl[0]	横滚输出					- <---  ---> +		左负右正（负数向左有加速度，正数向右有加速度）
		*/
		
		if( bias_error_flag != 0 )
		{
			//偏移过大，使用乒乓控制，系数对应 param_A param_B
			
			if( bias_detect < -50.0f )
			{
				//右偏过大
				p_out = -40.0f * user_parameter.groups.param_A;	//左飞
			}
			else if( bias_detect > 50.0f )
			{
				//左偏过大
				p_out =  40.0f * user_parameter.groups.param_B;	//右飞
			}

			i_out = 0.0f;
			d_out = 0.0f;
			
		}
		else
		{
			//正常值
			
			//不用计算error，因为期望为0
			
			//p
			p_out = bias_lpf * user_parameter.groups.param_D;
			
			//i
			roll_integration += bias_lpf * user_parameter.groups.param_E;
			roll_integration = LIMIT(roll_integration,-40.0f,40.0f);
			i_out = roll_integration;
			
			//d
			d_out = speed_d_bias_lpf * user_parameter.groups.param_F;		//speed_d_bias_lpf 左正右负
			d_out = LIMIT(d_out,-70.0f,70.0f);	//限制输出幅度为+-70，允许d引起刹车动作
		}
		
		//输出整合
		out = p_out + i_out + d_out;
		out = LIMIT(out,-150.0f,150.0f);
		
		//float变量安全隔离
		out_tmp = (s32)(out*100.0f);	//放大100倍，保留小数点后2位精度
		out_tmp = LIMIT(out_tmp,-15000,15000);	//限幅
		out = ((float)out_tmp) / 100.0f;	//缩小100倍，回归float
		
		//横滚输出
		CH_ctrl[0] = out;

		//俯仰和航向手动控制
		CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
		CH_ctrl[3] = CH_filter[3];								//3：航向 YAW
	}
	else
	{
		roll_integration = 0;
	}
}

void yaw_pid(void)
{
	float yaw_error,yaw_out;
	
	/*
		angle：			偏左 - ，偏右 +
		CH_ctrl[YAW]：	左转 - ，右转 +
	*/
	
	//计算偏差
	yaw_error = 0.0f - angle;	
	
	//纠正输出
	yaw_out = yaw_error * user_parameter.groups.param_C;
	
	CH_ctrl[YAW] = yaw_out;		//3：航向 YAW
	
	//=======================================
	
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0：横滚 ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1：俯仰 PIT
	
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
	else
	{
		
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
		land();	//降落模式
	}
	else
	{
		
	}
	
	//意外状况处理
	if(height_command > 2)	//不应该出现的情况
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2：油门 THR
	}
	
/* ********************* 姿态控制 ********************* */
	
	if(ctrl_command == 0)
	{
		attitude_hand();
	}
	
	if(ctrl_command == 1)
	{
		attitude_hand();
	}
	
	if(ctrl_command == 2)
	{
		attitude_hand();
	}
	
//	if(ctrl_command == 3)
//	{
//		//attitude_single_p(1);
//	}
//	else
//	{
//		//attitude_single_p(0);
//	}
	
//	if(ctrl_command == 4)
//	{
//		//yaw_pid();
//	}

//	if(ctrl_command == 5)
//	{
//		attitude_hand();
//	}
	
	//意外状况处理
	if(ctrl_command > 5)
	{
		attitude_hand();
	}
	
}

//Cam频率调用的飞行控制函数
void Fly_Ctrl_Cam(void)		//调用周期与camera数据相同
{	
	//只有自动模式才会执行自动控制代码
	if(mode_state != 3)
	{
		return;
	}
	
/* ********************* 姿态控制 ********************* */
	
	if(ctrl_command == 3)
	{
		position_pid(1);
	}
	else
	{
		position_pid(0);
	}
	
	if(ctrl_command == 4)
	{
		position_ctrl(1,0);
		speed_ctrl(1);
	}
	else
	{
		position_ctrl(0,0);
		speed_ctrl(0);
	}
	
	if(ctrl_command == 5)
	{
		if(sonar.displacement >= 250)
		{
			//离地后使用pid
			position_to_speed_pid(1);
			speed_pid(1);
		}
		else
		{
			//近地面姿态锁定
			land_attitude();
			
			//停用位置控制pid
			position_to_speed_pid(0);
			speed_pid(0);
		}
	}
	else
	{
		position_to_speed_pid(0);
		speed_pid(0);
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


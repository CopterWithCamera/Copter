#include "camera_data_calculate.h"
#include "camera_datatransfer.h"
#include "mymath.h"
#include "math.h"
#include "stdio.h"

//roll方向
float bias_detect = 0.0;
u8 bias_error_flag = 0;		//bias_detect值异常指示		0：正常    1：从异常中恢复回来后的第一帧    2：异常
float bias_real = 0;
float bias_lpf = 0;
float speed_d_bias = 0;
float speed_d_bias_lpf = 0;

//pitch方向
float bias_detect_pitch = 0.0;
u8 bias_error_flag_pitch = 0;		//bias_detect值异常指示		0：正常    1：从异常中恢复回来后的第一帧    2：异常
float bias_real_pitch = 0;
float bias_lpf_pitch = 0;
float speed_d_bias_pitch = 0;
float speed_d_bias_lpf_pitch = 0;

//其余参数
float receive_fps = 0;

//低通滤波器
//dt：采样时间间隔（单位us）
//fc：截止频率
float cam_bias_lpf(float bias ,float dt_us ,float fc,float last_bias_lpf)
{
	float q,out,T;
	
	T = dt_us / 1000000.0f;
	q = 6.28f * T * fc;
	
	if(q >0.95f)
		q = 0.95f;
	
	out = q * bias + (1.0f-q) * last_bias_lpf;
	
	return out;
}

/***************************

	roll		左负右正
	pitch		前负后正
	
	bias		左正右负
	bias_pitch	前正后负

****************************/

/*
//bias数据校准函数
float bias_correct(float roll, float hight,float bias)   ///hight --超声波测量值   roll--横滚偏角  bias--图像像素点偏移
{
    float x1,real_bias;
	x1=hight*sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}
*/

//roll方向补偿
float bias_correct(float roll, float pitch, float hight, float bias)   ///hight --超声波测量值   roll--横滚偏角  bias--图像像素点偏移
{
    float x1, real_bias, coe_bias;  //coe_bias 为测得的bias 到实际bias的系数
	
	//矫正参数
	x1 = hight * sin(roll*3.141f/180.0f);
	
	//系数
	coe_bias = hight / 2.0f / 40.0f;
	
	real_bias = coe_bias * bias - x1;
	
	return real_bias;
}

//pitch方向补偿
float bias_pitch_correct(float roll, float pitch, float hight, float bias)   ///hight --超声波测量值   roll--横滚偏角  bias--图像像素点偏移
{
    float x1, real_bias, coe_bias;  //coe_bias 为测得的bias 到实际bias的系数
	
	//矫正参数
	x1 = hight * sin(pitch*3.141f/180.0f);
	
	//系数
	coe_bias = hight / 2.0f / 40.0f;
	
	real_bias = coe_bias * bias - x1;
	
	return real_bias;
}

/*
	T：时间间隔（单位us）
	bias：本次水平偏移量（单位cm）
	bias_old：上次水平偏移量（单位cm）
*/
float get_speed(u32 T,float bias,float bias_last,float speed_last)
{
	/*
		bias数值： 	+ <--- ---> -
		speed方向：	+ <--- ---> -	向左飞速度为正，向右飞速度为负
									
									向前飞速度为正，向后飞速度为负
	*/
	
	float dx,dt,speed;
	dx = bias - bias_last;
	dt = T / 1000000.0f;
	speed = safe_div(dx,dt,0);
	
	//数值过大则舍弃
	if( ABS(speed) > 25)
		speed = speed_last;
	
	return speed;
}

//定时读取帧率，要求调用频率为2s一次，在主循环中调用
u16 receive_fps_counter = 0;	//每次读取数据时+1
void get_fps(void)
{
	//计算接收帧率
	receive_fps = receive_fps_counter / 2.0f;	//转化为以Hz为单位
	receive_fps_counter = 0;
}

//Camera数据处理（根据标志位在主循环中调用）
void Camera_Calculate(void)
{
	
	//*****************************************8
	//roll方向处理

	//不使用三帧决断，直接信任采集值
	bias_detect = bias;
	
	//数据校准与滤波（生成 bias_error_flag、bias_real、bias_lpf、speed_d_bias、speed_d_bias_lpf 数值）
	static float bias_lpf_old;	//上一个在可用范围内的bias_lpf
	if(ABS(bias_detect)<38.0f)	//只有在合理范围内才会矫正，矫正的同时进行低通滤波
	{
		//正常情况
		
		//偏移运算
		bias_real = bias_correct(Roll_Image,Pitch_Image, Height_Image/10.0f, bias_detect);	//姿态误差校准
		bias_lpf = cam_bias_lpf(bias_real,receive_T,0.8f,bias_lpf);		//低通滤波器
		
		//速度运算（使用 bias_lpf 为数据源）
		if(bias_error_flag == 2)	//这一帧要等着bias_lpf_old更新
		{
			speed_d_bias = 0;		//无法运算speed_d_bias，所以归零
			speed_d_bias_lpf = 0;	//无法运算speed_d_bias_lpf，由于speed_d_bias_lpf要进入d运算，所以归零消除影响
			
			bias_error_flag--;	//这个函数处理完了 bias_error_flag 就是1了，所以这个状态相当于是从异常恢复后的第一帧，有偏移没速度
		}
		else if(bias_error_flag == 1)
		{
			speed_d_bias = get_speed(receive_T,bias_lpf,bias_lpf_old,speed_d_bias_lpf);
			speed_d_bias_lpf = speed_d_bias;	//由于之前的数值是0，所以直接采纳本帧结果
			
			bias_error_flag--;	//这个函数处理完了 bias_error_flag 就是0了，速度和速度lpf都恢复了
		}
		else
		{
			speed_d_bias = get_speed(receive_T,bias_lpf,bias_lpf_old,speed_d_bias_lpf);
			speed_d_bias_lpf = cam_bias_lpf(speed_d_bias,receive_T,1.0f,speed_d_bias_lpf);
		}
		
		bias_lpf_old = bias_lpf;	//更新上一帧的bias_lpf（只有bias_detect有效时才会更新 bias_lpf_old ，防止+-100的异常值加入运算）
	}
	else
	{
		bias_error_flag = 2;	//表示bias_detect数据处于异常值（微分运算得到速度需要两帧）
		
		speed_d_bias = 0;
		speed_d_bias_lpf = 0;
	}
	
	//*****************************************
	//pitch方向处理	
	
	//直接信任传入数值
	bias_detect_pitch = bias_pitch;
	
	//数据校准与滤波
	static float bias_lpf_old_pitch;	//上一个在可用范围内的bias_lpf
	if(ABS(bias_detect_pitch)<22.0f)	//数据正常（bias_detect_pitch的正常值只有-24~+24）
	{
		//偏移运算
		bias_real_pitch = bias_pitch_correct(Roll_Image,Pitch_Image, Height_Image/10.0f, bias_detect_pitch);	//姿态误差校准
		bias_lpf_pitch = cam_bias_lpf(bias_real_pitch,receive_T,0.8f,bias_lpf_pitch);		//低通滤波器
		
		//速度运算（使用 bias_lpf_pitch 为数据源）
		if(bias_error_flag_pitch == 2)	//这一帧要等着bias_lpf_old更新
		{
			speed_d_bias_pitch = 0;		//无法运算speed_d_bias_pitch，所以归零
			speed_d_bias_lpf_pitch = 0;	//无法运算speed_d_bias_lpf_pitch，由于speed_d_bias_lpf_pitch要进入d运算，所以归零消除影响
			
			bias_error_flag_pitch--;	//这个函数处理完了 bias_error_flag 就是1了，所以这个状态相当于是从异常恢复后的第一帧，有偏移没速度
		}
		else if(bias_error_flag_pitch == 1)
		{
			speed_d_bias_pitch = get_speed(receive_T,bias_lpf_pitch,bias_lpf_old_pitch,speed_d_bias_lpf_pitch);
			speed_d_bias_lpf_pitch = speed_d_bias_pitch;	//由于之前的数值是0，所以直接采纳本帧结果
			
			bias_error_flag_pitch--;	//这个函数处理完了 bias_error_flag 就是0了，速度和速度lpf都恢复了
		}
		else
		{
			speed_d_bias_pitch = get_speed(receive_T,bias_lpf_pitch,bias_lpf_old_pitch,speed_d_bias_lpf_pitch);
			speed_d_bias_lpf_pitch = cam_bias_lpf(speed_d_bias_pitch,receive_T,1.0f,speed_d_bias_lpf_pitch);
		}
		
		bias_lpf_old_pitch = bias_lpf_pitch;	//更新上一帧的bias_lpf_pitch（只有bias_detect_pitch有效时才会更新 bias_lpf_old_pitch ，防止+-100的异常值加入运算）
		
	}
	else	//数据异常
	{
		bias_error_flag_pitch = 2;	//偏移异常
		
		speed_d_bias_pitch = 0;
		speed_d_bias_lpf_pitch = 0;
	}
	
}

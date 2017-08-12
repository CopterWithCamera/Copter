/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：scheduler.c
 * 描述    ：任务调度
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "scheduler.h"
#include "include.h"
#include "time.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "led.h"
#include "rc.h"
#include "imu.h"
#include "pwm_in.h"
#include "ctrl.h"
#include "ms5611.h"
#include "parameter.h"
#include "ultrasonic.h"
#include "height_ctrl.h"
#include "fly_mode.h"
#include "fly_ctrl.h"
#include "anotc_baro_ctrl.h"
#include "usart.h"
#include "camera_datatransfer.h"
#include "camera_data_calculate.h"
#include "adc.h"
#include "ano_of.h"
#include "position_function.h"
#include "track_mode.h"
#include "renesas_datatransfer.h"

s16 loop_cnt;

loop_t loop;

void Loop_check()  //TIME INTTERRUPT
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
//	loop.cnt_camera_data_ms++;
	#if defined(ANO_OF_FUNCTION)
		loop.cnt_flow_data_ms++;	//只有允许光流线程运行时才能开启定时器累加
	#endif

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //每累加一次，证明代码在预定周期内没有跑完。
	}
	else
	{	
		loop.check_flag = 1;	//该标志位在循环的最后被清零
	}
	
	LED_1ms_DRV();				//20级led渐变显示
}

//********************************************************************************************************
//										线程
//********************************************************************************************************

float test[5];

//1ms线程
void Duty_1ms()
{
//	float ms_loop_time;
//	ms_loop_time = Get_Cycle_T(1)/1000000.0f;
	ANO_DT_Data_Exchange();		//数传通信定时调用
}

//2ms线程
void Duty_2ms()
{
	//获取本线程两次调用的时间差
	float inner_loop_time;
	inner_loop_time = Get_Cycle_T(0)/1000000.0f; 		//获取内环准确的执行周期（本次和上次调用的时间差，单位是s）
	
	/* ********************* 姿态计算 ********************* */
	
	//mpu6050处理
	MPU6050_Read(); 									//读取mpu6轴传感器
	MPU6050_Data_Prepare( inner_loop_time );			//mpu6轴传感器数据处理   校准、滤波、坐标转换
														
	//姿态解算（使用MPU6050数据进行姿态解算）
	/*IMU更新姿态。输入：半个执行周期，三轴陀螺仪数据（转换到度每秒），三轴加速度计数据（4096--1G）；输出：ROLPITYAW姿态角*/
 	IMUupdate(0.5f *inner_loop_time,											//半周期时间
				mpu6050.Gyro_deg.x, mpu6050.Gyro_deg.y, mpu6050.Gyro_deg.z, 	//三轴陀螺仪数据（转换到度每秒）
				mpu6050.Acc.x, mpu6050.Acc.y, mpu6050.Acc.z,					//三轴加速度计数据（4096--1G）
				&Roll,&Pitch,&Yaw);												//输出：ROL PIT YAW 姿态角
	
	//高度数据采集
	baro_ctrl( inner_loop_time ,&hc_value);			//高度数据获取，为内环函数最后调用的高度控制函数做准备（获取气压计数据，调用超声波数据，融合计算高度数据）
	
	/* ****************** 姿态 和 高度 控制 ******************** */
	
	//姿态控制
	CTRL_1( inner_loop_time ); 						//内环角速度控制。输入：执行周期，期望角速度，测量角速度，角度前馈；输出：电机PWM占空比。<函数未封装>
	
	//高度控制
	Thr_Ctrl( inner_loop_time , mode_state);		//油门控制，这里面包含高度控制闭环	thr_value 
	
	//输出控制
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);	//电机输出处理（包含电机输出判断，未解锁状态输出为0）
														//输出值包括两部分，posture_value 和 thr_value
														//out_roll,out_pitch,out_yaw 生成 posture_value
														//在 All_Out 里这两部分按照权重参数 Thr_Weight 整合
	
	/* ****************** RC接收机采集 ******************** */
	
	RC_Duty( inner_loop_time , Rc_Pwm_In );				//遥控器通道数据处理 ，输入：执行周期，接收机pwm捕获的数据。
}

//5ms线程
void Duty_5ms()
{
	float outer_loop_time;
	outer_loop_time = Get_Cycle_T(2)/1000000.0f;		//获取外环准确的执行周期，Get_Cycle_T(2)返回值的单位是us，除以1000000后单位是s
	
	/* ****************** 自动控制功能实现函数 ****************** */
	
	//控制中层
	Fly_Height_Ctrl(outer_loop_time);				//高度控制函数
	Fly_Ctrl(outer_loop_time);						//位置控制函数
	
	CTRL_2( outer_loop_time ); 					//外环角度控制。输入：执行周期，期望角度（摇杆量），姿态角度；输出：期望角速度。<函数未封装>
	
	//控制上层
	Fly_Mode_Ctrl(outer_loop_time);					//飞行模式控制
	Height_Mode_Ctrl(outer_loop_time);
	
	/* ********************** 姿态外环 ********************* */
 	
	
	//数值监控

//	mydata.d1 = (s16)ultra.height * 10;	//height
//	mydata.d2 = (s16)sonar.displacement;
//	mydata.d3 = (s16)sonar_fusion.fusion_displacement.out;
//	mydata.d4 = (s16)my_except_height;
	
	mydata.d1 = (s16)ultra.height * 10;	//height
	mydata.d2 = (s16)sonar.displacement;
	mydata.d3 = (s16)sonar_fusion.fusion_displacement.out;
	mydata.d4 = (s16)my_except_height;
	mydata.d5 = (s16)bias;				//roll方向
	mydata.d6 = (s16)bias_detect;
	mydata.d7 = (s16)bias_real;
	mydata.d8 = (s16)bias_lpf;
	mydata.d9 = (s16)speed_d_bias;
	mydata.d10 = (s16)speed_d_bias_lpf;
	mydata.d11 = (s16)angle;			//yaw
	mydata.d12 = (s16)bias_pitch;		//pitch方向
	mydata.d13 = (s16)bias_detect_pitch;
	mydata.d14 = (s16)bias_real_pitch;
	mydata.d15 = (s16)bias_lpf_pitch;
	mydata.d16 = (s16)speed_d_bias_pitch;
	mydata.d17 = (s16)speed_d_bias_lpf_pitch;
	mydata.d18 = (s16)CH_ctrl[ROL];
	mydata.d19 = (s16)CH_ctrl[PIT];
//	mydata.d20 = (s16)0;
	
//	mydata.d1 = (s16)ultra.height * 10;	//height
//	mydata.d2 = (s16)sonar.displacement;
//	mydata.d3 = (s16)sonar_fusion.fusion_displacement.out;
//	mydata.d4 = (s16)angle;
//	mydata.d5 = (s16)OF_QUA;	//质量参数
//	mydata.d6 = (s16)OF_LIGHT;
//	mydata.d7 = (s16)OF_DX;		//x方向
//	mydata.d8 = (s16)OF_DX2;
//	mydata.d9 = (s16)OF_DX2FIX;
//	mydata.d10 = (s16)OF_DY;	//y方向
//	mydata.d11 = (s16)OF_DY2;
//	mydata.d12 = (s16)OF_DY2FIX;
//	mydata.d13 = (s16)OF_ALT;	//高度数据
//	mydata.d14 = (s16)OF_ALT2;
//	mydata.d15 = (s16)bias;				//roll方向
//	mydata.d16 = (s16)bias_real;
//	mydata.d17 = (s16)bias_lpf;			
//	mydata.d18 = (s16)bias_pitch;		//pitch方向
//	mydata.d19 = (s16)bias_real_pitch;
//	mydata.d20 = (s16)bias_lpf_pitch;

//	mydata.d10 = (s16)position_roll_out;
//	mydata.d9 = (s16)position_pitch_out;
//	mydata.d11 = (s16)speed_d_bias_lpf;
//	mydata.d12 = (s16)speed_d_bias_lpf_pitch;
//	
//	mydata.d15 = (s16)bias;				//roll方向
//	mydata.d16 = (s16)bias_real;
//	mydata.d17 = (s16)bias_lpf;			
//	mydata.d18 = (s16)bias_pitch;		//pitch方向
//	mydata.d19 = (s16)bias_real_pitch;
//	mydata.d20 = (s16)bias_lpf_pitch;
}

//10ms线程
void Duty_10ms()
{
	ANO_AK8975_Read();			//获取电子罗盘数据	
}

//20ms线程
void Duty_20ms()
{
	Parameter_Save();
}

//50ms线程
void Duty_50ms()
{
	
	Ultra_Duty();			//定时向超声波传感器写入测距指令
	
	mode_check(CH_filter);	//根据辅助通道状态切换当前模式
	Ctrl_Mode(CH_filter);	//fly_ctrl用的飞行模式控制，在mode_state=3时起作用
	
	LED_Duty();				//根据标志位和飞机模式情况控制LED闪烁
	Copter_Data_Send();		//向图像处理板发送信息
	
	ADC_Read();			//获取电池电压
	
	static u8 get_fps_funtion_counter = 0;
	get_fps_funtion_counter++;
	if(get_fps_funtion_counter >= 40)
	{
		get_fps_funtion_counter = 0;
		get_fps();
	}
	
	renesas_data_send();	//向瑞萨发送数据
	
}

void Duty_Camera()
{
	float camera_loop_time;
	camera_loop_time = Get_Cycle_T(4)/1000000.0f;	//以s为单位
	
	Camera_Calculate();	//处理Camera数据
	Fly_Ctrl_Cam(camera_loop_time);
}

void Duty_Flow()
{
	float flow_loop_time;
	flow_loop_time = Get_Cycle_T(5)/1000000.0f;	//以s为单位
	
	flow_data_detect(flow_loop_time);
	Fly_Ctrl_Flow();
}

//********************************************************************************************************





//任务调度
void Duty_Loop()   					//最短任务周期为1ms，总的代码执行时间需要小于1ms。
{

	if( loop.check_flag == 1 )
	{
		loop_cnt = time_1ms;
		
		Duty_1ms();							//周期1ms的任务
		
		if( loop.camera_data_ok || loop.cnt_camera_data_ms >= 500 )
		{
			loop.camera_data_ok = 0;
			loop.cnt_camera_data_ms = 0;
			Duty_Camera();
		}
		
		if( loop.flow_data_ok || loop.cnt_flow_data_ms >= 500 )
		{
			loop.flow_data_ok = 0;
			loop.cnt_flow_data_ms = 0;
			Duty_Flow();
		}
		
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//周期2ms的任务
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//周期5ms的任务
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//周期10ms的任务
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//周期20ms的任务
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//周期50ms的任务
		}
		
		loop.check_flag = 0;		//循环运行完毕标志
	}
}




	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	


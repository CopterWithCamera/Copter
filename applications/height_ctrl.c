/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：height_ctrl.c
 * 描述    ：高度控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "height_ctrl.h"
#include "anotc_baro_ctrl.h"
#include "mymath.h"
#include "filter.h"
#include "rc.h"
#include "PID.h"
#include "ctrl.h"
#include "include.h"
#include "fly_mode.h"
#include "fly_ctrl.h"

float	set_height_e,	//期望高度差 -- 最终采纳的目标高度差
		set_height_em,	//期望高度差（积分速度源没有经过加速度修正和带通滤波）
		set_speed_t,	//遥控器数据转换为期望速度时用的中间变量，经过低通滤波得到set_speed
		set_speed,		//遥控器设置的期望速度，单位mm/s
		exp_speed,		//位置PID算出的期望速度，用于速度PID
		exp_acc,		//期望加速度（速度PID得出）
		fb_speed,
		fb_acc,
		fb_speed_old;

_hc_value_st hc_value;


u8 thr_take_off_f = 0;


_PID_arg_st h_acc_arg;		//加速度
_PID_arg_st h_speed_arg;	//速度
_PID_arg_st h_height_arg;	//高度

_PID_val_st h_acc_val;
_PID_val_st h_speed_val;
_PID_val_st h_height_val;

void h_pid_init()
{
	h_acc_arg.kp = 0.01f ;				//比例系数
	h_acc_arg.ki = 0.02f  *pid_setup.groups.hc_sp.kp;				//积分系数
	h_acc_arg.kd = 0;				//微分系数
	h_acc_arg.k_pre_d = 0 ;	
	h_acc_arg.inc_hz = 0;
	h_acc_arg.k_inc_d_norm = 0.0f;
	h_acc_arg.k_ff = 0.05f;

	h_speed_arg.kp = 1.5f *pid_setup.groups.hc_sp.kp;				//比例系数
	h_speed_arg.ki = 0.0f *pid_setup.groups.hc_sp.ki;				//积分系数
	h_speed_arg.kd = 0.0f;				//微分系数
	h_speed_arg.k_pre_d = 0.10f *pid_setup.groups.hc_sp.kd;
	h_speed_arg.inc_hz = 20;
	h_speed_arg.k_inc_d_norm = 0.8f;
	h_speed_arg.k_ff = 0.5f;	
	
	h_height_arg.kp = 1.5f *pid_setup.groups.hc_height.kp;				//比例系数
	h_height_arg.ki = 0.0f *pid_setup.groups.hc_height.ki;				//积分系数
	h_height_arg.kd = 0.05f *pid_setup.groups.hc_height.kd;				//微分系数
	h_height_arg.k_pre_d = 0.01f ;
	h_height_arg.inc_hz = 20;
	h_height_arg.k_inc_d_norm = 0.5f;
	h_height_arg.k_ff = 0;	
	
}

float thr_set,thr_pid_out,thr_out,thr_take_off,tilted_fix;

float en_old;
u8 ex_i_en;
float Height_Ctrl(float T,float thr,u8 ready,float en)	//en	1：定高   0：非定高
{
	/*
	 * 这套算法有个很有意思的地方：
	 * 加速度PID用的期望加速度值是上个周期速度PID算出来的，
	 * 速度PID用的期望速度值是上个周期位置PID算出来的
	 * 这样看这个代码的实时性其实是非常不好的，速度--加速度延迟20ms，位置--速度延迟20ms，整体来看延迟了40ms
	 * 
	 */
	
	//thr：0 -- 1000
	static u8 speed_cnt,height_cnt;
	
	//高度数据获取（获取气压计数据，调用超声波数据，融合计算高度数据）
	baro_ctrl(T,&hc_value); 
	
	//解锁情况判断，用于选择后续代码是否执行
	if(ready == 0)	//没有解锁（已经上锁）
	{
		en = 0;						//转换为手动模式，禁止自动定高代码的执行
		thr_take_off_f = 0;			//起飞标志清零
		thr_take_off = 0;			//基准油门清零
	}
	
	//油门处理：
	//取值范围转换、设置死区
	thr_set = my_deathzoom_2(my_deathzoom((thr - 500),0,40),0,10);	//±50为死区，零点为±40的位置
	
	//thr_set是经过死区设置的油门控制量输入值，取值范围 -500 -- +500
	
	//模式判断
	if(en < 0.1f)		//手动模式
	{
		en_old = en;	//更新历史模式
		
		return (thr);	//thr是传入的油门值，thr：0 -- 1000
						//把传入油门直接传出去了，上面所有算法都没用上
	}
	else
	{
		//检测模式切换
		//飞行中初次进入定高模式切换处理（安全保护，防止基准油门过低）
		if( ABS(en - en_old) > 0.5f )	//从非定高切换到定高（官方注释）	//我认为是模式在飞行中被切换，切换方向不确定
		{
			
			if(thr_take_off<10)			//未计算起飞油门（官方注释）
			{
				//thr_set是经过死区设置的油门控制量输入值，取值范围 -500 -- +500。
				//这个判断有可能造成在地面上解锁后推一点油门时，切换模式导致飞机一下子窜上天（比定高起飞还快）
				if(thr_set > -150)		//thr_set > -150 代表油门非低
				{
					thr_take_off = 400;
				}
			}
			en_old = en;	//更新历史模式
		}
		
		//======================================================================================
		

		
		if(ctrl_command == 3)	//飞行控制函数直接控制垂直速度期望
		{
			//直接由控制函数设置垂直速度期望
			
			set_speed = height_speed_ctrl;
		}
		else if(ctrl_command == 4)	//起飞
		{
			
		}
		else if(0)				//飞信控制函数直接控制高度期望
		{
			
		}
		else					//油门控制垂直速度期望
		{
			//遥控器输入值控制升降
			
			//升降判断，生成速度期望
			if(thr_set>0)	//上升
			{
				set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_UP;	//set_speed_t 表示期望上升速度占最大上升速度的比值
				
				//遥控器控制条件下的起飞处理
				if(thr_take_off_f == 0)	//如果没有起飞（本次解锁后还没有起飞）
				{
					if(thr_set>100)	//达到起飞油门
					{
						thr_take_off_f = 1;	//起飞标志置1，此标志只在上锁后会被归零
						thr_take_off = 350; //直接赋值起飞基准油门
					}
				}
			}
			else			//悬停或下降
			{
				set_speed_t = thr_set/450 * MAX_VERTICAL_SPEED_DW;
			}

			//速度期望限幅滤波
			set_speed_t = LIMIT(set_speed_t,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);	//速度期望限幅
			LPF_1_(10.0f,T,my_pow_2_curve(set_speed_t,0.25f,MAX_VERTICAL_SPEED_DW),set_speed);	//LPF_1_是低通滤波器，截至频率是10Hz，输出值是set_speed，my_pow_2_curve把输入数据转换为2阶的曲线，在0附近平缓，在数值较大的部分卸率大
			set_speed = LIMIT(set_speed,-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP);	//限幅，单位mm/s
		}
		
		//set_speed为最终输出的期望速度
		
		//======================================================================================
		
		//计算高度误差（可加滤波）
		//高度差 = ∑速度差*T （单位 mm/s）
		//h(n) = h(n-1) + △h  ， △h =（期望速度 - 当前速度） * △t
		//用解锁状态给目标高度差积分控制变量赋值，只有在ex_i_en = 1时才会开始积分计算目标高度差
		ex_i_en = thr_take_off_f;
		
		set_height_em += (set_speed -        hc_value.m_speed)      * T;	//没有经过加速度修正和带通滤波的速度值算出的速度差 * △T
		set_height_em = LIMIT(set_height_em,-5000 *ex_i_en,5000 *ex_i_en);	//ex_i_en = 1 表示已经到达起飞油门，否则为0
		
		set_height_e += (set_speed  - 1.05f *hc_value.fusion_speed) * T;	//经过加速度修正和带通滤波的速度值算出的速度差 * △T
		set_height_e  = LIMIT(set_height_e ,-5000 *ex_i_en,5000 *ex_i_en);
		
		LPF_1_(0.0005f,T,set_height_em,set_height_e);	//频率 时间 输入 输出	//两个速度差按比例融合，第一个参数越大，set_height_em的占比越大	
		
		//至此得出高度差 set_height_e ，单位 mm
		
		//===============================================================================
		//	加速度PID
			
		float acc_i_lim;
		acc_i_lim = safe_div(150,h_acc_arg.ki,0);		//acc_i_lim = 150 / h_acc_arg.ki
														//避免除零错误（如果出现除零情况，就得0）
		
		//计算加速度
		fb_speed_old = fb_speed;						//存储上一次的速度
		fb_speed = hc_value.fusion_speed;				//读取当前速度
		fb_acc = safe_div(fb_speed - fb_speed_old,T,0);	//计算得到加速度：a = dy/dt = [ x(n)-x(n-1)]/dt
		
		//fb_acc是当前加速度值（反馈回来的加速度值）
		
		//加速度PID
		thr_pid_out = PID_calculate( T,            		//周期
									 exp_acc,			//前馈				//exp_acc由速度PID给出
									 exp_acc,			//期望值（设定值）	//exp_acc由速度PID给出
									 fb_acc,			//反馈值
									 &h_acc_arg, 		//PID参数结构体
									 &h_acc_val,		//PID数据结构体
									 acc_i_lim*en		//integration limit，积分限幅     如果在手动模式，en = 0，这个结果就是0了
									);					//输出
									
		
		//基准油门调整（防止积分饱和过深）
		if(h_acc_val.err_i > (acc_i_lim * 0.2f))
		{
			if(thr_take_off<THR_TAKE_OFF_LIMIT)
			{
				thr_take_off += 150 *T;
				h_acc_val.err_i -= safe_div(150,h_acc_arg.ki,0) *T;
			}
		}
		else if(h_acc_val.err_i < (-acc_i_lim * 0.2f))
		{
			if(thr_take_off>0)
			{
				thr_take_off -= 150 *T;
				h_acc_val.err_i += safe_div(150,h_acc_arg.ki,0) *T;
			}
		}
		thr_take_off = LIMIT(thr_take_off,0,THR_TAKE_OFF_LIMIT); //限幅
		
		/////////////////////////////////////////////////////////////////////////////////
		
		//油门补偿、油门输出
		/*
				这个公式的原型是：
				z(机体) = x.z * R(地理) + y.z * R(地理) + reference_v.z * R(地理)
		
				在这里的R的值在地理坐标系Z轴上，R = [0,0,thr_take_off](转置)
		
				所以：z(机体) = x.z * 0 + y.z * 0 + reference_v.z * thr_take_off = reference_v.z * thr_take_off

								   1
				tilted_fix = -------------
							 reference_v.z
												  1
				tilted_fix * thr_take_off = ------------- * thr_take_off
											reference_v.z
		
				reference_v.z 是 地理坐标系 向 机体坐标系 转换时z轴数值乘的系数，z(机体)(地理Z轴分量) = reference_v.z * Z(地理)
		
				1 / reference_v.z 是 机体坐标系 向 地理坐标系 转换时z轴数值乘的系数，z（地理） = 1 / reference_v.z * z（机体）
		
				thr_take_off 是 本应该施加在飞机于地理坐标系Z轴上受到的力，但是由于飞机会倾斜，油门的输出值实际上是输出在机体坐标系的z轴上的
				thr_take_off 乘上了 1 / reference_v.z 这个系数，就算出了机体坐标系应该输出的值，也就是说：
		
				如果要在地理坐标系的Z轴输出 thr_take_off ，则应该在机体坐标系的z轴输出 1 / reference_v.z * thr_take_off 这个值
		
				（飞机越斜，地理坐标系Z轴在机体z轴上映射的分量越小，其cos值reference_v.z越小， 1 / reference_v.z 也就越大）
				
				==========================================================================================================

				x(机体)(地理Z轴分量) = reference_v.x * Z(地理)
				y(机体)(地理Z轴分量) = reference_v.y * Z(地理)
				z(机体)(地理Z轴分量) = reference_v.z * Z(地理)
				
				这里只有一个 Z(地理) 的力对应于 z(机体) 的力，只有地理Z轴上有数值，所以只有 reference_v.z 一个系数出现，然后把这个公式反向使用，就是上面的那个公式
				
		*/
		tilted_fix = safe_div(1,LIMIT(reference_v.z,0.707f,1),0); //45度内补偿
		thr_out = (thr_pid_out + tilted_fix *(thr_take_off) );	//由两部分组成：油门PID + 油门补偿 * 起飞油门
		thr_out = LIMIT(thr_out,0,1000);
		
		
		/////////////////////////////////////////////////////////////////////////////////	
		
		//速度PID
		
		static float dT,dT2;
		dT += T;
		speed_cnt++;
		if(speed_cnt>=10) //u8  20ms
		{
			speed_cnt = 0;
			
			exp_acc = PID_calculate( dT,           				//周期
									exp_speed,					//前馈				//exp_speed由位置PID给出
									(set_speed + exp_speed),	//期望值（设定值）	//set_speed由油门输入给出，exp_speed由位置PID给出
									hc_value.fusion_speed,		//反馈值
									&h_speed_arg, 				//PID参数结构体
									&h_speed_val,				//PID数据结构体
									500 *en						//integration limit，积分限幅
									 );							//输出
			
			exp_acc = LIMIT(exp_acc,-3000,3000);
			dT = 0;
			
			dT2 += dT;		//计算微分时间
			height_cnt++;	//计算循环执行周期
			if(height_cnt>=10)  //200ms 
			{
				height_cnt = 0;
				
				//位置PID
				//输入的反馈值是高度差而不是高度，相当于已经把error输入了，所以期望值为0时正好是 error = error - 0
				exp_speed = PID_calculate( 		dT2,            //周期
												0,				//前馈
												0,				//期望值（设定值）
												-set_height_e,	//反馈值				高度差，单位mm
												&h_height_arg, 	//PID参数结构体
												&h_height_val,	//PID数据结构体
												1500 *en		//integration limit，积分限幅
										 );			//输出	
				exp_speed = LIMIT(exp_speed,-300,300);
				
				dT2 = 0;
			}
		}
		
		return (thr_out);	//经过定高运算的油门值
	}

}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

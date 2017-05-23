/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：ctrl.c
 * 描述    ：飞控控制
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "ctrl.h"
#include "height_ctrl.h"
#include "fly_mode.h"
#include "fly_ctrl.h"

ctrl_t ctrl_1;
ctrl_t ctrl_2;

void Ctrl_Para_Init()		//设置默认参数
{
//====================================
	ctrl_1.PID[PIDROLL].kdamp  = 1;
	ctrl_1.PID[PIDPITCH].kdamp = 1;
	ctrl_1.PID[PIDYAW].kdamp   = 1;
	
	ctrl_1.FB = 0.20;   //外  0<fb<1
}

xyz_f_t except_A = {0,0,0};				//角度期望

xyz_f_t ctrl_angle_offset = {0,0,0};	



void CTRL_2(float T)
{

//=========================== 期望角度 ========================================
	
	//=================== filter ===================================
	//  全局输出，CH_filter[],0横滚，1俯仰，2油门，3航向 范围：+-500	
	//	CH_filter[]的输入数值在输出前是经过限幅的
	//=================== filter =================================== 
	
	//将±30这个区域设置为死区
	//并把输入的 -500 -- +500 这个区间的遥控器数值归一化，然后乘上最大期望值，使输入值成为当前期望值对最大期望值的占比
	
	//x轴、y轴处理
//	except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[ROL]) ,0,30 )/500.0f );
//	except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_filter[PIT]) ,0,30 )/500.0f );
	
	except_A.x  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_ctrl[ROL]) ,0,30 )/500.0f );
	except_A.y  = MAX_CTRL_ANGLE  *( my_deathzoom( ( CH_ctrl[PIT]) ,0,30 )/500.0f );
	
	//z轴处理，将输入值转化为期望角速度
	if( Thr_Low == 0 )	//这东西顶多跟起不起飞有关系，跟油门低不低有啥关系？？
	{
		//油门非低
		//设置死区，进行归一化处理，最大角度值由最大角速度值 MAX_CTRL_YAW_SPEED 对 时间的积分 来影响
//		except_A.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_filter[YAW]) ,0,40 )/500.0f ) ) *T ;
		
		except_A.z += (s16)( MAX_CTRL_YAW_SPEED *( my_deathzoom_2( (CH_ctrl[YAW]) ,0,40 )/500.0f ) ) *T ;
	}
	else	//油门低
	{
		except_A.z += 1 *3.14 *T *( Yaw - except_A.z );	//油门低状态下的z轴期望角速度与所处角度有关
														//这个地方真的有点不清楚，前面那些参数和T有关，看起来像是低通滤波器
	}
	except_A.z = To_180_degrees(except_A.z);			//将 except_A.z 的数值限制在 -180 -- +180 之间
	
//===================================================================

	/* 得到角度误差 */
	//将误差角度限制在±180°之间
	ctrl_2.err.x =  To_180_degrees( ctrl_angle_offset.x + except_A.x - Roll  );	//ctrl_angle_offset的值默认为0，没有相关的设置代码
	ctrl_2.err.y =  To_180_degrees( ctrl_angle_offset.y + except_A.y - Pitch );
	ctrl_2.err.z =  To_180_degrees( ctrl_angle_offset.z + except_A.z - Yaw	 );
	
	/* 计算角度误差权重 */
	ctrl_2.err_weight.x = ABS(ctrl_2.err.x)/ANGLE_TO_MAX_AS;	//期望角度差 与 最大倾角的比值
	ctrl_2.err_weight.y = ABS(ctrl_2.err.y)/ANGLE_TO_MAX_AS;
	ctrl_2.err_weight.z = ABS(ctrl_2.err.z)/ANGLE_TO_MAX_AS;
	
	//===============================================================================================
	//D
	
	/* 角度误差微分（跟随误差曲线变化）*/			
	ctrl_2.err_d.x = 10 *ctrl_2.PID[PIDROLL].kd  *(ctrl_2.err.x - ctrl_2.err_old.x) *( 0.005f/T ) ;
	ctrl_2.err_d.y = 10 *ctrl_2.PID[PIDPITCH].kd *(ctrl_2.err.y - ctrl_2.err_old.y) *( 0.005f/T ) ;
	ctrl_2.err_d.z = 10 *ctrl_2.PID[PIDYAW].kd 	 *(ctrl_2.err.z - ctrl_2.err_old.z) *( 0.005f/T ) ;
	
	//===============================================================================================
	//I
	
	/* 角度误差积分 */							
	ctrl_2.err_i.x += ctrl_2.PID[PIDROLL].ki  *ctrl_2.err.x *T;
	ctrl_2.err_i.y += ctrl_2.PID[PIDPITCH].ki *ctrl_2.err.y *T;
	ctrl_2.err_i.z += ctrl_2.PID[PIDYAW].ki   *ctrl_2.err.z *T;
	
	/* 角度误差积分分离 */
	ctrl_2.eliminate_I.x = Thr_Weight *CTRL_2_INT_LIMIT;	//生成积分值的最大幅度，并用于限幅
	ctrl_2.eliminate_I.y = Thr_Weight *CTRL_2_INT_LIMIT;	//油门越高，Thr_Weight越大，允许的角度误差积分越值大
	ctrl_2.eliminate_I.z = Thr_Weight *CTRL_2_INT_LIMIT;	//CTRL_2_INT_LIMIT = 0.5f *MAX_CTRL_ANGLE，Thr_Weight = LIMIT(Thr_tmp,0,1)
															//Thr_tmp 是 thr_value 经过低通滤波得到的
															//thr_value 是 经过控制算法计算的油门输出值（经过高度控制算法控制）
	/* 角度误差积分限幅 */
	ctrl_2.err_i.x = LIMIT( ctrl_2.err_i.x, -ctrl_2.eliminate_I.x,ctrl_2.eliminate_I.x );	//限幅后最大范围是±15
	ctrl_2.err_i.y = LIMIT( ctrl_2.err_i.y, -ctrl_2.eliminate_I.y,ctrl_2.eliminate_I.y );
	ctrl_2.err_i.z = LIMIT( ctrl_2.err_i.z, -ctrl_2.eliminate_I.z,ctrl_2.eliminate_I.z );
	
	//===============================================================================================
	
	/* 记录历史数据 */	
	ctrl_2.err_old.x = ctrl_2.err.x;
	ctrl_2.err_old.y = ctrl_2.err.y;
	ctrl_2.err_old.z = ctrl_2.err.z;	
	
	//===============================================================================================
	//P
	
	/* 对用于计算比例项输出的角度误差限幅 */
	ctrl_2.err.x = LIMIT( ctrl_2.err.x, -90, 90 );
	ctrl_2.err.y = LIMIT( ctrl_2.err.y, -90, 90 );
	ctrl_2.err.z = LIMIT( ctrl_2.err.z, -90, 90 );
	
	//===============================================================================================
	
	/* 角度PID输出 */
	ctrl_2.out.x = ctrl_2.PID[PIDROLL].kp  *( ctrl_2.err.x + ctrl_2.err_d.x + ctrl_2.err_i.x );	//rol
	ctrl_2.out.y = ctrl_2.PID[PIDPITCH].kp *( ctrl_2.err.y + ctrl_2.err_d.y + ctrl_2.err_i.y ); //pitch
	ctrl_2.out.z = ctrl_2.PID[PIDYAW].kp   *( ctrl_2.err.z + ctrl_2.err_d.z + ctrl_2.err_i.z );	//yaw
}

xyz_f_t except_AS;	//期望角速度

float g_old[ITEMS];

void CTRL_1(float T)  //x roll,y pitch,z yaw
{
	xyz_f_t EXP_LPF_TMP;
	
	/* 给期望（目标）角速度 */
	// 从含义上分析是期望角速度对最大角速度的占比，ctrl_2.out.x / ANGLE_TO_MAX_AS 是对 ctrl_2.out.x 的归一化
	EXP_LPF_TMP.x = MAX_CTRL_ASPEED *(ctrl_2.out.x/ANGLE_TO_MAX_AS);//
	EXP_LPF_TMP.y = MAX_CTRL_ASPEED *(ctrl_2.out.y/ANGLE_TO_MAX_AS);//
	EXP_LPF_TMP.z = MAX_CTRL_YAW_SPEED *(ctrl_2.out.z/ANGLE_TO_MAX_AS);
	
	except_AS.x = EXP_LPF_TMP.x;//20 *3.14 *T *( EXP_LPF_TMP.x - except_AS.x );//
	except_AS.y = EXP_LPF_TMP.y;//20 *3.14 *T *( EXP_LPF_TMP.y - except_AS.y );//
	except_AS.z = EXP_LPF_TMP.z;//20 *3.14 *T *( EXP_LPF_TMP.z - except_AS.z );//
	
	/* 期望角速度限幅 */
	except_AS.x = LIMIT(except_AS.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.y = LIMIT(except_AS.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
	except_AS.z = LIMIT(except_AS.z, -MAX_CTRL_YAW_SPEED,MAX_CTRL_YAW_SPEED );

	//这里开始直接使用陀螺仪的输出数据了
	
	//================================================================================================
	//D
	
	/* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
	//本次和上次的差值，并不是变化量的差值，这个和PID的标准的D不大一样
	ctrl_1.damp.x = ( mpu6050.Gyro_deg.x - g_old[A_X]) *( 0.002f/T );//ctrl_1.PID[PIDROLL].kdamp
	ctrl_1.damp.y = (-mpu6050.Gyro_deg.y - g_old[A_Y]) *( 0.002f/T );//ctrl_1.PID[PIDPITCH].kdamp *
	ctrl_1.damp.z = (-mpu6050.Gyro_deg.z - g_old[A_Z]) *( 0.002f/T );//ctrl_1.PID[PIDYAW].kdamp	 *
	
	/* 角速度微分 */
	ctrl_1.err_d.x = ( ctrl_1.PID[PIDROLL].kd  *( -10 *ctrl_1.damp.x) *( 0.002f/T ) );
	ctrl_1.err_d.y = ( ctrl_1.PID[PIDPITCH].kd *( -10 *ctrl_1.damp.y) *( 0.002f/T ) );
	ctrl_1.err_d.z = ( ctrl_1.PID[PIDYAW].kd   *( -10 *ctrl_1.damp.z) *( 0.002f/T ) );
	
	//================================================================================================
	
	/* 角速度误差 */
	ctrl_1.err.x =  ( except_AS.x - mpu6050.Gyro_deg.x ) *(300.0f/MAX_CTRL_ASPEED);
	ctrl_1.err.y =  ( except_AS.y + mpu6050.Gyro_deg.y ) *(300.0f/MAX_CTRL_ASPEED);  //-y
	ctrl_1.err.z =  ( except_AS.z + mpu6050.Gyro_deg.z ) *(300.0f/MAX_CTRL_ASPEED);	 //-z
	
	/* 角速度误差权重 */
	ctrl_1.err_weight.x = ABS(ctrl_1.err.x)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.y = ABS(ctrl_1.err.y)/MAX_CTRL_ASPEED;
	ctrl_1.err_weight.z = ABS(ctrl_1.err.z)/MAX_CTRL_YAW_SPEED;
	
//	ctrl_1.err_d.x += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDROLL].kd  *(ctrl_1.err.x - ctrl_1.err_old.x) *( 0.002f/T ) - ctrl_1.err_d.x);
//	ctrl_1.err_d.y += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDPITCH].kd *(ctrl_1.err.y - ctrl_1.err_old.y) *( 0.002f/T ) - ctrl_1.err_d.y);
//	ctrl_1.err_d.z += 40 *3.14 *0.002 *( 10 *ctrl_1.PID[PIDYAW].kd   *(ctrl_1.err.z - ctrl_1.err_old.z) *( 0.002f/T ) - ctrl_1.err_d.z);

	//================================================================================================
	//I
	
	/* 角速度误差积分 */
	ctrl_1.err_i.x += ctrl_1.PID[PIDROLL].ki  *(ctrl_1.err.x - ctrl_1.damp.x) *T;
	ctrl_1.err_i.y += ctrl_1.PID[PIDPITCH].ki *(ctrl_1.err.y - ctrl_1.damp.y) *T;
	ctrl_1.err_i.z += ctrl_1.PID[PIDYAW].ki 	*(ctrl_1.err.z - ctrl_1.damp.z) *T;
	/* 角速度误差积分分离 */
	ctrl_1.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;	//这个权重跟油门输出值有关，油门越高幅度越大
	ctrl_1.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
	ctrl_1.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
	/* 角速度误差积分限幅 */
	ctrl_1.err_i.x = LIMIT( ctrl_1.err_i.x, -ctrl_1.eliminate_I.x,ctrl_1.eliminate_I.x );
	ctrl_1.err_i.y = LIMIT( ctrl_1.err_i.y, -ctrl_1.eliminate_I.y,ctrl_1.eliminate_I.y );
	ctrl_1.err_i.z = LIMIT( ctrl_1.err_i.z, -ctrl_1.eliminate_I.z,ctrl_1.eliminate_I.z );
	
	//================================================================================================
	
	/* 角速度PID输出 */
	ctrl_1.out.x = 2 *( ctrl_1.FB    * LIMIT((0.45f + 0.55f*ctrl_2.err_weight.x),0,1)  * except_AS.x           + ( 1 - ctrl_1.FB ) *  ctrl_1.PID[PIDROLL].kp  *( ctrl_1.err.x + ctrl_1.err_d.x + ctrl_1.err_i.x ) );	//*(MAX_CTRL_ASPEED/300.0f);
	ctrl_1.out.y = 2 *( ctrl_1.FB    * LIMIT((0.45f + 0.55f*ctrl_2.err_weight.y),0,1)  * except_AS.y           + ( 1 - ctrl_1.FB ) *  ctrl_1.PID[PIDPITCH].kp *( ctrl_1.err.y + ctrl_1.err_d.y + ctrl_1.err_i.y ) );	//*(MAX_CTRL_ASPEED/300.0f);					
	ctrl_1.out.z = 4 *( ctrl_1.FB    * LIMIT((0.45f + 0.55f*ctrl_2.err_weight.z),0,1)  * except_AS.z           + ( 1 - ctrl_1.FB ) *  ctrl_1.PID[PIDYAW].kp   *( ctrl_1.err.z + ctrl_1.err_d.z + ctrl_1.err_i.z ) );	//*(MAX_CTRL_ASPEED/300.0f);													
//						ctrl_1在总输   根据本次计算中外环error值计算出的外环输出值的	归一化后的外环输出值	  ctrl_1计算结果在	   							  P				 D				  I	
//						出的占比	   影响权重										  （被认为是期望角速度值）	  总输出值的占比

	Thr_Ctrl(T);// 油门控制，这里面包含高度控制闭环
				// 输出 thr_value
	
	//电机输出（包含解锁判断，未解锁状态输出为0）
	All_Out(ctrl_1.out.x,ctrl_1.out.y,ctrl_1.out.z);	//输出值包括两部分，posture_value 和 thr_value
														//out_roll,out_pitch,out_yaw 生成 posture_value
														//在 All_Out 里这两部分按照权重参数 Thr_Weight 整合
														
	//记录历史数据
	ctrl_1.err_old.x = ctrl_1.err.x;
	ctrl_1.err_old.y = ctrl_1.err.y;
	ctrl_1.err_old.z = ctrl_1.err.z;

	g_old[A_X] =  mpu6050.Gyro_deg.x ;
	g_old[A_Y] = -mpu6050.Gyro_deg.y ;
	g_old[A_Z] = -mpu6050.Gyro_deg.z ;
}


float thr_value;
u8 Thr_Low;
float Thr_Weight;

void Thr_Ctrl(float T)
{
	static float thr;
	static float Thr_tmp;
	
//	thr = 500 + CH_filter[THR]; //油门值 0 ~ 1000
	thr = 500 + CH_ctrl[THR];	//油门值 0 ~ 1000
	
	//thr取值范围0-1000
	if( thr < 100 )	//油门低判断（用于 ALL_Out里的最低转速保护 和 ctrl2里的Yaw轴起飞前处理）
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
	
	//根据飞行模式选择油门控制方法
	//mode_state： 0 -- 姿态    1 -- 气压计   2 -- 超声波 + 气压计
	if(mode_state)	//定高模式（在此版本代码里，mode_state由fly_mode.c控制）
	{
		if(NS==0) //丢失信号
		{
			thr = LIMIT(thr,0,500);	//保持当前油门值，但不能超过半油门，500这个数在定高代码里代表悬停
									//也就是说定高模式丢信号时只能悬停或下降（依照丢信号前状态）
		}
		
		thr_value = Height_Ctrl(T,thr,fly_ready,1);   //输出经过定高算法修正的值
	}
	else					//手动模式（只有mode_state = 0时才是手动，其余的都是自动控高）
	{
		if(NS==0) //丢失信号
		{
			thr = LIMIT(thr,0,300);	//非定高模式丢信号，油门300，基本上就是悬停或者慢速下降
		}
		thr_value = Height_Ctrl(T,thr,fly_ready,0);   //实际使用值
	}
	
	thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);	//油门值限幅		//thr_value直接被用于计算电机输出（是最终的油门输出值）
	
	//计算权重参数 Thr_Weight，油门越高，权重越大（表示姿态输出在总输出中占比增大）
	Thr_tmp += 10 *3.14f *T *(thr_value/400.0f - Thr_tmp); 			//thr_value 的低通滤波值为 Thr_tmp
	Thr_Weight = LIMIT(Thr_tmp,0,1);    							//后边多处分离数据会用到这个值
	
}


float motor[MAXMOTORS];
float posture_value[MAXMOTORS];
s16 motor_out[MAXMOTORS];
void All_Out(float out_roll,float out_pitch,float out_yaw)
{
	u8 i;
	float posture_value[MAXMOTORS];		//由姿态PID输出数据生成的电机控制量
	
	out_yaw = LIMIT( out_yaw , -5*MAX_THR ,5*MAX_THR ); //50%

//==============================================================================
//电机数量切换
	
#if (MAXMOTORS == 4)	
	
	posture_value[0] = - out_roll + out_pitch + out_yaw ;
	posture_value[1] = + out_roll + out_pitch - out_yaw ;
	posture_value[2] = + out_roll - out_pitch + out_yaw ;
	posture_value[3] = - out_roll - out_pitch - out_yaw ;
	
#elif (MAXMOTORS == 6)
	//0.866 == sqrt(3)/2    4/6 == 0.667f
	posture_value[0] = - 0.866f *out_roll + out_pitch + 0.667f *out_yaw ;
	posture_value[1] = + 0.866f *out_roll + out_pitch - 0.667f *out_yaw ;
	posture_value[2] = + 0.866f *out_roll             + 0.667f *out_yaw ;
	posture_value[3] = + 0.866f *out_roll - out_pitch - 0.667f *out_yaw ;
	posture_value[4] = - 0.866f *out_roll - out_pitch + 0.667f *out_yaw ;
	posture_value[5] = - 0.866f *out_roll             - 0.667f *out_yaw ;
	
#elif (MAXMOTORS == 8)
	posture_value[0] = - 0.5f *out_roll + 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[1] = + 0.5f *out_roll + 0.5f *out_pitch - 0.5f *out_yaw ;
	posture_value[2] = + 0.5f *out_roll + 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[3] = + 0.5f *out_roll - 0.5f *out_pitch - 0.5f *out_yaw ;
	posture_value[4] = + 0.5f *out_roll - 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[5] = - 0.5f *out_roll - 0.5f *out_pitch - 0.5f *out_yaw ;
	posture_value[6] = - 0.5f *out_roll - 0.5f *out_pitch + 0.5f *out_yaw ;
	posture_value[7] = - 0.5f *out_roll + 0.5f *out_pitch - 0.5f *out_yaw ;	
	
#else

#endif
//==============================================================================

	//油门输出和姿态输出共同合成每个电机的输出
	for(i=0;i<MAXMOTORS;i++)
	{
		posture_value[i] = LIMIT(posture_value[i], -1000,1000 );
		
		motor[i] = thr_value + Thr_Weight *posture_value[i] ;	//输出值 = 油门值 + 权重 * 姿态控制值
	}
	
	/* 是否解锁 */
	if(fly_ready)
	{
		//保证基准转速，防止算法输出总输出过低导致的在空中停转
		if( !Thr_Low )  			//油门拉起
		{
			for(i=0;i<MAXMOTORS;i++)
			{
				motor[i] = LIMIT(motor[i], (10 *READY_SPEED),(10*MAX_PWM) );	//如果油门已经推起来，最低转速就被被限制在READY_SPEED
																				//防止因为算法调整导致转速过慢电机停转
			}
		}
		else						//油门低
		{
			for(i=0;i<MAXMOTORS;i++)
			{
				motor[i] = LIMIT(motor[i], 0,(10*MAX_PWM) );	//不限制最高油门，允许最低油门归0
			}
		}
	}
	else	//未解锁状态下所有电机输出值强制为0
	{
		for(i=0;i<MAXMOTORS;i++)
		{
			motor[i] = 0;
		}
	}
	
	/* 赋值给输出变量 */
	for(i=0;i<MAXMOTORS;i++)
	{
		motor_out[i] = (s16)(motor[i]);
	}

	SetPwm(motor_out,0,1000); //1000
	
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

//一段不知道有什么用的矫正
//原位置在 得到角度误差 前头

// 	static xyz_f_t acc_no_g;
// 	static xyz_f_t acc_no_g_lpf;

//xyz_f_t compensation;

//==============================================================================
// 	acc_no_g.x =  mpu6050.Acc.x - reference_v.x *4096;
// 	acc_no_g.y =  mpu6050.Acc.y - reference_v.y *4096;
// 	acc_no_g.z =  mpu6050.Acc.z - reference_v.z *4096;
// 	
// 	acc_no_g_lpf.x += 0.5f *T *3.14f * ( acc_no_g.x - acc_no_g_lpf.x );
// 	acc_no_g_lpf.y += 0.5f *T *3.14f * ( acc_no_g.y - acc_no_g_lpf.y );
// 	acc_no_g_lpf.z += 0.5f *T *3.14f * ( acc_no_g.z - acc_no_g_lpf.z );
// 	
// 	compensation.x = LIMIT( 0.003f *acc_no_g_lpf.x, -10,10 );
// 	compensation.y = LIMIT( 0.003f *acc_no_g_lpf.y, -10,10 );
// 	compensation.z = LIMIT( 0.003f *acc_no_g_lpf.z, -10,10 );
//==============================================================================	




//一套被弃用的带油门曲线的电机输出计算方案

//	float curve[MAXMOTORS];

//	curve[0] = posture_value[0] ;//(0.55f + 0.45f *ABS(posture_value[0])/1000.0f) *
//	curve[1] = posture_value[1] ;//(0.55f + 0.45f *ABS(posture_value[1])/1000.0f) *
//	curve[2] = posture_value[2] ;//(0.55f + 0.45f *ABS(posture_value[2])/1000.0f) *
//	curve[3] = posture_value[3] ;//(0.55f + 0.45f *ABS(posture_value[3])/1000.0f) *

//  motor[0] = thr_value + Thr_Weight *curve[0] ;
//	motor[1] = thr_value + Thr_Weight *curve[1] ;
//	motor[2] = thr_value + Thr_Weight *curve[2] ;
//	motor[3] = thr_value + Thr_Weight *curve[3] ;

//	motor_out[0] = (s16)(motor[0]);  
//	motor_out[1] = (s16)(motor[1]);	 
//	motor_out[2] = (s16)(motor[2]);
//	motor_out[3] = (s16)(motor[3]);

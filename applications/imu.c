/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：imu.c
 * 描述    ：姿态解算
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "imu.h"
#include "include.h"
#include "ak8975.h"
#include "mymath.h"
#include "filter.h"

#define Kp 0.3f                	// proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0f                	// 0.001  integral gain governs rate of convergence of gyroscope biases

#define IMU_INTEGRAL_LIM  ( 2.0f *ANGLE_TO_RADIAN )
#define NORM_ACC_LPF_HZ 10  		//(Hz)
#define REF_ERR_LPF_HZ  1				//(Hz)

xyz_f_t reference_v;
ref_t 	ref;

//xyz_f_t Gravity_Vec;  				//解算的重力向量
	
float Roll,Pitch,Yaw;    				//姿态角

float ref_q[4] = {1,0,0,0};
float norm_acc,norm_q;
float norm_acc_lpf;

float mag_norm ,mag_norm_xyz ;

xyz_f_t	mag_sim_3d,
		acc_3d_hg,		//排除重力加速度影响的地理坐标系的三轴加速度值
		acc_ng,			//排除重力加速度影响的机体坐标系三轴加速度值
		acc_ng_offset;

u8 acc_ng_cali;
extern u8 fly_ready;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw) 
{		
	float ref_err_lpf_hz;
	static float yaw_correct;
	float mag_norm_tmp;			//
	static xyz_f_t mag_tmp;		//
	static float yaw_mag;
	
	//mag 地磁计（magnetometer）
	
//====================================================================================================================
//	低通滤波器
//====================================================================================================================
	
	//原理：Y(n) = qX(n) + (1-q)X(n-1)    ，  q = 2π * △t * fc，fc为截止频率
	
	//输入：		(float)ak8975.Mag_Val.x /( mag_norm_xyz )	归一化的磁通量数据
	//输出：		mag_tmp.x									归一化后的经过滤波的各轴磁通量数据
	//滤波参数：	mag_norm_tmp = 20 *(6.28f *half_T);			公式：q = 2π * △t * fc，fc为截止频率
	
	mag_norm_tmp = 20 *(6.28f *half_T);	//计算滤波系数， 20为截止频率
	
	//平方根，计算方向向量的模
	mag_norm_xyz = my_sqrt(ak8975.Mag_Val.x * ak8975.Mag_Val.x + ak8975.Mag_Val.y * ak8975.Mag_Val.y + ak8975.Mag_Val.z * ak8975.Mag_Val.z);
	
	if( mag_norm_xyz != 0)
	{
		mag_tmp.x += mag_norm_tmp *( (float)ak8975.Mag_Val.x /( mag_norm_xyz ) - mag_tmp.x);
		mag_tmp.y += mag_norm_tmp *( (float)ak8975.Mag_Val.y /( mag_norm_xyz ) - mag_tmp.y);	
		mag_tmp.z += mag_norm_tmp *( (float)ak8975.Mag_Val.z /( mag_norm_xyz ) - mag_tmp.z);	
	}
	
	//滤波输出结果为：	mag_tmp.x
	//					mag_tmp.y
	//					mag_tmp.z
	
//====================================================================================================================

	/*
	void simple_3d_trans(_xyz_f_t *ref, _xyz_f_t *in, _xyz_f_t *out)
	罗盘数据是机体坐标下的，且磁场方向不是平行于地面，如果飞机倾斜，投影计算的角度会存在误差。
	此函数可在一定范围内做近似转换，让结果逼近实际角度，减小飞机倾斜的影响。
	注意：该函数内的计算并不是正确也不是准确的，正确的计算相对复杂，这里不给出，在未来的版本中会再更新。
	*/
	
	//由 reference_v 和 mag_tmp 计算出 mag_sim_3d
	//reference_v	等效重力向量（方向余弦矩阵第三列的三个数，是地理坐标系转到机体坐标系的值）
	//mag_tmp		经过滤波的归一化磁通量数据
	//mag_sim_3d	输出的角度数据
	simple_3d_trans(&reference_v,&mag_tmp,&mag_sim_3d); 	//补偿飞机倾斜对磁场角度的影响
	
	mag_norm = my_sqrt(mag_sim_3d.x * mag_sim_3d.x + mag_sim_3d.y *mag_sim_3d.y);	//归一化的磁向量的模 √(x^2 + y^2)
	
	if( mag_sim_3d.x != 0 && mag_sim_3d.y != 0 && mag_sim_3d.z != 0 && mag_norm != 0)
	{
		//算出方向角，并转换为角度制
		yaw_mag = fast_atan2( ( mag_sim_3d.y/mag_norm ) , ( mag_sim_3d.x/mag_norm) ) *57.3f; //   360/2π = 57.3f 
	}
	
	//=============================================================================
	// 计算等效重力向量（生成reference_v.x/y/z）
	
	reference_v.x = 2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2]);
	reference_v.y = 2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]);
	reference_v.z = 1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2]);//ref_q[0]*ref_q[0] - ref_q[1]*ref_q[1] - ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3];

	/*
	
	参考文献：http://www.cnblogs.com/andychenforever/p/6298073.html
	
	这是把 四元数 换算 成《方向余弦矩阵》中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	
	方向余弦矩阵第三列三个元素的含义就是地理坐标系G上的Z轴向机体坐标系B的(x,y,z)三个轴做投影的三个cos系数
	
	所以这里的v.x\v.y\v.z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
	
	reference_v 是 重力向量（在地理坐标系Z轴、竖直向下1G）在机体坐标系上映射的向量
	
	reference_v.x 是 重力向量（在地理坐标系中为竖直向下的1G）在机体坐标系上x轴的分量
	reference_v.y 是 重力向量（在地理坐标系中为竖直向下的1G）在机体坐标系上y轴的分量
	reference_v.z 是 重力向量（在地理坐标系中为竖直向下的1G）在机体坐标系上z轴的分量
	
	方向余弦矩阵的第三列对应的是 kB 矩阵的三个值，是把 地理坐标系 上的的某个向量向 机体坐标系转换的三个系数
	对应的含义是 地理坐标系 Z轴数值在 机体坐标系 三个轴上的投影
	
	x(机体)(地理Z轴分量) = reference_v.x * Z(地理)
	y(机体)(地理Z轴分量) = reference_v.y * Z(地理)
	z(机体)(地理Z轴分量) = reference_v.z * Z(地理)
	
	机体坐标系上的数值（由传感器直接测出）减去reference_v值，就是排除了重力的影响
	
	*/
	
	//=============================================================================
	//加速度计校准

	if(acc_ng_cali)	//加速度计校准，计算offset，要求此段代码执行时飞机处于静止状态
	{
		if(acc_ng_cali==2)	//acc_ng_cali在解锁一瞬间会被置2
		{
			acc_ng_offset.x = 0;
			acc_ng_offset.y = 0;
			acc_ng_offset.z = 0;
		}
		
		//加速度计的默认偏移量
		//注：ax是加速度计输入值经过低通滤波的结果，没有进行归一化和单位变换，取值范围还是0-4095
		acc_ng_offset.x += 10 *TO_M_S2 *(ax - 4096*reference_v.x) *0.0125f ;
		acc_ng_offset.y += 10 *TO_M_S2 *(ay - 4096*reference_v.y) *0.0125f ;
		acc_ng_offset.z += 10 *TO_M_S2 *(az - 4096*reference_v.z) *0.0125f ;	
		
		acc_ng_cali ++;
		if(acc_ng_cali>=82) //start on 2
		{
			acc_ng_cali = 0;
		}
	}
	
	//=============================================================================
	//计算地理坐标系Z轴加速度值
	
	//准确的xyz轴加速度（不包含重力加速度分量）
	acc_ng.x = 10 *TO_M_S2 *(ax - 4096*reference_v.x) - acc_ng_offset.x;
	acc_ng.y = 10 *TO_M_S2 *(ay - 4096*reference_v.y) - acc_ng_offset.y;
	acc_ng.z = 10 *TO_M_S2 *(az - 4096*reference_v.z) - acc_ng_offset.z;
	
	//地球坐标系上飞机的z方向加速度值（垂直于地面方向）
	//reference_v的三个数值可以用于把地理系Z轴映射到机体xyz，从物理意义上也能把机体xyz上的向量上的地理Z轴分量算出来
	//这个变量用于高度数据融合
	acc_3d_hg.z = acc_ng.x *reference_v.x + acc_ng.y *reference_v.y + acc_ng.z *reference_v.z;
	
	
	
	
	//=============================================================================
	//更新四元数
	
	// 计算加速度向量的模
	norm_acc = my_sqrt(ax*ax + ay*ay + az*az);   

	if(ABS(ax)<4400 && ABS(ay)<4400 && ABS(az)<4400 )
	{	
		//把加计的三维向量转成单位向量。
		ax = ax / norm_acc;//4096.0f;
		ay = ay / norm_acc;//4096.0f;
		az = az / norm_acc;//4096.0f; 
		
		if( 3800 < norm_acc && norm_acc < 4400 )	//在4096（1g）附近
		{
			/* 叉乘得到误差 */
			ref.err_tmp.x = ay*reference_v.z - az*reference_v.y;
			ref.err_tmp.y = az*reference_v.x - ax*reference_v.z;
//			ref.err_tmp.z = ax*reference_v.y - ay*reference_v.x;
			
			/* 误差低通 */
			ref_err_lpf_hz = REF_ERR_LPF_HZ *(6.28f *half_T);
			ref.err_lpf.x += ref_err_lpf_hz *( ref.err_tmp.x  - ref.err_lpf.x );
			ref.err_lpf.y += ref_err_lpf_hz *( ref.err_tmp.y  - ref.err_lpf.y );
//			ref.err_lpf.z += ref_err_lpf_hz *( ref.err_tmp.z  - ref.err_lpf.z );
			
			ref.err.x = ref.err_lpf.x;//
			ref.err.y = ref.err_lpf.y;//
//			ref.err.z = ref.err_lpf.z;
		}
	}
	else
	{
		ref.err.x = 0; 
		ref.err.y = 0  ;
//		ref.err.z = 0 ;
	}
	
	/* 误差积分 */
	ref.err_Int.x += ref.err.x *Ki *2 *half_T ;
	ref.err_Int.y += ref.err.y *Ki *2 *half_T ;
	ref.err_Int.z += ref.err.z *Ki *2 *half_T ;
	
	/* 积分限幅 */
	ref.err_Int.x = LIMIT(ref.err_Int.x, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.y = LIMIT(ref.err_Int.y, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ref.err_Int.z = LIMIT(ref.err_Int.z, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
	//方向角矫正
	//用 上一次的数据Yaw 对 地磁计输出的yaw_mag 进行矫正，得到 yaw_correct
	if( reference_v.z > 0.0f )
	{
		if( fly_ready  )
		{
			yaw_correct = Kp *0.2f *To_180_degrees(yaw_mag - Yaw);	//对方向角进行低通滤波
			//已经解锁，只需要低速纠正。
		}
		else
		{
			yaw_correct = Kp *1.5f *To_180_degrees(yaw_mag - Yaw);	//对方向角进行低通滤波（滤波系数较大）
			//没有解锁，视作开机时刻，快速纠正
		}
	}
	else
	{
		yaw_correct = 0; //角度过大，停止修正，修正的目标值可能不正确
	}

	//gx：陀螺仪数据x轴（x轴角速度）
	ref.g.x = (gx - reference_v.x *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.x + ref.err_Int.x) ) ;		//IN RADIAN
	ref.g.y = (gy - reference_v.y *yaw_correct) *ANGLE_TO_RADIAN + ( Kp*(ref.err.y + ref.err_Int.y) ) ;		//IN RADIAN
	ref.g.z = (gz - reference_v.z *yaw_correct) *ANGLE_TO_RADIAN;											//ANGLE_TO_RADIAN  角度转弧度   ANGLE_TO_RADIAN = 2π/360
	
	/* 用叉积误差来做PI修正陀螺零偏 */

	// integrate quaternion rate and normalise
	ref_q[0] = ref_q[0] +(-ref_q[1]*ref.g.x - ref_q[2]*ref.g.y - ref_q[3]*ref.g.z)*half_T;
	ref_q[1] = ref_q[1] + (ref_q[0]*ref.g.x + ref_q[2]*ref.g.z - ref_q[3]*ref.g.y)*half_T;
	ref_q[2] = ref_q[2] + (ref_q[0]*ref.g.y - ref_q[1]*ref.g.z + ref_q[3]*ref.g.x)*half_T;
	ref_q[3] = ref_q[3] + (ref_q[0]*ref.g.z + ref_q[1]*ref.g.y - ref_q[2]*ref.g.x)*half_T;

	/* 四元数规一化 normalise quaternion */
	norm_q = my_sqrt(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2] + ref_q[3]*ref_q[3]);
	ref_q[0] = ref_q[0] / norm_q;
	ref_q[1] = ref_q[1] / norm_q;
	ref_q[2] = ref_q[2] / norm_q;
	ref_q[3] = ref_q[3] / norm_q;
	
	//=============================================================================
	
	//解算出的三轴角度（四元数转姿态角）
	*rol = fast_atan2(2*(ref_q[0]*ref_q[1] + ref_q[2]*ref_q[3]),1 - 2*(ref_q[1]*ref_q[1] + ref_q[2]*ref_q[2])) *57.3f;
	*pit = asin(2*(ref_q[1]*ref_q[3] - ref_q[0]*ref_q[2])) *57.3f;
	*yaw = fast_atan2(2*(-ref_q[1]*ref_q[2] - ref_q[0]*ref_q[3]), 2*(ref_q[0]*ref_q[0] + ref_q[1]*ref_q[1]) - 1) *57.3f;//
	//*yaw = yaw_mag;

}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

// 		if( yaw_correct>360 || yaw_correct < -360  )
// 		{
// 			yaw_correct = 0;
// 			//限制纠正范围+-360，配合+-180度取值函数
// 		}

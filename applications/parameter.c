/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：parameter.c
 * 描述    ：参数配置等
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "include.h"
#include "mpu6050.h"
#include "ak8975.h"
#include "ctrl.h"
#include "string.h"
#include "ff.h"
#include "height_ctrl.h"

//文件名
#define SENSOR_SETUP_FILE      "sensor.bin"
#define PID_SETUP_FILE         "pid.bin"
#define USER_PARAM_FILE        "user.bin"

//参数存储结构体（参数以结构体形式整体写入）
sensor_setup_t sensor_setup;
pid_setup_t pid_setup;
user_parameter_t user_parameter;	//自定义的用户PID结构体

/* 文件相关定义 */
static FATFS fs;
static 	FIL file;
static 	DIR DirInf;

//========================================================================================================
//========================================================================================================
//											与文件系统接口
//						功能：1.读取所有参数到sensor_setup、pid_setup、user_parameter
//							  2.sensor_setup、pid_setup、user_parameter三个结构体中的参数写入FlASH
//							  3.sensor_setup、pid_setup、user_parameter中参数拷贝到运行结构体
//========================================================================================================
//========================================================================================================

//从Flash里读取数据，用于初始化变量
static int32_t Para_ReadSettingFromFile(void)
{
	FRESULT result;
	uint32_t bw;

	/* ********************** 挂载文件系统 并 打开文件所在目录 ********************** */
	
 	/* 挂载文件系统 */
	result = f_mount(&fs, "0:", 1);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		/* 如果挂载不成功，进行格式化 */
		result = f_mkfs("0:",0,0);
		if (result != FR_OK)
		{
			 return -1;     //flash有问题，无法格式化
		}
		else
		{
			/* 重新进行挂载 */
			result = f_mount(&fs, "0:", 0);			/* Mount a logical drive */
	   if (result != FR_OK)
      {
			 	/* 卸载文件系统 */
	      f_mount(NULL, "0:", 0);
			 return -2 ;
			}
		}
	}

	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "/"); 
	if (result != FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -3;
	}

	/* 打开文件 */
	result = f_open(&file, SENSOR_SETUP_FILE, FA_OPEN_EXISTING | FA_READ);	//打开Sensor配置文件
	if (result !=  FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		/* 文件不存在 */
		return -4;
	}

	/* ********************** 读取Sensor配置文件 并 存入sensor_setup结构体  ********************** */
	
	/* 读取Sensor配置文件 */
	result = f_read(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);
	if (bw > 0)
	{
		/* 关闭文件*/
		f_close(&file);
	}
	else
	{
		/* 关闭文件*/
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -5;
	}
	
	/* ********************** 读取PID配置文件 并 存入pid_setup结构体  ********************** */
	
	/* 打开文件 */
	result = f_open(&file, PID_SETUP_FILE, FA_OPEN_EXISTING | FA_READ);	//打开PID配置文件
	if (result !=  FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -6;
	}
	
	/* 读取PID配置文件 */
	result = f_read(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);	//读取文件，存入对应变量
	if(bw > 0)
	{
		/* 关闭文件*/
		f_close(&file);
	}
	else
	{
		/* 关闭文件*/
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -7;
	}
	
	/* ********************** 读取用户数据配置文件 并 存入pid_setup结构体  ********************** */
		
	result = f_open(&file, USER_PARAM_FILE, FA_OPEN_EXISTING | FA_READ);	//打开用户数据文件
	if (result !=  FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -8;
	}
	
	result = f_read(&file, &user_parameter.raw_data, sizeof(user_parameter), &bw);	//读取用户数据文件，存入对应变量
	if(bw > 0)
	{
		f_close(&file);	
	}
	else
	{
		f_close(&file);
		f_mount(NULL, "0:", 0);
		return -9;
	}
	
	/* ********************** 完成读取，卸载文件系统  ********************** */
	
	f_mount(NULL, "0:", 0);
	return 1;					//成功则return 1
}

//将参数结构体写入Flash
//每次写入都要写入所有参数
static int32_t Para_WriteSettingToFile(void)
{
	FRESULT result;
	uint32_t bw;
	
	/* ******************** 挂载文件系统 并 打开文件所在目录 ******************** */

 	/* 挂载文件系统 */
	result = f_mount(&fs, "0:", 0);			/* Mount a logical drive */
	if (result != FR_OK)
	{
		/* 如果挂载不成功，进行格式化 */
		result = f_mkfs("0:",0,0);
		if (result != FR_OK)
		{
			 return -1;     //flash有问题，无法格式化
		}
		else
		{
			/* 重新进行挂载 */
			result = f_mount(&fs, "0:", 0);			/* Mount a logical drive */
			if (result != FR_OK)
			{
				/* 卸载文件系统 */
				f_mount(NULL, "0:", 0);
				return -2 ;
			}
		}
	}

	/* 打开根文件夹 */
	result = f_opendir(&DirInf, "/"); 
	if (result != FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -3;
	}

	/* ******************** 写入Sensor配置文件 ******************** */
	
	/* 打开文件 */
	result = f_open(&file, SENSOR_SETUP_FILE, FA_CREATE_ALWAYS | FA_WRITE);	//打开Sensor配置文件
	if (result !=  FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -4;
	}

	/* 写入Sensor配置文件 */
	result = f_write(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);	//写入Sensor配置文件
	if (result == FR_OK)
	{
		/* 关闭文件*/
		f_close(&file);
	}
	else
	{
		/* 关闭文件*/
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -5;
	}
	
	/* ******************** 写入PID配置文件 ******************** */
	
	/* 打开文件 */
	result = f_open(&file, PID_SETUP_FILE, FA_CREATE_ALWAYS | FA_WRITE);
	if (result !=  FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -6;
	}
	
	/* 写入PID配置文件 */
	result = f_write(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);
	if(result == FR_OK)
	{
		/* 关闭文件*/
		f_close(&file);
	}
	else
	{
		/* 关闭文件*/
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -7;
	}
	
	/* ******************** 写入用户数据配置文件 ******************** */
	
	/* 打开文件 */
	result = f_open(&file, USER_PARAM_FILE, FA_CREATE_ALWAYS | FA_WRITE);
	if (result !=  FR_OK)
	{
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -8;
	}
	
	/* 写入用户数据配置文件 */
	result = f_write(&file, &user_parameter.raw_data, sizeof(user_parameter), &bw);
	if(result == FR_OK)
	{
		f_close(&file);
	}
	else
	{
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -9;
	}			
	
	/* ******************** 写入用户数据配置文件 ******************** */
	
	/* 卸载文件系统 */
	f_mount(NULL, "0:", 0);
	return 1;

}

//将读出的数据存入运行结构体
//存入内容：mpu6050数据、地磁计数据、内环PID（ctrl_1），外环PID（ctrl_2）
static void Param_SetSettingToFC(void)
{
	//读取传感器数值
	memcpy(&mpu6050.Acc_Offset,  &sensor_setup.Offset.Accel,       sizeof(xyz_f_t));	//加速度
	memcpy(&mpu6050.Gyro_Offset, &sensor_setup.Offset.Gyro,        sizeof(xyz_f_t));	//角速度
	memcpy(&ak8975.Mag_Offset,   &sensor_setup.Offset.Mag,         sizeof(xyz_f_t));	//地磁计
	memcpy(&mpu6050.vec_3d_cali, &sensor_setup.Offset.vec_3d_cali, sizeof(xyz_f_t));	//姿态
	
	mpu6050.Acc_Temprea_Offset = sensor_setup.Offset.Acc_Temperature;
	mpu6050.Gyro_Temprea_Offset = sensor_setup.Offset.Gyro_Temperature;
  
	//内环PID
	memcpy(&ctrl_1.PID[PIDROLL],&pid_setup.groups.ctrl1.roll,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDPITCH],&pid_setup.groups.ctrl1.pitch,sizeof(pid_t));
	memcpy(&ctrl_1.PID[PIDYAW],&pid_setup.groups.ctrl1.yaw,sizeof(pid_t));
	
	//外环PID
	memcpy(&ctrl_2.PID[PIDROLL],&pid_setup.groups.ctrl2.roll,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDPITCH],&pid_setup.groups.ctrl2.pitch,sizeof(pid_t));
	memcpy(&ctrl_2.PID[PIDYAW],&pid_setup.groups.ctrl2.yaw,sizeof(pid_t));
}

//========================================================================================================
//========================================================================================================
//										参数操作应用函数
//						基础功能：参数恢复默认值、PID运行参数初始化
//						核心功能：参数初始化（系统启动时调用）
//========================================================================================================
//========================================================================================================

//参数恢复默认值
void Para_ResetToFactorySetup(void)
{

	/* ************** 修复文件系统 ************** */
	/* 如果挂载不成功，进行格式化 */
	f_mkfs("0:",1,0);
	
	/* ************** 生成初始化结构体 ************** */
	
	/* PID 默认值 */
	pid_setup.groups.ctrl1.pitch.kp = 0.6;
	pid_setup.groups.ctrl1.roll.kp  = 0.6;	
	pid_setup.groups.ctrl1.yaw.kp   = 1.0;	
	
	pid_setup.groups.ctrl1.pitch.ki = 0.1;
	pid_setup.groups.ctrl1.roll.ki  = 0.1;	
	pid_setup.groups.ctrl1.yaw.ki   = 0.1;	
	
	
	pid_setup.groups.ctrl1.pitch.kd = 2.2;
	pid_setup.groups.ctrl1.roll.kd  = 2.2;	
	pid_setup.groups.ctrl1.yaw.kd   = 2.0;	
	
	pid_setup.groups.ctrl1.pitch.kdamp = 1;
	pid_setup.groups.ctrl1.roll.kdamp  = 1;	
	pid_setup.groups.ctrl1.yaw.kdamp   = 1;

	pid_setup.groups.ctrl2.pitch.kp = 0.5;
	pid_setup.groups.ctrl2.roll.kp  = 0.5;	
	pid_setup.groups.ctrl2.yaw.kp   = 0.8;	
	
	pid_setup.groups.ctrl2.pitch.ki = 0.05;
	pid_setup.groups.ctrl2.roll.ki  = 0.05;	
	pid_setup.groups.ctrl2.yaw.ki   = 0.05;	
	
	pid_setup.groups.ctrl2.pitch.kd = 0.3;
	pid_setup.groups.ctrl2.roll.kd  = 0.3;
	pid_setup.groups.ctrl2.yaw.kd   = 0.1;
	
	pid_setup.groups.ctrl3.kp = 1.0f;
	pid_setup.groups.ctrl3.ki = 1.0f;
	pid_setup.groups.ctrl3.kd = 1.0f;
	
	pid_setup.groups.ctrl4.kp = 1.0f;
	pid_setup.groups.ctrl4.ki = 1.0f;
	pid_setup.groups.ctrl4.kd = 1.0;
	
	pid_setup.groups.hc_sp.kp = 1.0f;
	pid_setup.groups.hc_sp.ki = 1.0f;
	pid_setup.groups.hc_sp.kd = 1.0f;
	
	pid_setup.groups.hc_height.kp = 1.0f;
	pid_setup.groups.hc_height.ki = 1.0f;
	pid_setup.groups.hc_height.kd = 1.0f;
	
	pid_setup.groups.ctrl5.kp = 1.0f;
	pid_setup.groups.ctrl5.ki = 1.0f;
	pid_setup.groups.ctrl5.kd = 1.0;
	
	pid_setup.groups.ctrl6.kp = 1.0f;
	pid_setup.groups.ctrl6.ki = 1.0f;
	pid_setup.groups.ctrl6.kd = 1.0;

	/* ************** 新结构体数值写入FLASH ************** */

	Para_WriteSettingToFile();	//默认值写入到Flash中

	/* ****************************** 用新数值初始化飞控 ****************************************** */
	/* ************** （在单独收到恢复默认值指令时会需要在这个函数里赋值运行结构体） ************** */

	Param_SetSettingToFC();		//将数值赋值到运行结构体
	PID_Para_Init();			
}

//PID运行参数初始化
//这个参数在系统启动时会被调用
//这个函数写在这里其实不大合适，但其根本意义在于更新PID后将一些参数归零
void PID_Para_Init()
{
	Ctrl_Para_Init();	//姿态PID参数初始化（ctrl.c）
	h_pid_init();		//高度PID参数初始化（用读取的高度PID参数初始化高度控制结构体）
}

//参数初始化（系统初始化时被调用）
void Para_Init()
{
	//1.读取FlASH中所有配置参数
	int32_t result = Para_ReadSettingFromFile();	//读取文件中的数据
	if(result < 0)	//读取FLASH中文件发生错误
	{
		Para_ResetToFactorySetup();	//恢复默认参数
	}

	//2.参数恢复到运行结构体（也是读取参数的一个必要步骤）
	Param_SetSettingToFC();
	
	//3.PID运行参数初始化
	PID_Para_Init();
}

//========================================================================================================
//========================================================================================================
//							参数保存功能（更新参数后用本组函数进行保存）
//					存储数据时有的需要拷贝到存储结构体，有的本身就在存储结构体中
//					功能：保存加速度计参数、保存陀螺仪参数、保存地磁计参数、
//						  保存姿态校准参数、保存PID参数
//========================================================================================================
//========================================================================================================

//加速度
void Param_SaveAccelOffset(xyz_f_t *offset)
{
	memcpy(&mpu6050.Acc_Offset,        offset,sizeof(xyz_f_t));	//拷贝到运行用的结构体
	memcpy(&sensor_setup.Offset.Accel, offset,sizeof(xyz_f_t));	//拷贝到存储用的结构体

	sensor_setup.Offset.Acc_Temperature = mpu6050.Acc_Temprea_Offset ;	//温度数值单独保存（不在xyz结构体里）

	Para_WriteSettingToFile();
}

//角速度
void Param_SaveGyroOffset(xyz_f_t *offset)
{
	memcpy(&mpu6050.Gyro_Offset,      offset,sizeof(xyz_f_t));	//拷贝到运行用的结构体
	memcpy(&sensor_setup.Offset.Gyro, offset,sizeof(xyz_f_t));	//拷贝到存储用的结构体

	sensor_setup.Offset.Gyro_Temperature = mpu6050.Gyro_Temprea_Offset ;	//温度数值单独保存（不在xyz结构体里）

	Para_WriteSettingToFile();
}

//地磁计
//更新结构体（同时更新存储结构体和运行时结构体）并保存数据
void Param_SaveMagOffset(xyz_f_t *offset)
{
	memcpy(&ak8975.Mag_Offset,       offset,sizeof(xyz_f_t));	//拷贝到运行用的结构体
	memcpy(&sensor_setup.Offset.Mag, offset,sizeof(xyz_f_t));	//拷贝到存储用的结构体
	
	Para_WriteSettingToFile();
}

//姿态（这个参数因为需要在线调参，所以只能靠Parameter_Save在上锁后保存）
void Param_Save3d_offset(xyz_f_t *offset)
{
	//memcpy  后边地址 拷贝到 前边地址
	memcpy(&mpu6050.vec_3d_cali,             offset, sizeof(xyz_f_t));	//拷贝到运行用的结构体
	memcpy(&sensor_setup.Offset.vec_3d_cali, offset, sizeof(xyz_f_t));	//拷贝到存储用的结构体

	Para_WriteSettingToFile();
}

//PID（这个参数因为需要在线调参，所以只能靠Parameter_Save在上锁后保存）
void Param_SavePID(void)
{
	//从运行结构体拷贝进入存储结构体
	
	//由于参数直接赋值到运行时结构体，所以不需要更新运行时结构体
	
	//ctrl1
	memcpy(&pid_setup.groups.ctrl1.roll,  &ctrl_1.PID[PIDROLL],  sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl1.pitch, &ctrl_1.PID[PIDPITCH], sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl1.yaw,   &ctrl_1.PID[PIDYAW],   sizeof(pid_t));

	//ctrl2
	memcpy(&pid_setup.groups.ctrl2.roll,  &ctrl_2.PID[PIDROLL],  sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl2.pitch, &ctrl_2.PID[PIDPITCH], sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl2.yaw,   &ctrl_2.PID[PIDYAW],   sizeof(pid_t));
	
	//存储数据时有的需要拷贝到存储结构体，有的本身就在存储结构体中
	Para_WriteSettingToFile();
}

//========================================================================================================
//========================================================================================================
//							上位机更新的数据在系统空闲（上锁后）保存
//========================================================================================================
//========================================================================================================

u16 flash_save_en_cnt = 0;
//参数保存（当前只有保存PID参数功能）
void Parameter_Save()
{
	if( !fly_ready )	//只有在上锁后才会调用函数存储PID数据
	{
		if( flash_save_en_cnt !=0 )
		{
			flash_save_en_cnt++;
		}

		if( flash_save_en_cnt > 60 ) // 20 *60 = 1200ms（两次存储的间隔大于1.2s）
		{
			flash_save_en_cnt = 0;	//存完了就归0，flash_save_en_cnt == 1时才会开启一次新的存储
			
			Param_SavePID();				//保存PID参数
			Param_Save3d_offset(&mpu6050.vec_3d_cali);	//保存姿态offset参数，由于此参数调参直接在运行时参数mpu6050.vec_3d_cali上调整，所以直接调用运行时参数赋值
		}
	}
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

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

#define SENSOR_SETUP_FILE      "sensor.bin"
#define PID_SETUP_FILE         "pid.bin"
#define USER_PARAM_FILE        "user.bin"

u8 flash_init_error;

sensor_setup_t sensor_setup;
pid_setup_t pid_setup;
user_parameter_t user_parameter;	//自定义的用户PID结构体

/* 文件相关定义 */
static FATFS fs;
static 	FIL file;
static 	DIR DirInf;

//从Flash里读取数据，用于初始化变量
static int32_t Para_ReadSettingFromFile(void)
{
	FRESULT result;
	uint32_t bw;

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

	/* 读取Sensor配置文件 */
	result = f_read(&file, &sensor_setup.raw_data, sizeof(sensor_setup), &bw);
	if (bw > 0)
	{
		/* 关闭文件*/
		f_close(&file);
		
		/* 打开文件 */
		result = f_open(&file, PID_SETUP_FILE, FA_OPEN_EXISTING | FA_READ);	//打开PID配置文件
		if (result !=  FR_OK)
		{
			/* 卸载文件系统 */
			f_mount(NULL, "0:", 0);
			return -4;
		}
		
		/* 读取PID配置文件 */
		result = f_read(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);	//读取文件，存入对应变量
		if(bw > 0)
		{
			/* 关闭文件*/
			f_close(&file);
			
			//用户数据读取
			
			result = f_open(&file, USER_PARAM_FILE, FA_OPEN_EXISTING | FA_READ);	//打开用户数据文件
			if (result !=  FR_OK)
			{
				/* 卸载文件系统 */
				f_mount(NULL, "0:", 0);
				return -5;
			}
			
			result = f_read(&file, &user_parameter.raw_data, sizeof(user_parameter), &bw);	//读取用户数据文件，存入对应变量
			if(bw > 0)
			{
				f_close(&file);	
				f_mount(NULL, "0:", 0);
				return 1;					//成功则return 1
			}
			else
			{
				f_close(&file);
				f_mount(NULL, "0:", 0);
				return -5;
			}
		}
		else
		{
			/* 关闭文件*/
			f_close(&file);
		 	/* 卸载文件系统 */
			f_mount(NULL, "0:", 0);
			return -4;
		}
	}
	else
	{
		/* 关闭文件*/
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -5;
	}

}

//将参数结构体写入Flash
static int32_t Para_WriteSettingToFile(void)
{
	FRESULT result;
	uint32_t bw;

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
		
		
		/* 打开文件 */
		result = f_open(&file, PID_SETUP_FILE, FA_CREATE_ALWAYS | FA_WRITE);
		if (result !=  FR_OK)
		{
			/* 卸载文件系统 */
			f_mount(NULL, "0:", 0);
			return -4;
		}
		/* 写入PID配置文件 */
		result = f_write(&file, &pid_setup.raw_data, sizeof(pid_setup), &bw);
		if(result == FR_OK)
		{
			/* 关闭文件*/
			f_close(&file);
			
			/* 打开文件 */
			result = f_open(&file, USER_PARAM_FILE, FA_CREATE_ALWAYS | FA_WRITE);
			if (result !=  FR_OK)
			{
				/* 卸载文件系统 */
				f_mount(NULL, "0:", 0);
				return -5;
			}
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
				return -4;
			}			
			
			/* 卸载文件系统 */
			f_mount(NULL, "0:", 0);
			return 1;
		}
		else
		{
			/* 关闭文件*/
			f_close(&file);
			/* 卸载文件系统 */
			f_mount(NULL, "0:", 0);
			return -4;
		}
	}
	else
	{
		/* 关闭文件*/
		f_close(&file);
		/* 卸载文件系统 */
		f_mount(NULL, "0:", 0);
		return -5;
	}

}


static void  Param_SetSettingToFC(void) 
{
	memcpy(&mpu6050.Acc_Offset,&sensor_setup.Offset.Accel,sizeof(xyz_f_t));	//加速度
	memcpy(&mpu6050.Gyro_Offset,&sensor_setup.Offset.Gyro,sizeof(xyz_f_t));	//角速度
	memcpy(&ak8975.Mag_Offset,&sensor_setup.Offset.Mag,sizeof(xyz_f_t));	//地磁计
	memcpy(&mpu6050.vec_3d_cali,&sensor_setup.Offset.vec_3d_cali,sizeof(xyz_f_t));	//姿态
	
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

void Para_ResetToFactorySetup(void)
{
	/* 如果挂载不成功，进行格式化 */
		f_mkfs("0:",1,0);
	
// 	/* 加速计默认校准值 */
// 	sensor_setup.Offset.Accel.x = 0;
// 	sensor_setup.Offset.Accel.y = 0;
// 	sensor_setup.Offset.Accel.z = 0;
// 	/* 陀螺仪默认校准值 */
// 	sensor_setup.Offset.Gyro.x = 0;
// 	sensor_setup.Offset.Gyro.y = 0;
// 	sensor_setup.Offset.Gyro.z = 0;
// 	/* 罗盘默认校准值 */
// 	sensor_setup.Offset.Mag.x = 0;		
// 	sensor_setup.Offset.Mag.y = 0;		
// 	sensor_setup.Offset.Mag.z = 0;	
// 	/* 气压计默认校准值 */	
// 	sensor_setup.Offset.Baro = 0;
//    /* 温度默认校准值 */	
// 	sensor_setup.Offset.Acc_Temperature = 0;
// 	sensor_setup.Offset.Gyro_Temperature = 0;
	
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
	
	Para_WriteSettingToFile();
	Param_SetSettingToFC();
	PID_Para_Init();
}

void PID_Para_Init()
{
	Ctrl_Para_Init();	//姿态PID参数初始化（ctrl.c）
	h_pid_init();		//高度PID参数初始化

}

//参数初始化
void Para_Init()
{
	int32_t result = Para_ReadSettingFromFile();	//读取文件中的数据
	if(result < 0)
	{
		Para_ResetToFactorySetup();
		flash_init_error = 1;
	}
	Param_SetSettingToFC();
	
	PID_Para_Init();
}

//加速度
void Param_SaveAccelOffset(xyz_f_t *offset)
{
 memcpy(&mpu6050.Acc_Offset,offset,sizeof(xyz_f_t));
 memcpy(&sensor_setup.Offset.Accel, offset,sizeof(xyz_f_t));
	
 sensor_setup.Offset.Acc_Temperature = mpu6050.Acc_Temprea_Offset ;
	
 Para_WriteSettingToFile();
}

//角速度
void Param_SaveGyroOffset(xyz_f_t *offset)
{
	memcpy(&mpu6050.Gyro_Offset,offset,sizeof(xyz_f_t));
	memcpy(&sensor_setup.Offset.Gyro, offset,sizeof(xyz_f_t));

	sensor_setup.Offset.Gyro_Temperature = mpu6050.Gyro_Temprea_Offset ;

	Para_WriteSettingToFile();
}

//地磁计
void Param_SaveMagOffset(xyz_f_t *offset)
{
	memcpy(&ak8975.Mag_Offset,offset,sizeof(xyz_f_t));
	memcpy(&sensor_setup.Offset.Mag, offset,sizeof(xyz_f_t));
	
	Para_WriteSettingToFile();
}

//姿态
void Param_Save_3d_offset(xyz_f_t *offset)
{
	memcpy(&mpu6050.vec_3d_cali,offset,sizeof(xyz_f_t));
	memcpy(&sensor_setup.Offset.vec_3d_cali, offset,sizeof(xyz_f_t));

	Para_WriteSettingToFile();
}

void Param_SavePID(void)
{
	memcpy(&pid_setup.groups.ctrl1.roll,&ctrl_1.PID[PIDROLL],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl1.pitch,&ctrl_1.PID[PIDPITCH],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl1.yaw,&ctrl_1.PID[PIDYAW],sizeof(pid_t));

	memcpy(&pid_setup.groups.ctrl2.roll,&ctrl_2.PID[PIDROLL],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl2.pitch,&ctrl_2.PID[PIDPITCH],sizeof(pid_t));
	memcpy(&pid_setup.groups.ctrl2.yaw,&ctrl_2.PID[PIDYAW],sizeof(pid_t));
	
	Para_WriteSettingToFile();
}
extern u16 flash_save_en_cnt;

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
			
			Param_SavePID();
		}
	}
}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

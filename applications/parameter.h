#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "stm32f4xx.h"

typedef struct
{
	float kp;
	float kd;
	float ki;
	float kdamp;

}pid_t;

typedef struct 
{
	float x;
	float y;
	float z;
}xyz_f_t;

typedef struct 
{
  s16 x;
	s16 y;
	s16 z;

}xyz_s16_t;

//传感器默认参数
typedef union
{
	uint8_t raw_data[64];
	struct
	{
		xyz_f_t Accel;
		xyz_f_t Gyro;
		xyz_f_t Mag;
		xyz_f_t vec_3d_cali;	//存储数据结构体中的vec_3d_cali
		uint32_t mpu_flag;
		float Acc_Temperature;
		float Gyro_Temperature;
	}Offset;
}sensor_setup_t; //__attribute__((packed)) 

//控制PID组 pid_group_t
typedef struct
{
	pid_t roll;
	pid_t pitch;	
	pid_t yaw;
	
}pid_group_t;

//PID结构体
typedef union
{
	uint8_t raw_data[192];
	struct
	{
		pid_group_t ctrl1;	//有3*3=9个
		pid_group_t ctrl2;

		pid_t hc_sp;		//3个
		pid_t hc_height;	//3个
		pid_t ctrl3;		//3个
		pid_t ctrl4;		//3个

		pid_t ctrl5;		//3个
		pid_t ctrl6;		//3个
		
	}groups;

}pid_setup_t;

//用户数据结构体
typedef union
{
	uint8_t raw_data[192];
	struct
	{
		//目前只自定义了两组pid参数
		pid_t self_def_1;
		pid_t self_def_2;
		float param_A;
		float param_B;
		float param_C;
		float param_D;
		float param_E;
		float param_F;
		float param_G;
		float param_H;
		float param_I;
		float param_J;
		float param_K;
		float param_L;
	}groups;

}user_parameter_t;

extern sensor_setup_t sensor_setup;
extern pid_setup_t pid_setup;
extern user_parameter_t user_parameter;	//自定义的用户PID结构体

void Senser_Calibrat_Read(void);
void PID_Para_Read(void);
void Para_Init(void);

void Para_ResetToFactorySetup(void);
void Param_SavePID(void);
void Param_SaveAccelOffset(xyz_f_t *offset);
void Param_SaveGyroOffset(xyz_f_t *offset);
void Param_SaveMagOffset(xyz_f_t *offset);
void Param_Save3d_offset(xyz_f_t *offset);
void Parameter_Save(void);
void PID_Para_Init(void);

extern u16 flash_save_en_cnt;

#endif


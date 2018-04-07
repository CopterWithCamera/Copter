#include "track_mode.h"

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
#include "position_function_flow.h"
#include "height_function.h"
#include "fly_ctrl.h"

//====================================================================================================
//====================================================================================================

/*
	//��̬
	
	roll_speed,		//�ٶȺ������ģʽ	0���ֶ�����		1������ͷ���ݶ���		2���������ݶ���
	pitch_speed,	//�ٶȸ�������ģʽ	0���ֶ�����		1������ͷ���ݶ���		2���������ݶ���
	roll_position,	//λ�ú������		0�����0			1���������ͷ����ƫ��
	pitch_position,	//λ�ø�������		0�����0			1���������ͷ����ƫ��
	
	//����
	
	yaw_mode;		//����ǿ���			0���ֶ�����
*/

u8 copter_fly_mode = 0;			//����ģʽ		0���ֶ�		1����Բ����		2��ǰ���ҳ�		3���ƶ��������		4������

//�ֶ�
void Copter_Attitude_Hand(float T)
{
	roll_speed = 0;
	pitch_speed = 0;
	
	roll_position = 0;
	pitch_position = 0;
	
	yaw_mode = 0;
}

//��ͣ
void Copter_Hover(float T)
{
	roll_speed = 1;
	pitch_speed = 1;
	
	roll_position = 1;
	pitch_position = 1;
	
	yaw_mode = 0;
}

//ǰ���ҳ�
void Copter_Search(float T)
{
	//�ٶȿ�����ǰ��״̬�����ٶȿ���
	pitch_speed = 3;
	roll_speed = 3;		//����ͷǰ��
	
	//ֻ����ˮƽλ������
	roll_position = 1;	//λ��
	pitch_position = 0;	//λ�������ر�
	
	yaw_mode = 0;
}

//�ƶ��������
void Copter_Track(float T)
{
	//��ȫʹ������ͷ����
	
	roll_speed = 5;		//����ר���ٶȻ�
	pitch_speed = 5;
	
	roll_position = 2;		//����ר��λ�û�
	pitch_position = 2;
	
	yaw_mode = 0;
}

//����һЩ���ڽ���
void Copter_Back_To_Land(float T)
{
	//�ٶȿ����ú���״̬�����ٶȿ���
	roll_speed = 4;
	pitch_speed = 4;
	
	//ֻ����ˮƽλ������
	roll_position = 1;
	pitch_position = 0;
	
	yaw_mode = 0;
}

//====================================================================================================

void Fly_Mode_Ctrl(float T)		//����ģʽ�л����ƺ���
{
	static u8 copter_fly_mode_old = 0;
	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	//����Ҫ���ݲ�ͬ��ָ��� copter_fly_mode �����л�
	
	if(ctrl_command != 0)	//����ָ�����ʱctrl_command��Ϊ0
	{
		if(ctrl_command == 1)
		{
			copter_fly_mode = 0;		//�ֶ�
		}
		
		if(ctrl_command == 2)
		{
			copter_fly_mode = 0;		//�ֶ�
		}
		
		if(ctrl_command == 3)
		{
			copter_fly_mode = 1;		//��ͣ

			speed_roll_clear();
			speed_pitch_clear();
			position_roll_clear();
			position_pitch_clear();
		}
		
		if(ctrl_command == 4)
		{
			copter_fly_mode = 2;		//ǰ��
		}

		if(ctrl_command == 5)
		{
			copter_fly_mode = 3;		//����
		}
		
		if(ctrl_command == 6)
		{
			copter_fly_mode = 4;		//���˽���
		}
		
		ctrl_command = 0;	//����ָ���ڴ�����

	}
	
	//��̬����ģʽ�л�
	switch(copter_fly_mode)
	{	
		case 0:
			Copter_Attitude_Hand(T);
		break;
		
		case 1:
			Copter_Hover(T);			//����ģʽ
		break;
			
		case 2:
			Copter_Search(T);
		break;
		
		case 3:
			Copter_Track(T);
		break;
		
		case 4:
			Copter_Back_To_Land(T);
		break;
		
		default:
			Copter_Attitude_Hand(T);
		break;
	}
	
	copter_fly_mode_old = copter_fly_mode;
}

//====================================================================================================
//====================================================================================================

/*

	//�߶�

	height_mode,		//�߶ȿ���ģʽ		0���ֶ��ظ�		1��������ǰ�߶�		2������ָ��߶ȿظ�		3�����		4������

*/


u8 copter_height_mode = 0;		//�߶�ģʽ		0���ֶ�		1������		2�����		3������		

void Copter_Height_Hand(float T)	//0
{
	height_mode = 0;
}

void Copter_Height_Lock(float T)	//1
{
	height_mode = 1;
}

void Copter_Take_Off(float T)		//2
{
	height_mode = 3;
}

void Copter_Land(float T)			//3
{
	height_mode = 4;
}

void Copter_Height_Set(float T)			//3
{
	height_mode = 2;		//ָ��ظ�
}

//================================================================================================================

void Height_Mode_Ctrl(float T)		//�߶�ģʽ�л����ƺ���
{
	static u8 copter_height_mode_old = 0;
	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	//�߶ȿ���ģʽ�л�
	
	if(height_command != 0)	//����ָ�����ʱ height_command ��Ϊ0
	{
		if(height_command == 1)
		{
			copter_height_mode = 0;		//�ֶ�
		}
		
		if(height_command == 2)
		{
			copter_height_mode = 1;		//����
		}
		
		if(height_command == 3)
		{
			copter_height_mode = 3;		//����
		}
		
		if(height_command == 4)
		{
			if(fly_ready)	//ֻ�н�������ܽ������ģʽ
			{
				copter_height_mode = 2;		//���
			}
		}
		
		if(height_command == 5)
		{
			copter_height_mode = 4;		//ָ��ظ�
		}
		
		height_command = 0;	//����ָ���ڴ�����
	}
	
	//��̬����ģʽ�л�
	switch(copter_height_mode)
	{
		case 0:		//�ֶ�
			Copter_Height_Hand(T);
		break;
		
		case 1:		//����
			Copter_Height_Lock(T);
		break;
		
		case 2:		//���
			Copter_Take_Off(T);
		break;
		
		case 3:		//����
			Copter_Land(T);
		break;
		
		case 4:		//ָ��ظ�
			Copter_Height_Set(T);
		break;
		
		default:	//�ֶ�
			Copter_Height_Hand(T);
		break;
	}
	
	copter_height_mode_old = copter_height_mode;
}


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
#include "ano_of.h"
#include "position_function.h"
#include "position_function_flow.h"
#include "height_function.h"

float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ
float my_except_height = 0;//�����߶�
u8 my_height_mode = 0;	//ģʽʹ�õĶ���ģʽ
						//0������
						//1�������߶�
						//2�������ٶ�

//========================================================================================
//========================================================================================
//										ָ�����
//========================================================================================
//========================================================================================

void set_except_height(u8 height)	//����վ����߶����ݣ���λcm��ȡֵ��Χ0cm-255cm
{
	my_except_height = height * 10;
}

void set_attitude_calibration(u8 cmd)	//��̬�ǵ���ָ��
{
	switch(cmd)
	{
		//ǰ�� -- ���� -- y��
		
		case 0x01:	//ǰ
			mpu6050.vec_3d_cali.y = mpu6050.vec_3d_cali.y + 0.1f;
		break;
		
		case 0x02:	//��
			mpu6050.vec_3d_cali.y = mpu6050.vec_3d_cali.y - 0.1f;
		break;
		
		//���� -- ��� -- x��
		
		case 0x03:	//��
			mpu6050.vec_3d_cali.x = mpu6050.vec_3d_cali.x + 0.1f;
		break;
		
		case 0x04:	//��
			mpu6050.vec_3d_cali.x = mpu6050.vec_3d_cali.x - 0.1f;
		break;
		
		//�洢
		
		case 0x05:	//�洢
			flash_save_en_cnt = 1;	//�洢ʹ��
		break;
		
		//����
		
		case 0x06:	//���㣨���洢��
			mpu6050.vec_3d_cali.x = 0;
			mpu6050.vec_3d_cali.y = 0;
		break;
		
		default:
			
		break;
	}
}

void set_all_out_switch(u8 cmd)		//����������ָ��
{
	switch(cmd)	//��һ�����ݣ�u8��
	{
		case 0x01:	//��ͣ
			All_Out_Switch = 0;	//��ֹ���
		break;
			
		case 0x02:	//�⼱ͣ
			All_Out_Switch = 1;	//�������
		break;
			
		default:
			
		break;
	}
}

/************************************************************************************
						��Ƶ�������Զ����ƺ���

	mode_state��	0���ֶ�		1����ѹ��	2��������+��ѹ��		3���Զ�

*************************************************************************************/

u8	height_mode = 0,	//�߶ȿ���ģʽ		0���ֶ��ظ�		1��������ǰ�߶�		2������ָ��߶ȿظ�		3�����				4������
	roll_speed = 0,		//�ٶȺ������ģʽ	0���ֶ�����		1������ͷ���ݶ���												4���������ݶ���
	pitch_speed = 0,	//�ٶȸ�������ģʽ	0���ֶ�����		1������ͷ���ݶ���	2������ͷ����ǰ��		3������ͷ���ݺ���	4���������ݶ���		5����������ǰ��		6���������ݺ���
	roll_position = 0,	//λ�ú������		0�����0			1���������ͷ����ƫ��
	pitch_position = 0,	//λ�ø�������		0�����0			1���������ͷ����ƫ��
	yaw_mode = 0;		//����ǿ���			0���ֶ�����

u8 my_fly_mode = 0, my_fly_mode_old = 0;	//����ģʽ
void Fly_Mode_Ctrl(float T)		//����ģʽ�л����ƺ���
{
	static float break_counter = 0;	//ɲ����ʱ��ʱ��
	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	//�߶ȿ���ģʽ�л�
	switch(height_command)
	{
		case 0:
			height_mode = 0;
		break;
		
		case 1:
			height_mode = 1;
		break;
		
		case 2:
			height_mode = 4;
		break;
		
		default:
			height_mode = 0;
		break;
	}
	
	//*****************************************************************************
	//����Ҫ���ݲ�ͬ��ָ��� my_fly_mode �����л�
	
	if(ctrl_command != 0)	//����ָ�����ʱctrl_command��Ϊ0
	{
		if(ctrl_command == 1)
		{
			my_fly_mode = 0;		//�ַ�
		}
		
		if(ctrl_command == 2)
		{
			my_fly_mode = 1;		//����Զ��������ã�
		}
		
		if(ctrl_command == 3)
		{
			my_fly_mode = 3;		//��������Զ��������ã�
		}
		
		if(ctrl_command == 4)
		{
			my_fly_mode = 5;		//����ͷǰ��
		}
		
		if(ctrl_command == 5)
		{
			if(my_fly_mode_old == 5)	//�����һ��״̬��ǰ��
			{
				ctrl_command = 7;		//ǰ����ɲ��
				break_counter = 0;
			}
			else if(my_fly_mode_old == 6)	//�����һ��״̬�Ǻ���
			{
				ctrl_command = 8;		//������ɲ��
				break_counter = 0;
			}
			else
			{
				my_fly_mode = 2;		//����ͷ��ͣ
			}
		}
		
		if(ctrl_command == 6)
		{
			my_fly_mode = 6;		//����ͷ����
		}
		
		if(ctrl_command == 7)	//ǰ����ɲ��
		{
			my_fly_mode = 7;
			
			//����ʱ
			break_counter++;
			if((break_counter*T)>0.5f)	//����ʱ0.5s
			{
				break_counter = 0;
				ctrl_command = 0;
				my_fly_mode = 2;
			}
		}
		
		if(ctrl_command == 8)	//������ɲ��
		{
			my_fly_mode = 8;

			//����ʱ
			break_counter++;
			if((break_counter*T)>0.5f)	//����ʱ0.5s
			{
				break_counter = 0;
				ctrl_command = 0;
				my_fly_mode = 2;
			}
		}
		
		//����
		if(ctrl_command<=6)
		{
			ctrl_command = 0;	//����ָ���ڴ�����
		}
	}
	
	//��̬����ģʽ�л�
	switch(my_fly_mode)
	{
		case 0:									//�ַ�
			roll_position = 0;
			roll_speed = 0;
			pitch_position = 0;
			pitch_speed = 0;
			yaw_mode = 0;
		break;
		
		case 1:									//����Զ������ԣ�
			pitch_position = 0;	//�����ַ�
			pitch_speed = 0;
			roll_position = 1;	//����Զ�
			roll_speed = 1;
			yaw_mode = 0;
		break;
		
		case 2:									//����ͷ����
			pitch_position = 1;	//�����Զ�
			pitch_speed = 1;
			roll_position = 1;	//����Զ�
			roll_speed = 1;
			yaw_mode = 0;
		break;
		
		case 3:									//��������Զ������ԣ�
			pitch_position = 0;	//�����ֶ�
			pitch_speed = 0;
			roll_position = 1;	//��������Զ�
			roll_speed = 4;
			yaw_mode = 0;
		break;
		
		case 4:									//���� ����
			pitch_position = 1;	//���������Զ�
			pitch_speed = 4;
			roll_position = 1;	//��������Զ�
			roll_speed = 4;
			yaw_mode = 0;
		break;
		
		case 5:									//ǰ��
			pitch_position = 0;	//����ͷǰ��
			pitch_speed = 2;
			roll_position = 1;	//����Զ�
			roll_speed = 1;
			yaw_mode = 0;
		break;
		
		case 6:									//����
			pitch_position = 0;	//����ͷ����
			pitch_speed = 2;
			roll_position = 1;	//����Զ�
			roll_speed = 1;
			yaw_mode = 0;
		break;
			
		case 7:									//ǰ����ɲ��
			pitch_position = 0;	
			pitch_speed = 7;	//ǰ����ɲ��
			roll_position = 1;	//����Զ�
			roll_speed = 1;
			yaw_mode = 0;
		break;
		
		case 8:									//������ɲ��
			pitch_position = 0;	
			pitch_speed = 7;	//������ɲ��
			roll_position = 1;	//����Զ�
			roll_speed = 1;
			yaw_mode = 0;
		break;
		
		default:
			
		break;
	}
	
	my_fly_mode_old = my_fly_mode;
}

void Fly_Height_Ctrl(float T)	//�߶ȿ��ƺ���
{
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	switch(height_mode)
	{
		case 0:
			hand();			//�ֶ�
		break;
		
		case 1:
			height_lock();	//������ǰ�߶�
		break;
		
		case 2:
			height_hold();	//���ݸ߶������ظ�
		break;
		
		case 3:
			take_off(T);	//���
		break;
		
		case 4:
			land();			//����
		break;
		
		default:
			hand();
		break;
	}
	
	//��������
	if(height_mode != 1)
		height_lock_clear();
}

void Fly_Ctrl(float T)		//��������5ms
{
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	/* ********************* �ۺϿ��� ********************* */
	
	if(roll_speed == 0)
	{
		attitude_roll();
	}
	
	if(pitch_speed == 0)
	{
		attitude_pitch();
	}
	
	if(pitch_speed == 7)	//ǰ���е�ɲ��
	{
		speed_pitch_forward_break(T);
	}
	
	if(pitch_speed == 7)	//�����е�ɲ��
	{
		speed_pitch_backward_break(T);
	}
	
	if(yaw_mode == 0)
	{
		attitude_yaw();
	}
	
	if( roll_position == 0 )
	{
		position_roll_zero();
	}
	
	if( pitch_position == 0 )
	{
		position_pitch_zero();
	}
}

void Fly_Ctrl_Cam(float T)		//����������camera������ͬ
{	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	//λ�ÿ���
	
	if( roll_position == 1 )	//����ͷλ�����
	{
		position_roll(T);
	}
	
	if( pitch_position == 1 )	//����ͷλ�����
	{
		position_pitch(T);
	}
	
	//�ٶȿ���
	
	if( roll_speed == 1 )	//����ͷ����
	{
		speed_roll();
	}
	
	if( pitch_speed == 1 )	//����ͷ����
	{
		speed_pitch();
	}
	
	if( pitch_speed == 2 )	//����ͷǰ��
	{
		speed_pitch_forward();
	}
	
	if( pitch_speed == 3 )	//����ͷ����
	{
		speed_pitch_backward();
	}
	
	//��������
	
	if( roll_position == 0 )
	{
		position_roll_clear();
	}
	
	if( pitch_position == 0 )
	{
		position_pitch_clear();
	}
	
	if( roll_speed == 0 )
	{
		speed_roll_clear();
	}
	
	if( pitch_speed == 0 )
	{
		speed_pitch_clear();
	}
}

void Fly_Ctrl_Flow(void)		//����������camera������ͬ
{	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	if(roll_speed == 4)	//��������
	{
		speed_flow_roll();
	}
	
	if(pitch_speed == 4)	//��������
	{
		speed_flow_pitch();
	}
	
	//��������
	
	if(roll_speed == 0)	//��������
	{
		speed_flow_roll_clear();
	}
	
	if(pitch_speed == 0)	//��������
	{
		speed_flow_pitch_clear();
	}
}


//========================================================================================
//========================================================================================
//				   ʶ�����ָ���mode_check�����е��ã�������ͨ����ֵ��
//========================================================================================
//========================================================================================
u8 ctrl_command;
u8 height_command;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{
	
	//====================================================================
	//����������ctrl_command�����п��ƺ������յ�������
	
	u8 aux2_in = 0;
	static u8 aux2_in_old = 0;	//��һ�ε�ָ��
	
	//����AUX2ͨ������6ͨ��������ֵ�����Զ�����ָ��
	if(*(ch_in+AUX2) < -350)			//-499 -- -350
	{
		aux2_in = 0;
	}
	else if(*(ch_in+AUX2) < -150)		//-350 -- -150
	{
		aux2_in = 1;
	}
	else if(*(ch_in+AUX2) < 0)			//-150 -- 0
	{
		aux2_in = 2;
	}
	else if(*(ch_in+AUX2) < 150)		//0 -- 150
	{
		aux2_in = 3;
	}
	else if(*(ch_in+AUX2) < 350)		//150 -- 350				
	{
		aux2_in = 4;
	}
	else								//350 -- 499
	{
		aux2_in = 5;
	}
	
	if(aux2_in != aux2_in_old)	//����ָ����仯
	{
		ctrl_command = aux2_in+1;	//����ָ���Ϊ1-6��0Ϊָ���Ѿ���ȡ��Ϻ�ĵȴ�ֵ
	}
	
	aux2_in_old = aux2_in;	//��¼��ʷ����
	
	//====================================================================
	
	//����AUX3ͨ�����Ƹ߶�
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
	
	//������ʹ��
	//All_Out_Switch = 0ʱ��ͣ��All_Out_Switch = 1ʱ��������
	//*(ch_in+AUX4)>0ʱ���ر��⶯���·���������ֹ���ָ��
	static u8 out_flag = 0;
	static u8 stop_flag = 0;
	if(*(ch_in+AUX4) > 0)			//SWA,���ʹ�ܿ��أ�ֻ���ڿ��ز����Ϸ�ʱ������ܱ������PWM
	{
		out_flag = 0;	//��out_flag
		
		if(stop_flag == 0)
		{
			stop_flag = 1;
			All_Out_Switch = 0;	//��ֹ���
		}
		
	}
	else
	{
		stop_flag = 0;	//��stop_flag
		
		if(out_flag == 0)
		{
			out_flag = 1;
			All_Out_Switch = 1;	//�������
		}
	}
}

//****************************************************************************************

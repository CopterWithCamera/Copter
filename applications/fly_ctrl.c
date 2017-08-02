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

void set_attitude_calibration(u8 cmd)
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

void set_all_out_switch(u8 cmd)
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

//========================================================================================
//========================================================================================
//										�߶ȿ���
//========================================================================================
//========================================================================================

//�ַ�ģʽ
void hand(void)
{
	//=================== filter ===================================
	//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
	//=================== filter ===================================

	//ҡ�˿ظ�
	my_height_mode = 0;
	CH_ctrl[THR] = CH_filter[THR];	//2������ THR
	
	if(NS==0) //��ʧ�ź�
	{
		CH_ctrl[THR] = LIMIT(CH_ctrl[THR],-499,0);	//���ֵ�ǰ����ֵ�������ܳ��������ţ�500������ڶ��ߴ����������ͣ
												//Ҳ����˵����ģʽ���ź�ʱֻ����ͣ���½������ն��ź�ǰ״̬��
	}
	
	//���������ſ���Thr_Lowλ
	//thrȡֵ��Χ0-1000����ΪCH_filter��ȡֵ��Χ+-500
	if( CH_ctrl[THR] < -400 )	//���ŵ��жϣ����� ALL_Out ������ת�ٱ��� �� ctrl2 ���Yaw�����ǰ����
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
}

//������ǰ�߶ȣ�����ģʽʱ������ǰ�߶ȣ���������λ��λ�ÿ��ƣ�
void height_lock(u8 en)	//en -- ģʽ���ñ�־λ�������жϴ�ģʽ�Ƿ�ʹ��
{
	static u8 height_lock_flag = 0;
	
	if(en)
	{
		//�߶ȿ���ģʽ������
		my_height_mode = 1;
		
		Thr_Low = 0;	//���ŵͱ�־��0����ֹ����������ͣת
		
		if(height_lock_flag == 0)
		{
			height_lock_flag = 1;
			
			//���������߶�
			
			#if (HEIGHT_SOURCE == 1)
				my_except_height = sonar_fusion.fusion_displacement.out;	//��ȡ��ǰ�߶�
			#elif (HEIGHT_SOURCE == 2)
				my_except_height = sonar.displacement;						//��ȡ��ǰ�߶�
			#endif
			
			if( my_except_height < 150)		//�ݴ�����ֹ�����߶ȹ���
				my_except_height = 150;
		}
	}
	else
	{
		height_lock_flag = 0;
	}
}

//����
void land(void)	//����߶��൱�ڽ����ˣ�����ܴ�Լ��15cm��һ��㣩
{
	Thr_Low = 1;	//���ŵͱ�־Ϊ1������������ͣת
	
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 100;		//�½���100mm = 10cm
	
	if(sonar.displacement < 140)	//С��14cm
	{
		fly_ready = 0;	//����
	}
}

//�½���15cm
void falling_to_15cm(void)	//����߶��൱�ڽ����ˣ�����ܴ�Լ��15cm��һ��㣩
{
	Thr_Low = 1;	//���ŵͱ�־Ϊ1������������ͣת
	
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 150;		//�½���150mm = 15cm
}

//������50cm
void rising_to_50cm(void)
{
	Thr_Low = 0;	//���ŵͱ�־��0����ֹ����������ͣת
	
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 500;		//������50cm
}

//=====================================================================================================================================
//=====================================================================================================================================
//
//									                     ��̬��λ�ã����ƺ���
//
//					����ʹ�õĲ�����	float bias��float bias_detect��float bias_real��float bias_lpf	�����Ҹ�����ֵΪ����ƫ����ֵΪ����ƫ�ң�
//									float angle��
//									float speed
//
//					���������		CH_ctrl[0]	������							�����������������м��ٶȣ����������м��ٶȣ�
//
//=====================================================================================================================================
//=====================================================================================================================================

/***********************************************

					λ�ÿ��ƺ���

	���룺	en		ģʽ���ñ�־			0��ͣ��		1������
			mode	λ�ÿ��ƹ���ģʽ		0������		1��ǰ��		2������
			
	�����	position_roll_out			���roll�����ٶ���������λcm/s			�����Ҹ�
			position_pitch_out			���pitch�����ٶ���������λcm/s			ǰ����
			position_mode_out			0������		1��ǰ��		2������

************************************************/
float position_roll_out = 0.0f;
float position_pitch_out = 0.0f;
u8 position_mode_out = 0;
void position_ctrl(u8 en,u8 mode)
{
	static float position_integration_roll = 0;
	static float position_integration_pitch = 0;
	
	position_mode_out = mode;
	
	if(en == 0)
	{
		position_integration_roll = 0;
		position_integration_pitch = 0;
	}
		
	/*
		bias_error_flag		bias_detect����ֵ�쳣ָʾ		0������		1�����쳣�лָ�������ĵ�һ֡    2���쳣
		bias		ԭʼֵ					+ <---  ---> -
		bias_detect ԭʼֵ��ͳ���˲����		+ <---  ---> -
		bias_real	У��ֵ					+ <---  ---> -
		bias_lpf	У��ֵ����ͨ�˲���		+ <---  ---> -
	
		bias_error_flag_pitch;	bias_detect_pitchֵ�쳣ָʾ		0������		1�����쳣�лָ�������ĵ�һ֡    2���쳣
		bias_pitch				ԭʼֵ						ǰ 
		bias_detect_pitch;		ԭʼֵ��ͳ���˲����			/\   +
		bias_real_pitch;		У��ֵ					    ||
		bias_lpf_pitch;			У��ֵ����ͨ�˲���			\/	 -
															��
	
		CH_ctrl[0]	������					- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
		CH_ctrl[1]	�������					ǰ -   �� +
	*/

	//***************************************************
	//pitch����
	
	float p_out_pitch,i_out_pitch,d_out_pitch,out_pitch;
	
	if(mode == 0)
	{
		//���㣨PID��
		
		if( bias_error_flag_pitch != 0 )
		{
			//ƫ�ƹ���ʹ��ƹ�ҿ���
			
			if( bias_detect_pitch < -30.0f )
			{
				//��ƫ����
				p_out_pitch = -100;		//���
			}
			else if( bias_detect_pitch > 30.0f )
			{
				//��ƫ����
				p_out_pitch =  100;		//�ҷ�
			}

			i_out_pitch = 0.0f;
			d_out_pitch = 0.0f;
		}
		else
		{
			//����ֵ
			
			//���ü���error����Ϊ����Ϊ0
			
			//p
			p_out_pitch = bias_lpf_pitch * pid_setup.groups.ctrl5.kp;
			
			//i
			position_integration_pitch += bias_lpf * pid_setup.groups.ctrl5.ki;
			position_integration_pitch = LIMIT(position_integration_pitch,-10.0f,10.0f);
			i_out_pitch = position_integration_pitch;
			
			//d
			d_out_pitch = speed_d_bias_lpf * pid_setup.groups.ctrl5.kd;		//speed_d_bias_lpf �����Ҹ�
			d_out_pitch = LIMIT(d_out_pitch,-10.0f,10.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		
		//�������
		//PID��� out�� - <-- --> +
		out_pitch = p_out_pitch + i_out_pitch + d_out_pitch;
		out_pitch = LIMIT(out_pitch,-15.0f,15.0f);
		
		//����� position_pitch_out
		//�����ֵӦ����-15��+15֮��
		position_pitch_out = -out_pitch;
	}
	else
	{
		//ǰ�������
		
		if(mode == 1)	//ǰ��
		{
			position_pitch_out = 100;
		}
		else
		{
			position_pitch_out = -100;
		}
	}
	
	
	
	//***************************************************
	//roll����
	
	float p_out_roll,i_out_roll,d_out_roll,out_roll;
	
	if( bias_error_flag != 0 )
	{
		//ƫ�ƹ���ʹ��ƹ�ҿ���
		
		if( bias_detect < -50.0f )
		{
			//��ƫ����
			p_out_roll = -5;		//���
		}
		else if( bias_detect > 50.0f )
		{
			//��ƫ����
			p_out_roll =  5;		//�ҷ�
		}

		i_out_roll = 0.0f;
		d_out_roll = 0.0f;
	}
	else
	{
		//����ֵ
		
		//���ü���error����Ϊ����Ϊ0
		
		//p
		p_out_roll = bias_lpf * user_parameter.groups.self_def_2.kp;
		
		//i
		position_integration_roll += bias_lpf * user_parameter.groups.self_def_2.ki;
		position_integration_roll = LIMIT(position_integration_roll,-10.0f,10.0f);
		i_out_roll = position_integration_roll;
		
		//d
		d_out_roll = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf �����Ҹ�
		d_out_roll = LIMIT(d_out_roll,-10.0f,10.0f);	//�����������Ϊ+-70������d����ɲ������
	}
	
	//�������
	//PID��� out�� - <-- --> +
	out_roll = p_out_roll + i_out_roll + d_out_roll;
	out_roll = LIMIT(out_roll,-15.0f,15.0f);

	
	//����� position_roll_out
	//�����ֵӦ����-15��+15֮��
	position_roll_out = -out_roll;

}

/**************************************************************************************

				�ٶȿ��ƻ�

	���룺	pitch_mode				0�����㣬����except_speed_pitch����		1��ǰ�����㶨10cm/s��	2�����ˣ��㶨-10cm/s��
			except_speed_pitch		pitch���������ٶȣ���λcm/s��ǰ����
			except_speed_roll		roll���������ٶȣ���λcm/s�������Ҹ�
			
	�����	CH_ctrl[0]				roll����Ƕ���������������
			CH_ctrl[1]				pitch����Ƕ�������ǰ��������
			CH_ctrl[3]				yaw������ٶ���������������


**************************************************************************************/
void speed_ctrl(u8 en)
{
	static float speed_error_integration_pitch = 0.0f;
	static float speed_error_integration_roll = 0.0f;
	
	//�Ǵ�ģʽʱ�����㴦��
	if(!en)
	{
		speed_error_integration_pitch = 0;
		speed_error_integration_roll = 0;
		
		return;
	}
	
	//����position_ctrl�����ݽӿ�
	u8 pitch_mode = position_mode_out;
	float except_speed_pitch = position_pitch_out;
	float except_speed_roll = position_roll_out;
	
	//*********************************************************
	
	//pitch����
	
	float speed_error_pitch = 0.0f;
	static float speed_error_old_pitch = 0.0f;

	float p_out_pitch = 0.0f, i_out_pitch = 0.0f, d_out_pitch = 0.0f, out_pitch = 0.0f;
	
	
	/*
								   ǰ               				ǰ
		speed_error_pitch��		   /\  +         CH_ctrl[PIT]��   	/\  -
								   ||								||
								   ||								||
								   \/  -							\/  +
								   ��								��
	*/
	
	if(pitch_mode == 0)
	{
		//��ͣ
		
		//�������루��λ��cm/s��
		//except_speed_pitch = 0.0f;	//-( my_deathzoom( ( CH_filter[RIT] ) , 0, 30 ) / 5.0f );
		except_speed_pitch = my_deathzoom( except_speed_pitch , 0, 1 );		//����+-1������
		except_speed_pitch = LIMIT(except_speed_pitch , -15, 15);			//�޷� -15 -- +15
		
	}
	else
	{
		//ǰ������ˣ����٣�
		//���ݿ���ʱ�����ٶȣ�������ʱƮ��ȥ
		
		if(pitch_mode == 1)	//ǰ��
		{
			except_speed_pitch = 10;	//��ǰ�������ٶ�Ϊ10cm/s
		}
		else	//pitch_mode = 2  ����
		{
			except_speed_pitch = -10;	//���������ٶ�Ϊ-10cm/s
		}
	}
	
	//����error
	//������ͷ������ʱspeed_d_bias_lpf_pitchΪ0
	speed_error_pitch = except_speed_pitch - speed_d_bias_lpf_pitch;	//����error   speed_errorֵ
																		//error   ����������ǰ�ٶ�С�ڵ�ǰ����ٶȣ�������ǰ�ٶȱȽ�С��Ӧ����ǰ����
																		//		  ����������ǰ�ٶȴ��ڵ�ǰ����ٶȣ�������ǰ�ٶȱȽϴ�Ӧ��������
	
	if(bias_error_flag_pitch != 0)
	{
		//�ٶȷ���ֵ������
		
		//ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ pid_setup.groups.ctrl6.kp��pid_setup.groups.ctrl6.ki�����������ٶȷ������ü��ٶȷ���
		
		//PID���Ϊ��������Ҫ��ǰ�м��ٶ�
		
		if(pitch_mode == 0)	//����
		{
			
			if( speed_error_pitch > 0.0f)	//��Ҫ��ǰ�ļ��ٶ�
			{
				p_out_pitch = 20.0f * pid_setup.groups.ctrl6.kp;	//ǰ��
				
			}
			else							//��Ҫ���ļ��ٶ�
			{
				p_out_pitch =  -20.0f * pid_setup.groups.ctrl6.ki;	//���
			}
			
		}
		else
		{
			//ǰ�������
			
			if(pitch_mode == 1)	//ǰ��
			{
				p_out_pitch = 2 * pid_setup.groups.ctrl6.kp;	//ǰƮ
			}
			else	//����
			{
				p_out_pitch = -2 * pid_setup.groups.ctrl6.ki;	//��Ʈ
			}

		}
		
		i_out_pitch = 0.0f;
		d_out_pitch = 0.0f;

		speed_error_old_pitch = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
	}
	else
	{
		//p
		p_out_pitch = speed_error_pitch * pid_setup.groups.ctrl4.kp;
		
		//i
		speed_error_integration_pitch += speed_error_pitch * pid_setup.groups.ctrl4.ki;
		speed_error_integration_pitch = LIMIT(speed_error_integration_pitch,-40.0f,40.0f);
		i_out_pitch = speed_error_integration_pitch;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������������ٶȲ����ˣ���Ҫ�������
		//					  ��������������ٶȲ��С�ˣ����ԷŻ��������/���Ҽ���
		d_out_pitch = (speed_error_pitch - speed_error_old_pitch) * pid_setup.groups.ctrl4.kd;
		d_out_pitch = LIMIT(d_out_pitch,-70.0f,70.0f);												//�����������Ϊ+-70������d����ɲ������
		
		speed_error_old_pitch = speed_error_pitch;
	}
	
	//�������
	out_pitch = p_out_pitch + i_out_pitch + d_out_pitch;
	out_pitch = LIMIT(out_pitch,-150.0f,150.0f);			//��λ��0.1��
	
	CH_ctrl[1] = -out_pitch;		//my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	
	//*********************************************************
	
	//roll����
	
	float speed_error_roll = 0.0f;
	static float speed_error_old_roll = 0.0f;
	float p_out_roll = 0.0f, i_out_roll = 0.0f, d_out_roll = 0.0f, out_roll = 0.0f;
	
	
	//�������루��λ��cm/s��
	//except_speed_roll = 0.0f;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	except_speed_roll = my_deathzoom( except_speed_roll , 0, 1 );	//����+-1������
	except_speed_roll = LIMIT(except_speed_roll , -15, 15);			//�޷� -15 -- +15
	
	//����error
	speed_error_roll = except_speed_roll - speed_d_bias_lpf;	//����error   speed_errorֵ
																//error   �������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
																//		  �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������
	
	//PID��������Ҹ� + <-- --> -
	
	if( bias_error_flag != 0 )
	{
		//�ٶȷ���ֵ������
		
		//ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B�����������ٶȷ������ü��ٶȷ���
		
		//PID���Ϊ��������Ҫ�����м��ٶ�
		
		if( speed_error_roll > 0.0f)	//��Ҫ����ļ��ٶ�
		{
			p_out_roll = 40.0f * user_parameter.groups.param_A;	//���
		}
		else							//��Ҫ���ҵļ��ٶ�
		{
			p_out_roll =  -40.0f * user_parameter.groups.param_B;	//�ҷ�
		}

		i_out_roll = 0.0f;
		d_out_roll = 0.0f;
		
		speed_error_old_roll = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		
	}
	else
	{
		//bias_detectֵ����
		
		//PID�����ֵΪ��������Ҫ�����м��ٶ�
		
		//p
		p_out_roll = speed_error_roll * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_error_integration_roll += speed_error_roll * user_parameter.groups.self_def_1.ki;
		speed_error_integration_roll = LIMIT(speed_error_integration_roll,-40.0f,40.0f);
		i_out_roll = speed_error_integration_roll;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������������ٶȲ����ˣ���Ҫ�������
		//					  ��������������ٶȲ��С�ˣ����ԷŻ��������/���Ҽ���
		d_out_roll = (speed_error_roll - speed_error_old_roll) * user_parameter.groups.self_def_1.kd;
		d_out_roll = LIMIT(d_out_roll,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		
		speed_error_old_roll = speed_error_roll;
	}

	//�������
	out_roll = p_out_roll + i_out_roll + d_out_roll;
	out_roll = LIMIT(out_roll,-150.0f,150.0f);			//��λ��0.1��

	CH_ctrl[0] = -out_roll;	//CH_ctrl   - <-- --> +
								//out_roll	+ <-- --> -
								//�ӿ���Ҫ�Ӹ���
	
	//*********************************************************
	
	//�����ͺ����ֶ�����
	CH_ctrl[3] = CH_filter[3];								//3������ YAW
	
}





//�ֶ�������̬
void attitude_hand(void)
{
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0����� ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	CH_ctrl[YAW] = CH_filter[YAW];	//3������ YAW
}


//********************************************************************************************************************
//													���Ժ���
//********************************************************************************************************************

//�����ƹ�ҿ���
void attitude_pingpong(void)
{
	/*
	
		������
		user_parameter.groups.self_def_1	�������PID		��Ӧ����վPID13
	
	*/
	
	//����Զ�����
	if(bias_real > 14)	//ƫ��
	{
		CH_ctrl[0] =  40 * user_parameter.groups.self_def_1.kp;	//�������kp	���ҵ�����Ϊ����
	}
	else if(bias_real < -14)
	{
		CH_ctrl[0] = -50 * user_parameter.groups.self_def_1.ki;	//���������Ϊ����
	}
	else
	{
		CH_ctrl[0] = 0;
	}
	
	//�����ͺ����ֶ�����
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	CH_ctrl[3] = CH_filter[3];								//3������ YAW
}

//��������̬����
void land_attitude(void)
{
	CH_ctrl[ROL] = 0;	//0����� ROL
	CH_ctrl[PIT] = 0;	//1������ PIT
	CH_ctrl[YAW] = 0;	//3������ YAW
}

//λ�ÿ��ƣ��Խӵ��ٶȿ��ƣ�
float position_except_speed = 0.0f;		//����ٶ���������λcm/s������ + <-- --> -
void position_to_speed_pid(u8 en)
{
	float p_out,i_out,d_out,out;
	static float position_roll_integration = 0;
	
	if(en)
	{
		
		/*
			bias		ԭʼֵ					+ <---  ---> -
			bias_detect ԭʼֵ��ͳ���˲����		+ <---  ---> -
			bias_real	У��ֵ					+ <---  ---> -
			bias_lpf	У��ֵ����ͨ�˲���		+ <---  ---> -
		
			CH_ctrl[0]	������					- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
		*/
		
		if( bias_error_flag != 0 )
		{
			//ƫ�ƹ���ʹ��ƹ�ҿ���
			
			if( bias_detect < -50.0f )
			{
				//��ƫ����
				p_out = -5;		//���
			}
			else if( bias_detect > 50.0f )
			{
				//��ƫ����
				p_out =  5;		//�ҷ�
			}

			i_out = 0.0f;
			d_out = 0.0f;
		}
		else
		{
			//����ֵ
			
			//���ü���error����Ϊ����Ϊ0
			
			//p
			p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
			
			//i
			position_roll_integration += bias_lpf * user_parameter.groups.self_def_2.ki;
			position_roll_integration = LIMIT(position_roll_integration,-10.0f,10.0f);
			i_out = position_roll_integration;
			
			//d
			d_out = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf �����Ҹ�
			d_out = LIMIT(d_out,-10.0f,10.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		
		//�������
		//PID��� out�� - <-- --> +
		out = p_out + i_out + d_out;
		out = LIMIT(out,-15.0f,15.0f);
		
		//����� position_except_speed
		//�����ֵӦ����-15��+15֮��
		position_except_speed = -out;

	}
	else
	{
		position_roll_integration = 0;
	}
}

//�ٶȿ���
//�ӿڣ�except_speed      + <-- --> -      ��λcm/s
void speed_pid(u8 en)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	static float roll_speed_integration = 0.0f;	//���ֱ���
	static float speed_error_old = 0.0f;	//old����
	s32 out_tmp;
	
	/*
		CH_filter[0]			ң�����������	- <---  ---> +
	
		speed_d_bias			�ٶ�ֵ			+ <---  ---> -
		speed_d_bias_lpf		lpfֵ			+ <---  ---> -
	
		CH_ctrl[0]	������						- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
	*/
	
	if(en)
	{
		//ģʽʹ��
		
		//except_speed      + <-- --> -      ��λcm/s
		
		except_speed = position_except_speed;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
		
		except_speed = my_deathzoom( except_speed , 0, 1 );	//����+-1������
		except_speed = LIMIT(except_speed,-15,15);			//�޷�
		
		if( bias_error_flag != 0 )
		{
			// bias_detect��ˮƽƫ�ֵ�쳣����
			
			//ƫ�ƹ���ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B
			
			if( bias_detect < -50.0f )
			{
				//��ƫ����
				p_out = -40.0f * user_parameter.groups.param_A;	//���
			}
			else if( bias_detect > 50.0f )
			{
				//��ƫ����
				p_out =  40.0f * user_parameter.groups.param_B;	//�ҷ�
			}

			i_out = 0.0f;
			d_out = 0.0f;
			
			speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
			
		}
		else
		{
			//bias_detectֵ����
			
			speed_error = except_speed - speed_d_bias_lpf;	//����error   speed_errorֵ
															//error   �������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
															//		  �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������
			
			//p
			p_out = - speed_error * user_parameter.groups.self_def_1.kp;
			
			//i
			roll_speed_integration += speed_error * user_parameter.groups.self_def_1.ki;
			roll_speed_integration = LIMIT(roll_speed_integration,-40.0f,40.0f);
			i_out = - roll_speed_integration;
			
			//d
			//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
			//error - error_old   ��������Ҫ�������
			//					  ����û��ô��Ҫ���������
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
			
			speed_error_old = speed_error;
		}
		
		//�������
		out = p_out + i_out + d_out;
		out = LIMIT(out,-150.0f,150.0f);
		
		//float������ȫ����
		out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
		out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
		out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

		CH_ctrl[0] = out;

		//�����ͺ����ֶ�����
		CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
		CH_ctrl[3] = CH_filter[3];								//3������ YAW
	}
	else
	{
		//�Ǵ�ģʽִ������
		roll_speed_integration = 0.0;
	}
}

//λ�ÿ���
void position_pid(u8 en)
{
	float p_out,i_out,d_out,out;
	static float roll_integration = 0;
	s32 out_tmp;
	
	if(en)
	{
		
		/*
			bias		ԭʼֵ					+ <---  ---> -
			bias_detect ԭʼֵ��ͳ���˲����		+ <---  ---> -
			bias_real	У��ֵ					+ <---  ---> -
			bias_lpf	У��ֵ����ͨ�˲���		+ <---  ---> -
		
			CH_ctrl[0]	������					- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
		*/
		
		if( bias_error_flag != 0 )
		{
			//ƫ�ƹ���ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B
			
			if( bias_detect < -50.0f )
			{
				//��ƫ����
				p_out = -40.0f * user_parameter.groups.param_A;	//���
			}
			else if( bias_detect > 50.0f )
			{
				//��ƫ����
				p_out =  40.0f * user_parameter.groups.param_B;	//�ҷ�
			}

			i_out = 0.0f;
			d_out = 0.0f;
			
		}
		else
		{
			//����ֵ
			
			//���ü���error����Ϊ����Ϊ0
			
			//p
			p_out = bias_lpf * user_parameter.groups.param_D;
			
			//i
			roll_integration += bias_lpf * user_parameter.groups.param_E;
			roll_integration = LIMIT(roll_integration,-40.0f,40.0f);
			i_out = roll_integration;
			
			//d
			d_out = speed_d_bias_lpf * user_parameter.groups.param_F;		//speed_d_bias_lpf �����Ҹ�
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		
		//�������
		out = p_out + i_out + d_out;
		out = LIMIT(out,-150.0f,150.0f);
		
		//float������ȫ����
		out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
		out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
		out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float
		
		//������
		CH_ctrl[0] = out;

		//�����ͺ����ֶ�����
		CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
		CH_ctrl[3] = CH_filter[3];								//3������ YAW
	}
	else
	{
		roll_integration = 0;
	}
}

void yaw_pid(void)
{
	float yaw_error,yaw_out;
	
	/*
		angle��			ƫ�� - ��ƫ�� +
		CH_ctrl[YAW]��	��ת - ����ת +
	*/
	
	//����ƫ��
	yaw_error = 0.0f - angle;	
	
	//�������
	yaw_out = yaw_error * user_parameter.groups.param_C;
	
	CH_ctrl[YAW] = yaw_out;		//3������ YAW
	
	//=======================================
	
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0����� ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	
}

//========================================================================================
//========================================================================================
//	�����Զ����ƺ���
//
//	����ctrl_command���ò�ͬ���Զ����ƺ���
//
//	mode_state��
//	0���ֶ�				1����ѹ��
//	2��������+��ѹ��		3���Զ�
//
//	ctrl_command��
//	0���������ֶ�����ģʽ��������+��ѹ�ƶ��ߣ�		1���߶�����
//	2���߶�����+��̬����							3������ģʽ												
//========================================================================================
//========================================================================================

void Fly_Ctrl(void)		//��������5ms
{
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}

//===========================================================
//==================== ���п����߼� ==========================
//===========================================================
	
	/*
	
	���п��Ƶ���Ҫ������������
	
	1.��̬���ƣ�����������������������
		CH_ctrl[0] = CH_filter[0];	//0�����
		CH_ctrl[1] = CH_filter[1];	//1������
		CH_ctrl[3] = CH_filter[3];	//3������
	
	2.�߶ȿ���
		my_height_mode = 0		��������ģʽ
		ʹ�� CH_filter[THR] ��������ֵ��ȡֵ��Χ -500 -- +500
	
		my_height_mode = 1		�����߶�ģʽ
		ʹ�� my_except_height �������Ƹ߶ȣ���λΪmm	
	
	*/
	
/* ********************* �߶ȿ��� ********************* */
	
	if(height_command == 0)
	{
		hand();
	}
	else
	{
		
	}
	
	if(height_command == 1)
	{
		height_lock(1);	//�����߶�
	}
	else
	{
		height_lock(0);	//�����־λ
	}
	
	if(height_command == 2)
	{
		land();	//����ģʽ
	}
	else
	{
		
	}
	
	//����״������
	if(height_command > 2)	//��Ӧ�ó��ֵ����
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2������ THR
	}
	
/* ********************* ��̬���� ********************* */
	
	if(ctrl_command == 0)
	{
		attitude_hand();
	}
	
	if(ctrl_command == 1)
	{
		attitude_hand();
	}
	
	if(ctrl_command == 2)
	{
		attitude_hand();
	}
	
//	if(ctrl_command == 3)
//	{
//		//attitude_single_p(1);
//	}
//	else
//	{
//		//attitude_single_p(0);
//	}
	
//	if(ctrl_command == 4)
//	{
//		//yaw_pid();
//	}

//	if(ctrl_command == 5)
//	{
//		attitude_hand();
//	}
	
	//����״������
	if(ctrl_command > 5)
	{
		attitude_hand();
	}
	
}

//CamƵ�ʵ��õķ��п��ƺ���
void Fly_Ctrl_Cam(void)		//����������camera������ͬ
{	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
/* ********************* ��̬���� ********************* */
	
	if(ctrl_command == 3)
	{
		position_pid(1);
	}
	else
	{
		position_pid(0);
	}
	
	if(ctrl_command == 4)
	{
		position_ctrl(1,0);
		speed_ctrl(1);
	}
	else
	{
		position_ctrl(0,0);
		speed_ctrl(0);
	}
	
	if(ctrl_command == 5)
	{
		if(sonar.displacement >= 250)
		{
			//��غ�ʹ��pid
			position_to_speed_pid(1);
			speed_pid(1);
		}
		else
		{
			//��������̬����
			land_attitude();
			
			//ͣ��λ�ÿ���pid
			position_to_speed_pid(0);
			speed_pid(0);
		}
	}
	else
	{
		position_to_speed_pid(0);
		speed_pid(0);
	}
	
	//����״������
	if(ctrl_command > 5)
	{
		attitude_hand();
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
	//����AUX2ͨ������6ͨ��������ֵ�����Զ�����ָ��
	if(*(ch_in+AUX2) < -350)			//-499 -- -350
	{
		ctrl_command = 0;
	}
	else if(*(ch_in+AUX2) < -150)		//-350 -- -150
	{
		ctrl_command = 1;
	}
	else if(*(ch_in+AUX2) < 0)			//-150 -- 0
	{
		ctrl_command = 2;
	}
	else if(*(ch_in+AUX2) < 150)		//0 -- 150
	{
		ctrl_command = 3;
	}
	else if(*(ch_in+AUX2) < 350)		//150 -- 350				
	{
		ctrl_command = 4;
	}
	else								//350 -- 499
	{
		ctrl_command = 5;
	}
	
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


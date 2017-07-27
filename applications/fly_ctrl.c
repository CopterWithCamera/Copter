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

//�ֶ�������̬
void attitude_hand(void)
{
	CH_ctrl[ROL] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0����� ROL
	CH_ctrl[PIT] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	CH_ctrl[YAW] = CH_filter[YAW];	//3������ YAW
}



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

//�ٶȿ���
void speed_pid(u8 en)
{
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	static float roll_speed_integration = 0.0f;	//���ֱ���
	static float speed_error_old = 0.0f;	//old����
	s32 out_tmp;
	
	/*
		speed_d_bias			�ٶ�ֵ			+ <---  ---> -
		speed_d_bias_lpf		lpfֵ			+ <---  ---> -
	
		CH_ctrl[0]	������						- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
	*/
	
	if(en)
	{
		//ģʽʹ��
		
		if( bias_error_flag != 0 )
		{
			//bias_detectֵ�쳣
			
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

			speed_error =  0 - speed_d_bias_lpf;	//����error   speed_errorֵ
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
			p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
			
			//i
			roll_integration += bias_lpf * user_parameter.groups.self_def_2.ki;
			roll_integration = LIMIT(roll_integration,-40.0f,40.0f);
			i_out = roll_integration;
			
			//d
			d_out = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf �����Ҹ�
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
		yaw_pid();
	}
	else
	{
		
	}
	
	if(ctrl_command == 5)
	{
		speed_pid(1);
	}
	else
	{
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


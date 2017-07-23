#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"
#include "camera_datatransfer.h"
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

void set_except_height(u8 height)	//����߶����ݣ���λcm��ȡֵ��Χ0cm-255cm
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

//�½���15cm
void falling_to_15cm(void)	//����߶��൱�ڽ����ˣ�����ܴ�Լ��15cm��һ��㣩
{
	Thr_Low = 1;	//���ŵͱ�־Ϊ1������������ͣת
	
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 150;		//�½���100mm = 10cm
}

//������50cm
void rising_to_50cm(void)
{
	Thr_Low = 0;	//���ŵͱ�־��0����ֹ����������ͣת
	
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 500;		//�½��� 150cm
}

//=====================================================================================================================================
//=====================================================================================================================================
//
//									                     ��̬��λ�ã����ƺ���
//
//					����ʹ�õĲ�����	float bias��float bias_real��float bias_lpf	�����Ҹ�����ֵΪ����ƫ����ֵΪ����ƫ�ң�
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

//����ƫ�������е�PID����
float roll_integration = 0;
void attitude_single_p(u8 en)
{
	float p_out,i_out,d_out,out;
	static float bias_old;
	
	if(en)
	{
		
		/*
		
			bias		ԭʼֵ					+ <---  ---> -
			bias_real	У��ֵ					+ <---  ---> -
			bias_lpf	У��ֵ����ͨ�˲���		+ <---  ---> -
		
			CH_ctrl[0]	������					- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
		
		*/
		
		if( ABS(bias) > 50 )
		{
			//ƫ�ƹ���
			
			if( bias > 50 )
			{
				//��ƫ����
				p_out = 40 * user_parameter.groups.self_def_1.kp;		//�ҷ�
			}
			else if( bias < -50 )
			{
				//��ƫ����
				p_out = -40 * user_parameter.groups.self_def_1.ki;	//���
			}
			i_out = 0;
			
		}
		else
		{
			//����ֵ
			
			//p
			p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
			
			//i
			roll_integration += bias_lpf * user_parameter.groups.self_def_2.ki;
			roll_integration = LIMIT(roll_integration,-40,40);
			i_out = roll_integration;
		}
		
		//d
		d_out = ( bias_lpf - bias_old ) * user_parameter.groups.self_def_2.kd;		//bias_lpf�����Ҹ�
																					//bias_lpf - bias_old	Ϊ����Ӧ����ɣ�Ϊ����Ӧ���ҷ�
																					//d���Ƶ�Ŀ���Ƕ��ٶȲ������ƣ�ֻҪ��֤���ķ������ٶȷ����෴����
																					//����ֻ�����ٶȷ����ṩ���ٶȼ���
		d_out = LIMIT(d_out,-70,70);	//�����������Ϊ+-70������d����ɲ������
		bias_old = bias_lpf;
		
		//�������
		out = p_out + i_out + d_out;
		out = LIMIT(out,-150,150);
		
		CH_ctrl[0] = out;

		//�����ͺ����ֶ�����
		CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
		CH_ctrl[3] = CH_filter[3];								//3������ YAW
		
		mydata.d9 = (s16)p_out;
		mydata.d10 = (s16)i_out;
		mydata.d11 = (s16)out;
	}
	else
	{
		roll_integration = 0;
	}
}

void yaw_pid(void)
{
	float yaw_error,yaw_out;
	
	//����ƫ��
	yaw_error = angle - 0.0f;	//ƫ�� +��ƫ�� -
	
	//�������
	yaw_out = user_parameter.groups.param_A *yaw_error;
	
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
		falling_to_15cm();
	}
	
	//����״������
	if(height_command > 2)	//��Ӧ�ó��ֵ����
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2������ THR
	}
	
/* ********************* ��̬���� ********************* */
	
	//ָ��1
	if(ctrl_command == 0)
	{
		attitude_hand();
	}
	
	//ָ��2
	if(ctrl_command == 1)
	{
		attitude_pingpong();	//�����ƹ�ҿ���
	}
	
	//ָ��3
	if(ctrl_command == 2)
	{
		attitude_hand();
	}
	
	//ָ��4
	if(ctrl_command == 3)
	{
		attitude_single_p(1);
	}
	else
	{
		attitude_single_p(0);
	}
	
	//ָ��5
	if(ctrl_command == 4)
	{
		yaw_pid();
	}
	
	//ָ��6
	if(ctrl_command == 5)
	{
		attitude_hand();
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


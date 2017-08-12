#include "track_function.h"

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

//ǰ��ʱ�õ�pitch����
void forward_pitch(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old����
	static u8 d_stop_flag = 0;		//ֹͣd����ı�־λ����ʾspeed_error_old��ֵ��Ч
	s32 out_tmp;
	
	static u8 lost_circle_flag = 0;	//������ֳ���״̬�������Ѿ���ʧԲ��׼���������ģʽ
	
	/*
		speed_d_bias_pitch			�ٶ�ֵ			+ <ǰ---  ---��> -
		speed_d_bias_lpf_pitch		lpfֵ			+ <ǰ---  ---��> -
	
		CH_filter[1]				ң������������	- <ǰ---  ---��> +
	*/

	//ģʽʹ��
	
	//except_speed_pitch      + <-- --> -      ��λcm/s
	
	except_speed = 10;	//��һ���ȽϺ��ʵ�ǰ�����ٶ�
	except_speed = LIMIT(except_speed,-15,15);			//�޷����ٶȵ���Ҫ��ƽ�ȣ�
	
	if( bias_error_flag != 0 )
	{
		// bias_detect��ˮƽƫ�ֵ�쳣����
		
		//��ǰƮ��
		
		p_out = 0.0f;	//��ǰ�ܵ͵����
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		d_stop_flag = 1;	//��ʾspeed_error_old��Ч���޷�����d����
		
		lost_circle_flag = 1;	//����ǰƮģʽ����ǰ�󳬳�����Ϊ���Ѿ���ʼƥ��С��
	}
	else
	{
		if(lost_circle_flag)
		{
			//�ڶ�ʧԲ���ֿ�������
			ctrl_command = 5;		//�������ģʽ
		}
		
		//bias_detectֵ����
		
		speed_error = except_speed - speed_d_bias_lpf_pitch;	//����error   speed_errorֵ
																//error   �������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
																//		  �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������
		
		//p
		p_out = - speed_error * pid_setup.groups.ctrl4.kp; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * pid_setup.groups.ctrl4.ki; //user_parameter.groups.self_def_1.ki;
		speed_integration_pitch = LIMIT(speed_integration_pitch,-40.0f,40.0f);
		
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_pitch = 0;
//		}
		
		i_out = - speed_integration_pitch;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������Ҫ�������
		//					  ����û��ô��Ҫ���������
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * pid_setup.groups.ctrl4.kd; //user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//�������
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float������ȫ����
	out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
	out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
	out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

	CH_ctrl[1] = out;	//���ݾ���ֵ��CH_ctrl������ֵӦ����50-100֮��
}

//ǰ��ʱ�õ�roll����
void forward_roll(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old����
	static u8 d_stop_flag = 0;		//ֹͣd����ı�־λ����ʾspeed_error_old��ֵ��Ч
	s32 out_tmp;
	
	/*
		CH_filter[0]			ң�����������	- <---  ---> +
	
		speed_d_bias			�ٶ�ֵ			+ <---  ---> -
		speed_d_bias_lpf		lpfֵ			+ <---  ---> -
	
		CH_ctrl[0]	������						- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
	*/

	//ģʽʹ��
	
	//except_speed      + <-- --> -      ��λcm/s
	
	except_speed = position_roll_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	
	except_speed = LIMIT(except_speed,-15,15);			//�޷����ٶȵ���Ҫ��ƽ�ȣ�
	
	if( bias_error_flag != 0 )
	{
		// bias_detect��ˮƽƫ�ֵ�쳣����
		
		// ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B

		// ��ʱ�Ѿ���ʧ��Ұ������roll����
		
		p_out = 0.0f;
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		d_stop_flag = 1;	//��ʾspeed_error_old��Ч���޷�����d����
	}
	else
	{
		//bias_detectֵ����
		
		speed_error = except_speed - speed_d_bias_lpf;	//���������ٶȲ�   speed_errorֵ   - <-- --> +
														//error   �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������		�������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
		
		//p
		p_out = - speed_error * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.self_def_1.ki;
		speed_integration_roll = LIMIT(speed_integration_roll,-40.0f,40.0f);
			
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_roll = 0;
//		}
		
		i_out = - speed_integration_roll;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������Ҫ�������
		//					  ����û��ô��Ҫ���������
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//�������
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float������ȫ����
	out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
	out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
	out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

	CH_ctrl[0] = out;	//���ݾ���ֵ��CH_ctrl������ֵӦ����50-100֮��
}


//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

//����ʱ�õ�pitch����
void backward_pitch(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old����
	static u8 d_stop_flag = 0;		//ֹͣd����ı�־λ����ʾspeed_error_old��ֵ��Ч
	s32 out_tmp;
	
	/*
		speed_d_bias_pitch			�ٶ�ֵ			+ <ǰ---  ---��> -
		speed_d_bias_lpf_pitch		lpfֵ			+ <ǰ---  ---��> -
	
		CH_filter[1]				ң������������	- <ǰ---  ---��> +
	*/

	//ģʽʹ��
	
	//except_speed_pitch      + <-- --> -      ��λcm/s
	
	except_speed = -5;	//��һ���ȽϺ��ʵ�ǰ�����ٶ�
	except_speed = LIMIT(except_speed,-15,15);			//�޷����ٶȵ���Ҫ��ƽ�ȣ�
	
	if( bias_error_flag != 0 )
	{
		// bias_detect��ˮƽƫ�ֵ�쳣����
		
		//��ǰƮ��
		
		p_out = 0.0f;	//���к���
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		d_stop_flag = 1;	//��ʾspeed_error_old��Ч���޷�����d����

	}
	else
	{
		//bias_detectֵ����
		
		speed_error = except_speed - speed_d_bias_lpf_pitch;	//����error   speed_errorֵ
																//error   �������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
																//		  �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������
		
		//p
		p_out = - speed_error * pid_setup.groups.ctrl4.kp; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * pid_setup.groups.ctrl4.ki; //user_parameter.groups.self_def_1.ki;
		speed_integration_pitch = LIMIT(speed_integration_pitch,-40.0f,40.0f);
		
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_pitch = 0;
//		}
		
		i_out = - speed_integration_pitch;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������Ҫ�������
		//					  ����û��ô��Ҫ���������
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * pid_setup.groups.ctrl4.kd; //user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//�������
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float������ȫ����
	out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
	out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
	out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

	CH_ctrl[1] = out;	//���ݾ���ֵ��CH_ctrl������ֵӦ����50-100֮��
}

//ǰ��ʱ�õ�roll����
void backward_roll(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old����
	static u8 d_stop_flag = 0;		//ֹͣd����ı�־λ����ʾspeed_error_old��ֵ��Ч
	s32 out_tmp;
	
	/*
		CH_filter[0]			ң�����������	- <---  ---> +
	
		speed_d_bias			�ٶ�ֵ			+ <---  ---> -
		speed_d_bias_lpf		lpfֵ			+ <---  ---> -
	
		CH_ctrl[0]	������						- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
	*/

	//ģʽʹ��
	
	//except_speed      + <-- --> -      ��λcm/s
	
	except_speed = position_roll_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	
	except_speed = LIMIT(except_speed,-15,15);			//�޷����ٶȵ���Ҫ��ƽ�ȣ�
	
	if( bias_error_flag != 0 )
	{
		// bias_detect��ˮƽƫ�ֵ�쳣����
		
		// ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B

		// ��ʱ�Ѿ���ʧ��Ұ������roll����
		
		p_out = 0.0f;
		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		d_stop_flag = 1;	//��ʾspeed_error_old��Ч���޷�����d����
	}
	else
	{
		//bias_detectֵ����
		
		speed_error = except_speed - speed_d_bias_lpf;	//���������ٶȲ�   speed_errorֵ   - <-- --> +
														//error   �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������		�������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
		
		//p
		p_out = - speed_error * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.self_def_1.ki;
		speed_integration_roll = LIMIT(speed_integration_roll,-40.0f,40.0f);
			
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_roll = 0;
//		}
		
		i_out = - speed_integration_roll;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������Ҫ�������
		//					  ����û��ô��Ҫ���������
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//�������
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float������ȫ����
	out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
	out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
	out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

	CH_ctrl[0] = out;	//���ݾ���ֵ��CH_ctrl������ֵӦ����50-100֮��
}


//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************


//Pitchλ�ÿ���
static float position_integration_pitch = 0;
void position_track_pitch(float T)	//������ͷ�ɼ�����ͬƵ����
{
	float p_out,i_out,d_out,out;
	static u16 bias_error_counter_pitch = 0;

	/*
		bias_pitch				ԭʼֵ					+ <ǰ---  ---��> -
		bias_detect_pitch 		ԭʼֵ��ͳ���˲����		+ <ǰ---  ---��> -
		bias_error_flag_pitch	ƫ��ֵ��Чָʾ			0��ƫ��ֵ����	1��ƫ��ֵ�쳣
		bias_real_pitch			У��ֵ					+ <ǰ---  ---��> -
		bias_lpf_pitch			У��ֵ����ͨ�˲���		+ <ǰ---  ---��> -
	
		CH_ctrl[1]				�������					- <---  ---> +		ǰ��������������ǰ�м��ٶȣ���������м��ٶȣ�
	*/
	
	if( bias_error_flag_pitch != 0 )
	{
		//ƫ�ƹ���ʹ��ƹ�ҿ���
		
		if(	( bias_error_counter_pitch * T ) > 20.0f )	//ʱ�䳬��5s
		{
			p_out = 0.0f;	//���ٿ���
			i_out = 0.0f;
			d_out = 0.0f;
		}
		else
		{
			bias_error_counter_pitch++;	//������¼��������ʱ��
			
			if( bias_detect_pitch < -30.0f )
			{
				//��ƫ����
				p_out = -5;		//ǰ��
			}
			else if( bias_detect_pitch > 30.0f )
			{
				//ǰƫ����
				p_out =  5;		//���
			}
			else
			{
				p_out = 0.0f;
			}
			
//			p_out = bias_lpf_pitch / 2.0f * pid_setup.groups.ctrl5.kp;

			i_out = 0.0f;
			d_out = 0.0f;
		}
		
		position_integration_pitch = 0.0f;		//��������
		
	}
	else
	{
		//����ֵ
		
		bias_error_counter_pitch = 0;
		
		//���ü���error����Ϊ����Ϊ0
		
		//p
		p_out = bias_lpf_pitch * pid_setup.groups.ctrl5.kp; //user_parameter.groups.self_def_2.kp;
		
		//i
		position_integration_pitch += bias_lpf_pitch * pid_setup.groups.ctrl5.ki; //user_parameter.groups.self_def_2.ki;
		position_integration_pitch = LIMIT(position_integration_pitch,-10.0f,10.0f);
		
		//�ӽ�����λ��I����
		if(ABS(bias_lpf_pitch) < 10)
		{
			position_integration_pitch = 0.0f;
		}
		
		i_out = position_integration_pitch;
		
		//d
		d_out = speed_d_bias_lpf_pitch * pid_setup.groups.ctrl5.kd; //user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf �����Ҹ�
		d_out = LIMIT(d_out,-10.0f,10.0f);	//�����������Ϊ+-70������d����ɲ������
	}
	
	//�������
	//PID��� out�� - <ǰ-- --��> +
	out = p_out + i_out + d_out;
	//out = LIMIT(out,-15.0f,15.0f);
	
	position_pitch_out = -out;	//	+ <ǰ-- --��> -
}

//Rollλ�ÿ���
static float position_integration_roll = 0;
void position_track_roll(float T)
{
	float p_out,i_out,d_out,out;
	
	static u16 bias_error_counter;

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
		
		if( bias_error_counter * T >20.0f )	//��ʱ���Ƴ���ʱ�����5s
		{
			p_out = 0.0f;	//���ٿ���
			i_out = 0.0f;
			d_out = 0.0f;
		}
		else
		{
			bias_error_counter++;
			
			if( bias_detect < -50.0f )	//����ƫ�Ʒ������ָ��
			{
				//��ƫ����
				p_out = -5;		//���
			}
			else if( bias_detect > 50.0f )
			{
				//��ƫ����
				p_out =  5;		//�ҷ�
			}
			else
			{
				//�ݴ�
				p_out = 0.0f;
			}
			
//			p_out = bias_lpf / 2.0f * user_parameter.groups.self_def_2.kp;

			i_out = 0.0f;
			d_out = 0.0f;
		}
		
		position_integration_roll = 0.0f;	//��������
	}
	else
	{
		//����ֵ
		
		bias_error_counter = 0;
		
		//���ü���error����Ϊ����Ϊ0
		
		//p
		p_out = bias_lpf * user_parameter.groups.self_def_2.kp;
		
		//i
		position_integration_roll += bias_lpf * user_parameter.groups.self_def_2.ki;
		position_integration_roll = LIMIT(position_integration_roll,-10.0f,10.0f);
		
		//�ӽ�����λ��I����
		if(ABS(bias_lpf) < 10)
		{
			position_integration_roll = 0.0f;
		}
		
		i_out = position_integration_roll;
		
		//d
		d_out = speed_d_bias_lpf * user_parameter.groups.self_def_2.kd;		//speed_d_bias_lpf �����Ҹ�
		d_out = LIMIT(d_out,-10.0f,10.0f);	//�����������Ϊ+-70������d����ɲ������
	}
	
	//�������
	//PID��� out�� - <-- --> +
	out = p_out + i_out + d_out;
	//out = LIMIT(out,-15.0f,15.0f);
	
	position_roll_out = -out;	//	+ <---  ---> -

}

//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************


//׷�����ٶȿ���
//�ӿڣ�except_speed      + <-- --> -      ��λcm/s
void speed_track_pitch(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old����
	static u8 d_stop_flag = 0;		//ֹͣd����ı�־λ����ʾspeed_error_old��ֵ��Ч
	s32 out_tmp;
	
	/*
		speed_d_bias_pitch			�ٶ�ֵ			+ <ǰ---  ---��> -
		speed_d_bias_lpf_pitch		lpfֵ			+ <ǰ---  ---��> -
	
		CH_filter[1]				ң������������	- <ǰ---  ---��> +
	*/

	//ģʽʹ��
	
	//except_speed_pitch      + <-- --> -      ��λcm/s
	
	except_speed = position_pitch_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	except_speed = LIMIT(except_speed,-15,15);			//�޷����ٶȵ���Ҫ��ƽ�ȣ�
	
	if( bias_error_flag != 0 )
	{
		// bias_detect��ˮƽƫ�ֵ�쳣����
		
		//ƫ�ƹ���ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B
		
		if( except_speed > 1.0f )	//�ٶ�������ǰ
		{
			p_out = -20.0f * pid_setup.groups.ctrl6.kp; //user_parameter.groups.param_A;	//ǰ��
		}
		else if(except_speed < -1.0f)	//�ٶ��������
		{
			p_out =  20.0f * pid_setup.groups.ctrl6.ki; //user_parameter.groups.param_B;	//���
		}
		else
		{
			p_out = 0.0f;	//�м���������
		}

		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		d_stop_flag = 1;	//��ʾspeed_error_old��Ч���޷�����d����
	}
	else
	{
		//bias_detectֵ����
		
		speed_error = except_speed - speed_d_bias_lpf_pitch;	//����error   speed_errorֵ
																//error   �������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
																//		  �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������
		
		//p
		p_out = - speed_error * pid_setup.groups.ctrl4.kp; //user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_pitch += speed_error * pid_setup.groups.ctrl4.ki; //user_parameter.groups.self_def_1.ki;
		speed_integration_pitch = LIMIT(speed_integration_pitch,-40.0f,40.0f);
		
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_pitch = 0;
//		}
		
		i_out = - speed_integration_pitch;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������Ҫ�������
		//					  ����û��ô��Ҫ���������
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * pid_setup.groups.ctrl4.kd; //user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//�������
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float������ȫ����
	out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
	out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
	out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

	CH_ctrl[1] = out;	//���ݾ���ֵ��CH_ctrl������ֵӦ����50-100֮��
}

//׷���ٶȿ���
//�ӿڣ�except_speed      + <-- --> -      ��λcm/s
void speed_track_roll(void)
{
	float except_speed = 0.0f;
	float p_out,i_out,d_out,out;
	float speed_error = 0.0f;
	
	static float speed_error_old = 0.0f;	//old����
	static u8 d_stop_flag = 0;		//ֹͣd����ı�־λ����ʾspeed_error_old��ֵ��Ч
	s32 out_tmp;
	
	/*
		CH_filter[0]			ң�����������	- <---  ---> +
	
		speed_d_bias			�ٶ�ֵ			+ <---  ---> -
		speed_d_bias_lpf		lpfֵ			+ <---  ---> -
	
		CH_ctrl[0]	������						- <---  ---> +		�����������������м��ٶȣ����������м��ٶȣ�
	*/

	//ģʽʹ��
	
	//except_speed      + <-- --> -      ��λcm/s
	
	except_speed = position_roll_out;	//-( my_deathzoom( ( CH_filter[ROL] ) , 0, 30 ) / 5.0f );
	
	except_speed = LIMIT(except_speed,-15,15);			//�޷����ٶȵ���Ҫ��ƽ�ȣ�
	
	if( bias_error_flag != 0 )
	{
		// bias_detect��ˮƽƫ�ֵ�쳣����
		
		// ���������ٶȽ��д����������ٶȲ��Ϊ�ٶȷ�����Ч��
		
		//ʹ��ƹ�ҿ��ƣ�ϵ����Ӧ param_A param_B
		
		if( except_speed > 1.0f )	//�ٶ���������
		{
			p_out = -30.0f * user_parameter.groups.param_A;	//���
		}
		else if(except_speed < -1.0f)	//�ٶ���������
		{
			p_out =  30.0f * user_parameter.groups.param_B;	//�ҷ�
		}
		else						
		{
			p_out = 0.0f;	//�м���������
		}

		i_out = 0.0f;
		d_out = 0.0f;
		
		speed_error_old = 0;	// speed_error_old ���㣨��һ���̶��ϼ�С��d��Ӱ�죩
		d_stop_flag = 1;	//��ʾspeed_error_old��Ч���޷�����d����
	}
	else
	{
		//bias_detectֵ����
		
		speed_error = except_speed - speed_d_bias_lpf;	//���������ٶȲ�   speed_errorֵ   - <-- --> +
														//error   �������������ٶȴ��ڵ�ǰ�����ٶȣ����������ٶȱȽϴ�Ӧ���������		�������������ٶ�С�ڵ�ǰ�����ٶȣ����������ٶȱȽ�С��Ӧ�����Ҽ���
		
		//p
		p_out = - speed_error * user_parameter.groups.self_def_1.kp;
		
		//i
		speed_integration_roll += speed_error * user_parameter.groups.self_def_1.ki;
		speed_integration_roll = LIMIT(speed_integration_roll,-40.0f,40.0f);
			
//		if(ABS(speed_error) < 4)
//		{
//			speed_integration_roll = 0;
//		}
		
		i_out = - speed_integration_roll;
		
		//d
		//error    +   ��Ӧ��������٣�<-- --> ��Ӧ�����Ҽ��٣�   -
		//error - error_old   ��������Ҫ�������
		//					  ����û��ô��Ҫ���������
		if(d_stop_flag)
		{
			d_out = -(speed_error - speed_error_old) * user_parameter.groups.self_def_1.kd;
			d_out = LIMIT(d_out,-70.0f,70.0f);	//�����������Ϊ+-70������d����ɲ������
		}
		else
		{
			d_out = 0.0f;
		}
		
		speed_error_old = speed_error;
		d_stop_flag = 0;
	}
	
	//�������
	out = p_out + i_out + d_out;
	out = LIMIT(out,-150.0f,150.0f);
	
	//float������ȫ����
	out_tmp = (s32)(out*100.0f);	//�Ŵ�100��������С�����2λ����
	out_tmp = LIMIT(out_tmp,-15000,15000);	//�޷�
	out = ((float)out_tmp) / 100.0f;	//��С100�����ع�float

	CH_ctrl[0] = out;	//���ݾ���ֵ��CH_ctrl������ֵӦ����50-100֮��
}



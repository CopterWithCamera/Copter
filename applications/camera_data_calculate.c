#include "camera_data_calculate.h"
#include "camera_datatransfer.h"
#include "mymath.h"
#include "math.h"
#include "stdio.h"

//roll����
float bias_detect = 0.0;
u8 bias_error_flag = 0;		//bias_detectֵ�쳣ָʾ		0������    1�����쳣�лָ�������ĵ�һ֡    2���쳣
float bias_real = 0;
float bias_lpf = 0;
float speed_d_bias = 0;
float speed_d_bias_lpf = 0;

//pitch����
float bias_detect_pitch = 0.0;
u8 bias_error_flag_pitch = 0;		//bias_detectֵ�쳣ָʾ		0������    1�����쳣�лָ�������ĵ�һ֡    2���쳣
float bias_real_pitch = 0;
float bias_lpf_pitch = 0;
float speed_d_bias_pitch = 0;
float speed_d_bias_lpf_pitch = 0;

//�������
float receive_fps = 0;

//��ͨ�˲���
//dt������ʱ��������λus��
//fc����ֹƵ��
float cam_bias_lpf(float bias ,float dt_us ,float fc,float last_bias_lpf)
{
	float q,out,T;
	
	T = dt_us / 1000000.0f;
	q = 6.28f * T * fc;
	
	if(q >0.95f)
		q = 0.95f;
	
	out = q * bias + (1.0f-q) * last_bias_lpf;
	
	return out;
}

/***************************

	roll		������
	pitch		ǰ������
	
	bias		�����Ҹ�
	bias_pitch	ǰ����

****************************/

/*
//bias����У׼����
float bias_correct(float roll, float hight,float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1,real_bias;
	x1=hight*sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}
*/

//roll���򲹳�
float bias_correct(float roll, float pitch, float hight, float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1, real_bias, coe_bias;  //coe_bias Ϊ��õ�bias ��ʵ��bias��ϵ��
	
	//��������
	x1 = hight * sin(roll*3.141f/180.0f);
	
	//ϵ��
	coe_bias = hight / 2.0f / 40.0f;
	
	real_bias = coe_bias * bias - x1;
	
	return real_bias;
}

//pitch���򲹳�
float bias_pitch_correct(float roll, float pitch, float hight, float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1, real_bias, coe_bias;  //coe_bias Ϊ��õ�bias ��ʵ��bias��ϵ��
	
	//��������
	x1 = hight * sin(pitch*3.141f/180.0f);
	
	//ϵ��
	coe_bias = hight / 2.0f / 40.0f;
	
	real_bias = coe_bias * bias - x1;
	
	return real_bias;
}

/*
	T��ʱ��������λus��
	bias������ˮƽƫ��������λcm��
	bias_old���ϴ�ˮƽƫ��������λcm��
*/
float get_speed(u32 T,float bias,float bias_last,float speed_last)
{
	/*
		bias��ֵ�� 	+ <--- ---> -
		speed����	+ <--- ---> -	������ٶ�Ϊ�������ҷ��ٶ�Ϊ��
									
									��ǰ���ٶ�Ϊ���������ٶ�Ϊ��
	*/
	
	float dx,dt,speed;
	dx = bias - bias_last;
	dt = T / 1000000.0f;
	speed = safe_div(dx,dt,0);
	
	//��ֵ����������
	if( ABS(speed) > 25)
		speed = speed_last;
	
	return speed;
}

//��ʱ��ȡ֡�ʣ�Ҫ�����Ƶ��Ϊ2sһ�Σ�����ѭ���е���
u16 receive_fps_counter = 0;	//ÿ�ζ�ȡ����ʱ+1
void get_fps(void)
{
	//�������֡��
	receive_fps = receive_fps_counter / 2.0f;	//ת��Ϊ��HzΪ��λ
	receive_fps_counter = 0;
}

//Camera���ݴ������ݱ�־λ����ѭ���е��ã�
void Camera_Calculate(void)
{
	
	//*****************************************8
	//roll������

	//��ʹ����֡���ϣ�ֱ�����βɼ�ֵ
	bias_detect = bias;
	
	//����У׼���˲������� bias_error_flag��bias_real��bias_lpf��speed_d_bias��speed_d_bias_lpf ��ֵ��
	static float bias_lpf_old;	//��һ���ڿ��÷�Χ�ڵ�bias_lpf
	if(ABS(bias_detect)<38.0f)	//ֻ���ں���Χ�ڲŻ������������ͬʱ���е�ͨ�˲�
	{
		//�������
		
		//ƫ������
		bias_real = bias_correct(Roll_Image,Pitch_Image, Height_Image/10.0f, bias_detect);	//��̬���У׼
		bias_lpf = cam_bias_lpf(bias_real,receive_T,0.8f,bias_lpf);		//��ͨ�˲���
		
		//�ٶ����㣨ʹ�� bias_lpf Ϊ����Դ��
		if(bias_error_flag == 2)	//��һ֡Ҫ����bias_lpf_old����
		{
			speed_d_bias = 0;		//�޷�����speed_d_bias�����Թ���
			speed_d_bias_lpf = 0;	//�޷�����speed_d_bias_lpf������speed_d_bias_lpfҪ����d���㣬���Թ�������Ӱ��
			
			bias_error_flag--;	//��������������� bias_error_flag ����1�ˣ��������״̬�൱���Ǵ��쳣�ָ���ĵ�һ֡����ƫ��û�ٶ�
		}
		else if(bias_error_flag == 1)
		{
			speed_d_bias = get_speed(receive_T,bias_lpf,bias_lpf_old,speed_d_bias_lpf);
			speed_d_bias_lpf = speed_d_bias;	//����֮ǰ����ֵ��0������ֱ�Ӳ��ɱ�֡���
			
			bias_error_flag--;	//��������������� bias_error_flag ����0�ˣ��ٶȺ��ٶ�lpf���ָ���
		}
		else
		{
			speed_d_bias = get_speed(receive_T,bias_lpf,bias_lpf_old,speed_d_bias_lpf);
			speed_d_bias_lpf = cam_bias_lpf(speed_d_bias,receive_T,1.0f,speed_d_bias_lpf);
		}
		
		bias_lpf_old = bias_lpf;	//������һ֡��bias_lpf��ֻ��bias_detect��Чʱ�Ż���� bias_lpf_old ����ֹ+-100���쳣ֵ�������㣩
	}
	else
	{
		bias_error_flag = 2;	//��ʾbias_detect���ݴ����쳣ֵ��΢������õ��ٶ���Ҫ��֡��
		
		speed_d_bias = 0;
		speed_d_bias_lpf = 0;
	}
	
	//*****************************************
	//pitch������	
	
	//ֱ�����δ�����ֵ
	bias_detect_pitch = bias_pitch;
	
	//����У׼���˲�
	static float bias_lpf_old_pitch;	//��һ���ڿ��÷�Χ�ڵ�bias_lpf
	if(ABS(bias_detect_pitch)<22.0f)	//����������bias_detect_pitch������ֵֻ��-24~+24��
	{
		//ƫ������
		bias_real_pitch = bias_pitch_correct(Roll_Image,Pitch_Image, Height_Image/10.0f, bias_detect_pitch);	//��̬���У׼
		bias_lpf_pitch = cam_bias_lpf(bias_real_pitch,receive_T,0.8f,bias_lpf_pitch);		//��ͨ�˲���
		
		//�ٶ����㣨ʹ�� bias_lpf_pitch Ϊ����Դ��
		if(bias_error_flag_pitch == 2)	//��һ֡Ҫ����bias_lpf_old����
		{
			speed_d_bias_pitch = 0;		//�޷�����speed_d_bias_pitch�����Թ���
			speed_d_bias_lpf_pitch = 0;	//�޷�����speed_d_bias_lpf_pitch������speed_d_bias_lpf_pitchҪ����d���㣬���Թ�������Ӱ��
			
			bias_error_flag_pitch--;	//��������������� bias_error_flag ����1�ˣ��������״̬�൱���Ǵ��쳣�ָ���ĵ�һ֡����ƫ��û�ٶ�
		}
		else if(bias_error_flag_pitch == 1)
		{
			speed_d_bias_pitch = get_speed(receive_T,bias_lpf_pitch,bias_lpf_old_pitch,speed_d_bias_lpf_pitch);
			speed_d_bias_lpf_pitch = speed_d_bias_pitch;	//����֮ǰ����ֵ��0������ֱ�Ӳ��ɱ�֡���
			
			bias_error_flag_pitch--;	//��������������� bias_error_flag ����0�ˣ��ٶȺ��ٶ�lpf���ָ���
		}
		else
		{
			speed_d_bias_pitch = get_speed(receive_T,bias_lpf_pitch,bias_lpf_old_pitch,speed_d_bias_lpf_pitch);
			speed_d_bias_lpf_pitch = cam_bias_lpf(speed_d_bias_pitch,receive_T,1.0f,speed_d_bias_lpf_pitch);
		}
		
		bias_lpf_old_pitch = bias_lpf_pitch;	//������һ֡��bias_lpf_pitch��ֻ��bias_detect_pitch��Чʱ�Ż���� bias_lpf_old_pitch ����ֹ+-100���쳣ֵ�������㣩
		
	}
	else	//�����쳣
	{
		bias_error_flag_pitch = 2;	//ƫ���쳣
		
		speed_d_bias_pitch = 0;
		speed_d_bias_lpf_pitch = 0;
	}
	
}

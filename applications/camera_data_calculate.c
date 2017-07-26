#include "camera_data_calculate.h"
#include "camera_datatransfer.h"
#include "mymath.h"
#include "math.h"
#include "stdio.h"

float bias_lpf = 0;
float bias_real = 0;
float speed_d_bias = 0;
float speed_d_bias_lpf = 0;
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

float bias_correct(float roll, float pitch, float hight, float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1, real_bias, coe_bias;  //coe_bias Ϊ��õ�bias ��ʵ��bias��ϵ��
	
	//��������
	x1 = hight * sin(roll*3.141f/180.0f);
	
	//ϵ��
	coe_bias = (0.0021f*hight*hight+0.2444f*hight+8.9576f) / 40.0f;
	
	real_bias = coe_bias * bias - x1;
	
	return real_bias;
}

/*
	T��ʱ��������λus��
	bias������ˮƽƫ��������λcm��
	bias_old���ϴ�ˮƽƫ��������λcm��
*/
float get_speed(u32 T,float bias,float bias_last)
{
	/*
		bias��ֵ��  + <--- ---> -
		speed���� + <--- ---> - ������ٶ�Ϊ�������ҷ��ٶ�Ϊ��
	*/
	
	float dx,dt,speed;
	dx = bias - bias_last;
	dt = T / 1000000.0f;
//	speed = dx / dt;
	speed = safe_div(dx,dt,0);
	
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
float bias_lpf_old;	//��һ���ڿ��÷�Χ�ڵ�bias
void Camera_Calculate(void)
{
	static float bias_old;

	//**************************************
	//����У׼���˲�
	if(ABS(bias)<50.0f)	//ֻ���ں���Χ�ڲŻ������������ͬʱ���е�ͨ�˲�
	{
		//�������
		
		//ƫ������
		bias_real = bias_correct(Roll_Image,Pitch_Image, Height_Image/10.0f,bias);	//��̬���У׼
		bias_lpf = cam_bias_lpf(bias_real,receive_T,0.8f,bias_lpf);		//��ͨ�˲���
		
		//�ٶ�����
		speed_d_bias = get_speed(receive_T,bias_lpf,bias_lpf_old);
		speed_d_bias_lpf = cam_bias_lpf(speed_d_bias,receive_T,1.0f,speed_d_bias_lpf);
		
		bias_lpf_old = bias_lpf;
	}
	else
	{
		speed_d_bias = 0;
		speed_d_bias_lpf = 0;
	}
	
}

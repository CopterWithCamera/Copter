#include "height_function.h"
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
#include "track_mode.h"


//========================================================================================
//========================================================================================
//										�߶ȿ���
//========================================================================================
//========================================================================================

//0.�ַ�ģʽ
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

//1.1������ǰ�߶ȣ�����ģʽʱ������ǰ�߶ȣ���������λ��λ�ÿ��ƣ�
u8 height_lock_flag = 0;
void height_lock()	//en -- ģʽ���ñ�־λ�������жϴ�ģʽ�Ƿ�ʹ��
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

//1.2��������
void height_lock_clear(void)
{
	height_lock_flag = 0;
}

//2.����ָ��ظ�
void height_hold(void)	//
{
	Thr_Low = 0;	//���ŵͱ�־Ϊ0
	
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//��ֹ�����߶ȹ���
	if(my_except_height < 150)
		my_except_height = 150;		//15cm
}

//3.����������Զ�����ڼ������ֵ�����ֵȡֵ��Χ��-499 -- +499��
//��ģʽ��ʱ�����У�����ʱ����
u8 auto_take_off;	//�Զ���ɱ�־λ
void take_off(float dT)	//dT��λ��s
{
	static float thr_auto = 0.0f;
	static float time_counter = 0;
	
	my_height_mode = 0;		//ҡ�˿��Ƹ߶�
	
	if(fly_ready==0)	//û����ʱ�����߶�Ϊ��ǰ�߶�
	{
		Thr_Low = 1;	//���ŵͱ�־Ϊ1������ͣת
		CH_ctrl[THR] = -499;	//�������
		auto_take_off = 0;	//����֮ǰ�Զ���ɱ�־λΪ0��ֻ������״̬������auto_take_off���㣬��ֹ����ڼ䷴�����ô˺�����
		return;
	}
	
	Thr_Low = 0;	//���ŵͱ�־Ϊ0�����ת�ٱ���
	
	if( auto_take_off == 0)
	{
		auto_take_off = 2;	//�Ѿ���������û�п�ʼ�Զ������ʼ�Զ���ɣ���ֵ1λԤ�������ǰ׼��ģʽ
	}
	
	#if defined(__COPTER_NO1)
	
		if(auto_take_off == 2)	//���������������˼��Ҫ������һ��������200����Χ��-500 -- +500�����������70%������
		{
			thr_auto = 300;		//�趨�������
			auto_take_off = 3;	//��ʼ��������
		}
		else if(auto_take_off == 3)	//Ȼ�����ʱ��һ�����������½��������������Ƶ����2ms��0.002*200 = 0.4��500*0.4=200����1s֮�������������
		{
			if(thr_auto > 0.0f)
			{
				thr_auto -= 180 *dT;	//���Ż�����С����2ms����������1.666s�к�˱�������
			}
			else
			{
				auto_take_off = 4;
				time_counter = 1.0;
			}
		}
		else if(auto_take_off == 4)		//1s����Ȼ��������ǰ�߶�
		{
			thr_auto = 50;	//thr��������+-40��40���ϲ���Ч
			
			time_counter -= dT;
			
			if(time_counter<0)
			{
				auto_take_off = 5;
			}
		}
		else
		{
			height_command = 2;		//��������ָ��
		}
		
		thr_auto = LIMIT(thr_auto,0,300);	//0������ͣ��300���������ֵ
	
	#elif defined(__COPTER_NO2)
		
		if(auto_take_off == 2)	//���������������˼��Ҫ������һ��������200����Χ��-500 -- +500�����������70%������
		{
			thr_auto = 350;		//�趨�������
			auto_take_off = 3;	//��ʼ��������
		}
		else if(auto_take_off == 3)	//Ȼ�����ʱ��һ�����������½��������������Ƶ����2ms��0.002*200 = 0.4��500*0.4=200����1s֮�������������
		{
			if(thr_auto > 0.0f)
			{
				thr_auto -= 180 *dT;	//���Ż�����С����2ms����������1.666s�к�˱�������
			}
			else
			{
				auto_take_off = 4;
				time_counter = 1.0;
			}
		}
		else if(auto_take_off == 4)		//1s����Ȼ��������ǰ�߶�
		{
			thr_auto = 100;	//thr��������+-40��40���ϲ���Ч
			
			time_counter -= dT;
			
			if(time_counter<0)
			{
				auto_take_off = 5;
			}
		}
		else
		{
			height_command = 2;		//��������ָ��
		}
		
		thr_auto = LIMIT(thr_auto,0,300);	//0������ͣ��300���������ֵ
	
	#endif

	CH_ctrl[THR] = thr_auto;	//���ֵ��������
	
	mydata.d14 = (u16)thr_auto;
}

//4.����
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

//********************************************************************************************

////�½���15cm
//void falling_to_15cm(void)	//����߶��൱�ڽ����ˣ�����ܴ�Լ��15cm��һ��㣩
//{
//	Thr_Low = 1;	//���ŵͱ�־Ϊ1������������ͣת
//	
//	//�����߶ȿ��Ƹ߶�
//	my_height_mode = 1;

//	//���������߶�
//	my_except_height = 150;		//�½���150mm = 15cm
//}

////������50cm
//void rising_to_50cm(void)
//{
//	Thr_Low = 0;	//���ŵͱ�־��0����ֹ����������ͣת
//	
//	//�����߶ȿ��Ƹ߶�
//	my_height_mode = 1;

//	//���������߶�
//	my_except_height = 500;		//������50cm
//}

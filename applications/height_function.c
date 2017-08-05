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
#include "height_function.h"

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

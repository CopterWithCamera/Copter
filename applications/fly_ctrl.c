#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"

//���帨��ͨ����Ӧ����ģʽ�ĺ�

#define	FUNCTION_1					hand_ctrl()
#define FUNCTION_2					height_lock()
#define FUNCTION_3					fly_ctrl_land()


//=================== filter ===================================
//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
//=================== filter ===================================

float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ

//�ַ�
void hand_ctrl(void)
{
	//�ַ�ģʽ�¸����ͺ��������
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0����� ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	CH_ctrl[2] = CH_filter[2];	//2������ THR
	CH_ctrl[3] = CH_filter[3];	//3������ YAW
}

//�߶�����
void height_lock(void)
{
	CH_ctrl[0] = CH_filter[0];	//0�����
	CH_ctrl[1] = CH_filter[1];	//1������
	CH_ctrl[3] = CH_filter[3];	//3������

	CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�

}

//�߶���̬����
void height_attitude_lock(void)
{
	CH_ctrl[0] = 0;	//0�����
	CH_ctrl[1] = 0;	//1������
	CH_ctrl[3] = 0;	//3������
	CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�
}

//����
float height_speed_ctrl = 0;
void fly_ctrl_land(void)	//��������2ms
{
	static u16 lock_time = 0;
	
	//���亯��ֻ������ɺ����
	
	//������ģʽ�л�������ģʽ
	if(ctrl_command != ctrl_command_old)
	{
		set_height_e = 0;	//�����ٶȲ����
	}
	
	//���������������������
	CH_ctrl[0] = CH_filter[0];	//0�����
	CH_ctrl[1] = CH_filter[1];	//1������
	CH_ctrl[3] = CH_filter[3];	//3������

//	//ҡ�˿��ƣ�ת��Ϊ�ٶ�������
//	CH_ctrl[2] = -60;	//-50��Ӧ�Ĵ�Լ��0.3m/s��-100��Լ��0.6m/s��������ֵͨ��ʵ��ȷ��
	
	//�趨������ֱ�ٶ�
	height_speed_ctrl = -400;	//��λmm/s
	CH_ctrl[2] = 0;				//����ֵ����ͣ�ת�����������⵽���ŵ�ʱ������������ͣת
	
	if(ultra.relative_height < 5)	//��ǰ�ɻ��ڵ���ʱ������������3-4cm���;�������йأ�
	{
		height_speed_ctrl = -2000;	//�л�Ϊ����½��ٶȣ�����ͨ��������������ͣת��
		
		lock_time++;			//�������ۼ�
		if(lock_time > 1000)	//2s
		{
			fly_ready = 0;	//����
		}
	}
	
}

//���
u8 takeoff_allow = 0;	//������ɱ�־������ʱ����������ɿ��ƺ�����1 -- ������ɣ�0 -- �����ɻ������
void fly_ctrl_takeoff(void)	//��������2ms
{
	static u16 counter = 0;
	
	if(fly_ready == 1)	//�Ѿ���������ɴ������ִ��
	{
		if(takeoff_allow == 1)	//�������
		{
			if(thr_take_off_f == 0)
			{
				thr_take_off_f = 1;	//��ɱ�־λ��1���߶ȿ��ƴ���ֻ������ɱ�־λ��1ʱ�Ż����У�
				thr_take_off = 500; //ֱ�Ӹ�ֵ��ɻ�׼����
			}
			
			height_speed_ctrl = 500;	//500mm/s����
			
			if(ultra.relative_height > 35)	//����30cm�������������4-5cm��
			{
				counter++;	//ÿ2ms�ۼ�1��
				
				if(counter > 150)	//0.3s ��Ϊ�Ѿ��ȶ�����30cm���ų���������Ӱ��
				{
					takeoff_allow = 0;	//������
					counter = 0;
				}
			}
			else
			{
				counter = 0;
			}
		}
		else	//��������ɣ��Ѿ��ڷ��У����������ִ�����
		{
			height_lock();	//�߶�����ģʽ�����ߵȴ�
		}
	}
}

//========================================================================================================

//ʶ�����ָ���mode_check�����е��ã�������ͨ����ֵ��
u8 ctrl_command;
u8 ctrl_command_old;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{
	//������ʷģʽ
	ctrl_command_old = ctrl_command;
	
	//����AUX2ͨ������6ͨ��������ֵ�����Զ�����ָ��
	if(*(ch_in+AUX2) <-200)		//��������������
	{
		ctrl_command = 1;
	}
	else if(*(ch_in+AUX2) <200)		//�����������м�
	{
		ctrl_command = 2;	
	}
	else							//��������������
	{
		ctrl_command = 3;	
	}
	
	//�Զ���λ����
	if(*(ch_in+AUX3) > 0)			//����
	{
		set_height_e = 0;	//�����ٶȲ����
	}
	
	//
	if(*(ch_in+AUX4) > 0)			//SWA,���ʹ�ܿ��أ�ֻ���ڿ��ز����Ϸ�ʱ������ܱ������PWM
	{
		All_Out_Switch = 0;
	}
	else
	{
		All_Out_Switch = 1;
	}
}

/* ************************************************************
	�����Զ����ƺ���

	����ctrl_command���ò�ͬ���Զ����ƺ���

	mode_state��
	0���ֶ�				1����ѹ��
	2��������+��ѹ��		3���Զ�

	ctrl_command��
	0���������ֶ�����ģʽ��������+��ѹ�ƶ��ߣ�		1���߶�����
	2���߶�����+��̬����							3������ģʽ
												
*************************************************************** */

void Fly_Ctrl(void)	
{
	
	//��������5ms
	
	if(mode_state != 3)
	{
		//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
		return;
	}
	
	//������ʱ��Ϊ�Ѿ����䣬���һЩ��־λ��Ϊ�´������׼��
	if(fly_ready == 0)
	{
		takeoff_allow = 1;	//������ɣ�ֻ��û�����ǰ������ִ����ɺ�����
	}
	
	switch(ctrl_command)
	{
		case 1:
			FUNCTION_1;
		break;
		
		case 2:
			FUNCTION_2;
		break;
		
		case 3:
			FUNCTION_3;
		break;
		
		default:
			hand_ctrl();	//Ĭ�ϣ����⣩������ַ�
		break;
	}

}


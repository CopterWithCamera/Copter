#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"

float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ
float my_except_height = 0;//�����߶�
u8 my_height_mode = 0;	//ģʽʹ�õĶ���ģʽ
						//0������
						//1�������߶�
						//2�������ٶ�

//========================================================================================

void set_except_height(u8 height)	//����߶����ݣ���λcm��ȡֵ��Χ0cm-255cm
{
	my_except_height = height * 10;
}

void set_attitude_calibration(u8 cmd)
{
	switch(cmd)
	{
		case 0x01:	//ǰ
			
		break;
		
		case 0x02:	//��
			
		break;
		
		case 0x03:	//��
			
		break;
		
		case 0x04:	//��
			
		break;
		
		case 0x05:	//�洢
			
		break;
		
		default:
			
		break;
	}
}


//========================================================================================

//�ַ�ģʽ
void hand(void)
{
	//=================== filter ===================================
	//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
	//=================== filter ===================================
	
	//ҡ�˿ظ�
	my_height_mode = 0;
	CH_ctrl[2] = CH_filter[2];	//2������ THR
}

//������ǰ�߶�
void lock_now_height(u8 en)	//en -- ģʽ���ñ�־λ�������жϴ�ģʽ�Ƿ�ʹ��
{
	static u8 height_lock_flag = 0;
	
	if(en)
	{
		//ģʽ������
		my_height_mode = 1;
		
		if(height_lock_flag == 0)
		{
			height_lock_flag = 1;
			
			//���������߶�
			my_except_height = sonar_fusion.fusion_displacement.out;	//��ȡ��ǰ�߶�
		}
	}
	else
	{
		height_lock_flag = 0;
	}
}

//���� my_except_height ����ֵ���Ƹ߶�
void use_my_except_height(void)
{
	//������͸߶�
	if( my_except_height < 120)	
		my_except_height = 120;
		
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;
}

//�½�����
void falling_to_15cm(void)	//����߶��൱�ڽ����ˣ�����ܴ�Լ��15cm��һ��㣩
{
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 150;		//�½���100mm = 10cm
}

//��������
void rising_to_50cm(void)
{
	//�����߶ȿ��Ƹ߶�
	my_height_mode = 1;

	//���������߶�
	my_except_height = 500;		//�½��� 150cm
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

void Fly_Ctrl(void)		//��������5ms
{
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	//==================== ���п����߼� ==========================
	
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
	
	//ָ��1
	if(ctrl_command == 0)
	{
		hand();
	}
	
	//ָ��2
	if(ctrl_command == 1)
	{
		lock_now_height(1);	//������ǰ�߶�
	}
	else
	{
		lock_now_height(0);	//�����־λ
	}
	
	//ָ��3
	if(ctrl_command == 2)
	{
		rising_to_50cm();
	}
	
	//ָ��4
	if(ctrl_command == 3)
	{
		hand();
	}
	
	//ָ��5
	if(ctrl_command == 4)
	{
		lock_now_height(1);
	}
	else
	{
		lock_now_height(0);
	}
	
	//ָ��6
	if(ctrl_command == 5)
	{
		falling_to_15cm();
	}
	
	//=======================================
	
	//����״������
	if(ctrl_command > 5)	//��Ӧ�ó��ֵ����
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2������ THR
	}
	
	/* ********************* ��̬���� ********************* */
	
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0����� ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	CH_ctrl[3] = CH_filter[3];	//3������ YAW
	
}

//ʶ�����ָ���mode_check�����е��ã�������ͨ����ֵ��
u8 ctrl_command;
u8 ctrl_command_old;
u8 All_Out_Switch = 0;
void Ctrl_Mode(float *ch_in)
{	
	//������ʷģʽ
	ctrl_command_old = ctrl_command;
	
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
	
	//�Զ���λ����
	if(*(ch_in+AUX3) > 0)			//����
	{
		set_height_e = 0;	//�����ٶȲ����
	}
	
	//��ͣ����
	//All_Out_Switch = 0ʱ��ͣ��All_Out_Switch = 1ʱ��������
	if(*(ch_in+AUX4) > 0)			//SWA,���ʹ�ܿ��أ�ֻ���ڿ��ز����Ϸ�ʱ������ܱ������PWM
	{
		//��ֹ���
		All_Out_Switch = 0;
	}
	else
	{
		//�������
		All_Out_Switch = 1;
	}
}

//==================== �ɷ��п����߼� ==========================
	
//	switch(ctrl_command)
//	{
//		case 1:
//			FUNCTION_1();
//		break;
//		
//		case 2:
//			FUNCTION_2();
//		break;
//		
//		case 3:
//			FUNCTION_3();
//		break;
//		
//		default:
//			hand_ctrl();	//����������ַ�
//		break;
//	}


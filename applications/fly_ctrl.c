#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"
#include "mymath.h"
#include "anotc_baro_ctrl.h"

//���帨��ͨ����Ӧ����ģʽ�ĺ�

#define	FUNCTION_1					hand_ctrl
#define FUNCTION_2					height_lock
#define FUNCTION_3					height_lock_displacement


//=================== filter ===================================
//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
//=================== filter ===================================

float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ

u8 my_height_mode = 0;	//ģʽʹ�õĶ���ģʽ
						//0������
						//1�������߶�
						//2�������ٶ�
						
float my_except_height = 0;//�����߶�

//�ַ�
void hand_ctrl(void)
{
	my_height_mode = 0;		//��������ֵ
	
	//�ַ�ģʽ�¸����ͺ��������
	CH_ctrl[0] = my_deathzoom( ( CH_filter[ROL]) ,0,30 );	//0����� ROL
	CH_ctrl[1] = my_deathzoom( ( CH_filter[PIT]) ,0,30 );	//1������ PIT
	CH_ctrl[2] = CH_filter[2];	//2������ THR
	CH_ctrl[3] = CH_filter[3];	//3������ YAW
}

//�߶�������ң��������ֵ��
void height_lock(void)
{
	my_height_mode = 0;		//��������ֵ
	
	CH_ctrl[0] = CH_filter[0];	//0�����
	CH_ctrl[1] = CH_filter[1];	//1������
	CH_ctrl[3] = CH_filter[3];	//3������

	CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�

}

//ֱ�����������߶ȶ���
void height_lock_displacement()
{
	static u8 flag;
	my_height_mode = 1;		//����Ŀ��߶Ȳ�
	
	if(!flag)
	{
		flag = 1;
		my_except_height = sonar_fusion.fusion_displacement.out;	//��ȡ��ǰ�߶�
	}
	
	CH_ctrl[0] = CH_filter[0];	//0�����
	CH_ctrl[1] = CH_filter[1];	//1������
	CH_ctrl[3] = CH_filter[3];	//3������
}

//�߶���̬����
void height_attitude_lock(void)
{
	my_height_mode = 0;
	
	CH_ctrl[0] = 0;	//0�����
	CH_ctrl[1] = 0;	//1������
	CH_ctrl[3] = 0;	//3������
	CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�
}


//========================================================================================================




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
	static u8 height_lock_flag = 0;
	
	//ֻ���Զ�ģʽ�Ż�ִ���Զ����ƴ���
	if(mode_state != 3)
	{
		return;
	}
	
	//==================== �·��п����߼� ==========================
	
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
	
	if(ctrl_command == 0)
	{
		//ҡ�˿ظ�
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2������ THR
		height_lock_flag = 0;
	}
	else if(ctrl_command == 1)
	{
		//�����߶ȿ��Ƹ߶�
		my_height_mode = 1;
		
		if(height_lock_flag == 0)
		{
			height_lock_flag = 1;
			
			//���������߶�
			my_except_height = sonar_fusion.fusion_displacement.out;	//��ȡ��ǰ�߶�
		}
	}
	else if(ctrl_command == 2)
	{
		//�����߶ȿ��Ƹ߶�
		my_height_mode = 1;

		//���������߶�
		my_except_height = 100;		//�½���100mm = 10cm
	}
	else if(ctrl_command == 3)
	{
		
	}
	else if(ctrl_command == 4)
	{
		
	}
	else if(ctrl_command == 5)
	{
		
	}
	else	//Ӧ��ģʽ���ֶ���������
	{
		my_height_mode = 0;
		CH_ctrl[2] = CH_filter[2];	//2������ THR
		
		height_lock_flag = 0;
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


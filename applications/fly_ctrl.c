#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"
#include "ultrasonic.h"

//=================== filter ===================================
//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
//=================== filter ===================================

float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ

//0���ַ�
void hand_ctrl(void)
{
	CH_ctrl[0] = CH_filter[0];	//0�����
	CH_ctrl[1] = CH_filter[1];	//1������
	CH_ctrl[2] = CH_filter[2];	//2������
	CH_ctrl[3] = CH_filter[3];	//3������	
}

//1���߶�����
void height_lock(void)
{
	CH_ctrl[0] = CH_filter[0];	//0�����
	CH_ctrl[1] = CH_filter[1];	//1������
	CH_ctrl[3] = CH_filter[3];	//3������

	CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�

}

//2���߶���̬����
void height_attitude_lock(void)
{
	CH_ctrl[0] = 0;	//0�����
	CH_ctrl[1] = 0;	//1������
	CH_ctrl[3] = 0;	//3������
	CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�
}

//3������
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

//4�����
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

//ʶ�����ָ��
u8 ctrl_command;
u8 ctrl_command_old;
void Ctrl_Mode(float *ch_in)
{
	//����ctrl_command_old
	ctrl_command_old = ctrl_command;
	
	//����AUX2ͨ������6ͨ��������ֵ�����Զ�����ָ��
	if(*(ch_in+AUX2) <-200)			//���
	{
		ctrl_command = 0;
	}
	else if(*(ch_in+AUX2) <200)		//�м�
	{
		ctrl_command = 3;
	}
	else							//���
	{
		ctrl_command = 4;
	}
	
	//�Զ���λ����
	if(*(ch_in+AUX3) > 0)			//����
	{
		set_height_e = 0;	//�����ٶȲ����
	}
	
	//
	if(*(ch_in+AUX4) > 0)			//
	{
		
	}
}

//�����Զ����ƺ���
void Fly_Ctrl(void)	//��������2ms
{
	uint8_t i;
	
	/*
	
	˵����
	
	mode_state��
	0���ֶ�
	1����ѹ��
	2��������+��ѹ��
	3���Զ�
	
	ctrl_command��
	0���������ֶ�����ģʽ��������+��ѹ�ƶ��ߣ�
	1���߶�����
	2���߶�����+��̬����
	3������ģʽ
	
	*/
	
	//������ʱ��Ϊ�Ѿ����䣬���һЩ��־λ��Ϊ�´������׼��
	if(fly_ready == 0)
	{
		takeoff_allow = 1;	//������ɣ�ֻ��û�����ǰ������ִ����ɺ�����
		
	}
	
	//ģʽ0 1 2�����ֶ����е�ģʽ���൱��ң�طɻ�
	//ֻ���л���ģʽ3ʱ���Ż����Զ����ƽ���
	if(mode_state == 0 || mode_state == 1 || mode_state == 2)	//	�ֶ�|��ѹ��|������+��ѹ��
	{
		//ͨ����ֵ����
		for(i=0;i<CH_NUM;i++)
		{
			CH_ctrl[i] = CH_filter[i];	//CH_filter[i]Ϊ����������ź�����ֵ����Դ�����ǽ��ջ���Ҳ����������
		}
	}
	else if(mode_state == 3)	//�Զ����߶ȿ����Ѿ�Ĭ���ǳ�����+��ѹ�ƶ��ߣ�
	{
		//0
		if(ctrl_command == 0)
		{
			hand_ctrl();				//�������ֶ�����ģʽ
				
			printf("hand\r\n");
		}
		//1
		else if(ctrl_command == 1)	
		{
			height_lock();				//�߶�����
			
			printf("height lock\r\n");
		}
		//2
		else if(ctrl_command == 2)	
		{
			height_attitude_lock();		//�߶�����+��̬����
			
			printf("attitude lock\r\n");
		}
		//3
		else if(ctrl_command == 3)	
		{	
			fly_ctrl_land();			//����ģʽ
			
			printf("land\r\n");
		}
		//4
		else if(ctrl_command == 4)
		{
			fly_ctrl_takeoff();			//���ģʽ
			
			printf("take off\r\n");
		}
	}
	
//	static u16 counter = 0;
//	counter++;
//	if(counter>20)
//	{
//		counter = 0;
//		
//		printf("command = %d\r\n",ctrl_command);
//	}
}


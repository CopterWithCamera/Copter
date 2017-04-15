#include "fly_ctrl.h"
#include "rc.h"
#include "fly_mode.h"

float CH_ctrl[CH_NUM];	//���������ctrl��ң����ֵ

void fly_ctrl(void)	//��������2ms
{
	uint8_t i;
	
	/*
	
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
	
	//ģʽ0 1 2�����ֶ����е�ģʽ���൱��ң�طɻ�
	//ֻ���л���ģʽ3ʱ���Ż����Զ����ƽ���
	if(mode_state == 0 || mode_state == 1 || mode_state == 2)	//	�ֶ�|��ѹ��|������+��ѹ��
	{
		
		//=================== filter ===================================
		//  ȫ�������CH_filter[],0�����1������2���ţ�3���� ��Χ��+-500	
		//=================== filter =================================== 	
		
		//ͨ����ֵ����
		for(i=0;i<CH_NUM;i++)
		{
			CH_ctrl[i] = CH_filter[i];	//CH_filter[i]Ϊ����������ź�����ֵ����Դ�����ǽ��ջ���Ҳ����������
		}
	}
	else if(mode_state == 3)	//�Զ����߶ȿ����Ѿ�Ĭ���ǳ�����+��ѹ�ƶ��ߣ�
	{
		if(ctrl_command == 0)	//�������ֶ�����ģʽ
		{
			CH_ctrl[0] = CH_filter[0];	//0�����
			CH_ctrl[1] = CH_filter[1];	//1������
			CH_ctrl[2] = CH_filter[2];	//2������
			CH_ctrl[3] = CH_filter[3];	//3������
		}
		else if(ctrl_command == 1)	//�߶�����
		{
			CH_ctrl[0] = CH_filter[0];	//0�����
			CH_ctrl[1] = CH_filter[1];	//1������
			CH_ctrl[3] = CH_filter[3];	//3������
			
			CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�
		}
		else if(ctrl_command == 2)	//�߶�����+��̬����
		{
			CH_ctrl[0] = 0;	//0�����
			CH_ctrl[1] = 0;	//1������
			CH_ctrl[3] = 0;	//3������
			CH_ctrl[2] = 0;	//2�����ţ�����λ����ֵ������Ϊ�߶ȱ��֣�
		}
		else if(ctrl_command == 3)	//���ٽ���
		{
			CH_ctrl[0] = CH_filter[0];	//0�����
			CH_ctrl[1] = CH_filter[1];	//1������
			CH_ctrl[3] = CH_filter[3];	//3������
			
			CH_ctrl[2] = -100;	//2�����ţ�����λ�����£�
		}
	}
}


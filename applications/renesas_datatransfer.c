#include "renesas_datatransfer.h"
#include "fly_ctrl.h"
#include "include.h"
#include "height_function.h"
#include "track_mode.h"
#include "rc.h"
#include "camera_datatransfer.h"
#include "track_mode.h"
#include "anotc_baro_ctrl.h"

u8 tmpbuff[20] = {0};

void renesas_setting_cmd(u8 data)
{
	if(data == 0x01)
	{
		fly_ready = 0;			//����
	}
	else if(data == 0x02)
	{
		fly_ready = 1;			//����
	}
	else if(data == 0x03)
	{
		All_Out_Switch = 0;		//��ͣ����ֹ�����
	}
	else
	{
		
	}
}

void renesas_height_cmd(u8 * data_array)
{
	switch(data_array[0])
	{
		case 0x01:													//�趨�����߶�
			height_command = 5;							//�л�Ϊָ��ظ�ģʽ
			my_except_height = data_array[1]*10.0f;		//���������߶�
		break;
		
		case 0x02:													//���
			height_command = 4;
		break;
		
		case 0x03:													//����
			height_command = 3;
		break;
		
		case 0x04:													//�ֶ�
			height_command = 1;
		break;
		
		default:
			
		break;
	}
}

void renesas_fly_cmd(u8 data)
{
	if(data == 1)				//��ͣ
	{
		ctrl_command = 3;
	}
	else if(data == 2)			//ǰ��
	{
		ctrl_command = 4; 
	}
	else if(data == 3)			//���˽���
	{
		ctrl_command = 6;
	}
	else if(data == 4)			//����
	{
		ctrl_command = 5;
	}
	else if(data == 5)			//�ֶ�
	{
		ctrl_command = 1;
	}
}

//��������ӿ�
void renesas_data_receive_handle(u8 data)
{
	static u8 mode = 0;
	static u8 counter = 0;
	
	switch(mode)
	{
		case 0:
			if(data == 0xBB)
				mode = 1;
			else
				mode = 0;
		break;
			
		case 1:
			if(data == 0xE1)
				mode = 2;
			else
				mode = 0;
		break;
			
		case 2:
			if(data == 0x01)			//��������
			{
				mode = 11;
			}
			else if(data == 0x02)		//�߶����2�ֽڣ�
			{
				mode = 12;
				counter = 0;
			}
			else if(data == 0x03)		//��������
			{
				mode = 13;
			}
			else
			{
				mode = 0;
			}
		break;
			
		case 11:						//��������
			renesas_setting_cmd(data);
			mode = 0;
		break;
		
		case 12:						//�߶����2�ֽڣ�
			tmpbuff[counter] = data;
			counter++;
			if(counter>=2)
			{
				counter = 0;
				renesas_height_cmd(tmpbuff);
				mode = 0;
			}
		break;
		
		case 13:						//��������
			renesas_fly_cmd(data);
			mode = 0;
		break;
		
		default:
			mode = 0;
		break;
	}
}

//===============================================================================

u8 RenesasSendBuff[20] = {0};

void renesas_send_state(void)
{
	RenesasSendBuff[0] = 0xBB;
	RenesasSendBuff[1] = 0xEE;
	
	RenesasSendBuff[2] = 0x01;
	
	RenesasSendBuff[3] = fly_ready;
	RenesasSendBuff[4] = tracking_state;
	RenesasSendBuff[5] = All_Out_Switch;	//0����ͣ   1�����
	RenesasSendBuff[6] = copter_fly_mode;
	RenesasSendBuff[7] = copter_height_mode;
	RenesasSendBuff[8] = (u8)(sonar_fusion.fusion_displacement.out/10.0f);
	
	Usart1_Send(RenesasSendBuff,9);	//���͵�����1
}

void renesas_data_send(void)
{
	renesas_send_state();
}


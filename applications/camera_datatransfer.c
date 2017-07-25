#include "camera_datatransfer.h"
#include "camera_data_calculate.h"
#include "usart.h"
#include "anotc_baro_ctrl.h"

/*

�߶����ݣ�

ultra.relative_height					cm		float
sonar.displacement						mm		float
sonar_fusion.fusion_displacement.out	mm		float

*/

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


//=========================================================================================================================
//====================================================��������==============================================================
//=========================================================================================================================
//=========================================================================================================================

unsigned char Data_Buffer[20];

//�������ݽӿ�
void Send_to_Camera(unsigned char *DataToSend ,u8 data_num)
{
	Usart3_Send(DataToSend,data_num);
}

//���͸߶�����
void Copter_Send_Height(void)
{
	float tmp_f;
	
	u8 cnt = 0;
	
	//֡ͷ
	Data_Buffer[cnt++] = 0xAA;	
	Data_Buffer[cnt++] = 0xAA;
	
	//������
	Data_Buffer[cnt++] = 0x01;	
	
	//����
	tmp_f = ultra.relative_height*10;	//תΪmm��λ
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = sonar.displacement;
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = sonar_fusion.fusion_displacement.out;
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	Send_to_Camera(Data_Buffer,cnt);
}

//������̬����
void Copter_Send_Attitude(void)
{
	float tmp_f;
	
	u8 cnt = 0;
	
	//֡ͷ
	Data_Buffer[cnt++] = 0xAA;	
	Data_Buffer[cnt++] = 0xAA;
	
	//������
	Data_Buffer[cnt++] = 0x02;	
	
	//����
	tmp_f = Roll;	//�������λ�ǡ�
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = Pitch;	//����
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	tmp_f = Yaw;	//����
	Data_Buffer[cnt++] = BYTE0(tmp_f);
	Data_Buffer[cnt++] = BYTE1(tmp_f);
	Data_Buffer[cnt++] = BYTE2(tmp_f);
	Data_Buffer[cnt++] = BYTE3(tmp_f);
	
	Send_to_Camera(Data_Buffer,cnt);
}

//��ʱ�������ݺ�����scheduler.c�е��ã�
void Copter_Data_Send(void)
{
	Copter_Send_Height();
	Copter_Send_Attitude();
}

//=========================================================================================================================
//=========================================================================================================================
//=================================================== ������ ==============================================================
//=========================================================================================================================
//=========================================================================================================================

//���ò�����

float bias = 0;		//ƫ��
float angle = 0;	//�Ƕ�
float bias_pitch = 0;	//�ٶ�

//����
float fps = 0;
float processing_fps = 0;
float receive_T = 0;

float Roll_Image = 0;		//�����Ӧ�ĽǶ�
float Pitch_Image = 0;
float Yaw_Image = 0;
float Height_Image = 0;		//�����Ӧ�ĸ߶�

//=========================================================================================================================
//====================================================��������==============================================================
//=========================================================================================================================

//�����ݴ�����
unsigned char Tmp_Buffer[20];

//����Camera״̬��Ϣ
void Get_Camera_Status(void)
{
	fps = *((float*)(&(Tmp_Buffer[0])));
	processing_fps = *((float*)(&(Tmp_Buffer[4])));
	//tmp = *((float*)(&(Tmp_Buffer[8])));
}

//����ͼ��ɼ�ʱ����Ϣ
float Roll_Image_Latest = 0;		//���βɼ���ʼʱ��Ӧ��Roll
float Pitch_Image_Latest = 0;
float Yaw_Image_Latest = 0;
float Height_Image_Latest = 0;	//���βɼ���ʼʱ��Ӧ��Height
void Get_Camera_Get_Image_Flag(u8 mode)
{
	static float roll_tmp = 0;	//��ʱ����roll��ֵ
	static float pitch_tmp = 0;
	static float yaw_tmp = 0;
	static float height_tmp = 0;
	
	if(mode == 0)
	{
		//������ͼ
		roll_tmp = Roll;
		pitch_tmp = Pitch;
		yaw_tmp = Yaw;
		height_tmp = sonar.displacement;
	}
	else if(mode == 1)
	{
		//��ͼ+���㿪ʼ
		Roll_Image_Latest = roll_tmp;	//��ȡͼ�����ʱ��rollΪ��ǰ���ݵ�roll
		Pitch_Image_Latest = pitch_tmp;
		Yaw_Image_Latest = yaw_tmp;
		Height_Image_Latest = height_tmp;
		
		//����tmp������ֵ
		roll_tmp = Roll;
		pitch_tmp = Pitch;
		yaw_tmp = Yaw;
		height_tmp = sonar.displacement;
	}
}

//����ƫ����Ϣ
void Get_Position(void)
{
	receive_T = Get_Cycle_T(3);	//��usΪ��λ
	
	//��ȡ����
	bias  = *((float*)(&(Tmp_Buffer[0])));
	bias_pitch = *((float*)(&(Tmp_Buffer[4])));
	angle = *((float*)(&(Tmp_Buffer[8])));
	
	//��ȡ���ν����Ӧͼ��ɼ�ʱ�ķɻ���̬��Ϣ
	Roll_Image = Roll_Image_Latest;
	Pitch_Image = Pitch_Image_Latest;
	Yaw_Image = Yaw_Image_Latest;
	Height_Image = Height_Image_Latest;
	
	loop.camera_data_ok = 1;
}

//=========================================================================================================================
//=========================================================================================================================
//=========================================================================================================================
//=========================================================================================================================

u8 counter = 0;
void Copter_Receive_Handle(unsigned char data)
{
	static u8 mode = 0;
	
	switch(mode)
	{
		case 0:
			if(data == 0xAA)
				mode = 1;
			else
				mode = 0;
			break;
			
		case 1:
			if(data == 0xAF)
				mode = 2;
			else
				mode = 0;
			break;
			
		case 2:
			if(data == 0x01)	//���빦����1����
			{
				mode = 10;
				counter = 0;
			}
			else if(data == 0x02)	//���빦����2����
			{
				mode = 11;
				counter = 0;
			}
			else if(data == 0x03)
			{
				mode = 12;
				counter = 0;
			}
			else				//û�ж�Ӧ������
			{
				mode = 0;
			}
			break;
		
		case 10:
			Tmp_Buffer[counter] = data;	//3*4�ֽڣ��ܹ�ռ������0-11λ
			counter++;
			if(counter>=12)
			{
				Get_Position();	//�߶����ݻ�ȡ���
				mode = 0;
			}
		break;
			
		case 11:
			Tmp_Buffer[counter] = data;	//3*4�ֽڣ��ܹ�ռ������0-11λ
			counter++;
			if(counter>=12)
			{
				Get_Camera_Status();	//�װ�״̬��ȡ���
				mode = 0;
			}
		break;
			
		case 12:
			Get_Camera_Get_Image_Flag(data);
			mode = 0;
		break;
			
		default:
			mode = 0;
		break;
	}
}

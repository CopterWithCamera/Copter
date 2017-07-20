#include "camera_datatransfer.h"
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
	
void Real_Length_Calculate(void);

unsigned char Data_Buffer[20];

void Send_to_Camera(unsigned char *DataToSend ,u8 data_num)
{
	Usart3_Send(DataToSend,data_num);
}

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

//��Camera�巢������
void Copter_Data_Send(void)
{
	Copter_Send_Height();
	Copter_Send_Attitude();
}

//========================================================================================================

float Roll_Image_Last = 0;	//��һ֡ͼ�����ʱ��roll
float Height_Image_Last = 0;

float real_length = 0;

float length = 0;
float angle = 0;
float speed = 0;

float fps = 0;
float processing_fps = 0;

//�����ݴ�����
unsigned char Tmp_Buffer[20];

void Get_Camera_Status(void)
{
	fps = *((float*)(&(Tmp_Buffer[0])));
	processing_fps = *((float*)(&(Tmp_Buffer[4])));
	//speed = *((float*)(&(Tmp_Buffer[8])));
	
//	printf("%.1f  %.1f   %.1f\r\n", length, angle, speed);
}


void Get_Camera_Get_Image_Flag(u8 mode)
{
	static float roll_tmp = 0;	//��ʱ����roll��ֵ
	static float height_tmp = 0;
	
	if(mode == 0)
	{
		//������ͼ
		roll_tmp = Roll;
		height_tmp = sonar.displacement;
	}
	else if(mode == 1)
	{
		//��ͼ+���㿪ʼ
		Roll_Image_Last = roll_tmp;	//��ȡͼ�����ʱ��rollΪ��ǰ���ݵ�roll
		Height_Image_Last = height_tmp;
		
		roll_tmp = Roll;	//������ͼ
		height_tmp = sonar.displacement;
	}
}

float Roll_Image = 0;		//�����Ӧ�ĽǶ�
float Height_Image = 0;		//�����Ӧ�ĸ߶�
void Get_Position(void)
{
	length  = *((float*)(&(Tmp_Buffer[0])));
	angle    = *((float*)(&(Tmp_Buffer[4])));
	speed = *((float*)(&(Tmp_Buffer[8])));
	
	Roll_Image = Roll_Image_Last;
	Height_Image = Height_Image_Last;
	
	Real_Length_Calculate();
}

float bias_correct(float roll, float hight,float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1,real_bias;
	x1=hight*my_sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}

void Real_Length_Calculate(void)
{
	if(!speed)
		real_length = bias_correct(Roll_Image,Height_Image/10.0f,length);
	
	mydata.d1 = real_length;
	mydata.d2 = length;
}

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

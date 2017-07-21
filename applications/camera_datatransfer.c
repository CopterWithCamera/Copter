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

//ƫ��
float bias = 0;
float bias_lpf = 0;
float bias_real = 0;

//�Ƕ�
float angle = 0;

//�ٶ�
float speed = 0;

//����
float fps = 0;
float processing_fps = 0;
float receive_fps = 0;

//=========================================================================================================================
//=========================================================================================================================
//=================================================== У׼���� =============================================================
//=========================================================================================================================
//=========================================================================================================================

//��ͨ�˲���
//dt������ʱ��������λus��
//fc����ֹƵ��
float cam_bias_lpf(float bias ,float dt_us ,float fc,float last_bias_lpf)
{
	float q,out,T;
	
	T = dt_us / 1000000.0f;
	q = 6.28f * T * fc;
	
	if(q >0.95f)
		q = 0.95f;
	
	out = q * bias + (1.0f-q) * last_bias_lpf;
	
	return out;
}

//bias����У׼����
float bias_correct(float roll, float hight,float bias)   ///hight --����������ֵ   roll--���ƫ��  bias--ͼ�����ص�ƫ��
{
    float x1,real_bias;
	x1=hight*my_sin(roll*3.141f/180.0f);
	real_bias=0.65f*bias-x1;
	return real_bias;
}

//ʵ��ƫ��У׼
float Roll_Image = 0;		//�����Ӧ�ĽǶ�
float Height_Image = 0;		//�����Ӧ�ĸ߶�
void Real_Length_Calculate(float T)
{
	static float bias_old;
	
	//ȫ��ʱ��+-100��ʾ
	if(speed)
	{
		if(bias_old > 0)
			bias = +100;
		else
			bias = -100;
	}
	bias_old = bias;

	//ֻ���ں���Χ�ڲŻ����
	//������ͬʱ���е�ͨ�˲�
	if(ABS(bias)<50)
	{
		//�������
		bias_real = bias_correct(Roll_Image,Height_Image/10.0f,bias);	//��̬���У׼
		bias_lpf = cam_bias_lpf(bias_real,T,0.8f,bias_lpf);		//��ͨ�˲���
	}
}

//=========================================================================================================================
//=========================================================================================================================
//====================================================��������==============================================================
//=========================================================================================================================
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
float Height_Image_Latest = 0;	//���βɼ���ʼʱ��Ӧ��Height
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
		Roll_Image_Latest = roll_tmp;	//��ȡͼ�����ʱ��rollΪ��ǰ���ݵ�roll
		Height_Image_Latest = height_tmp;
		
		roll_tmp = Roll;	//������ͼ
		height_tmp = sonar.displacement;
	}
}

//����ƫ����Ϣ
void Get_Position(void)
{
	//�������Ƶ��
	float T;	//ʱ��������usΪ��λ��
	T = Get_Cycle_T(3);	//��usΪ��λ
	receive_fps = 1 * 1000000.0f / T;	//ת��Ϊ��HzΪ��λ
	
	//��ȡ����
	bias  = *((float*)(&(Tmp_Buffer[0])));
	angle    = *((float*)(&(Tmp_Buffer[4])));
	speed = *((float*)(&(Tmp_Buffer[8])));
	
	//��ȡ���ν����Ӧͼ��ɼ�ʱ�ķɻ���̬��Ϣ
	Roll_Image = Roll_Image_Latest;
	Height_Image = Height_Image_Latest;
	
	//���ݲ���
	Real_Length_Calculate(T);
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

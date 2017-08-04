#ifndef __ANO_OF_H_
#define __ANO_OF_H_

#include "stm32f4xx.h"

//����Ϊȫ�ֱ������������ļ��У����ñ�h�ļ��������������ļ��з��ʵ����±���

extern uint8_t 	OF_QUA,OF_LIGHT;	//������Ϣ������QUA	//����ǿ�ȣ�LIGHT

extern int8_t	OF_DX,OF_DY;		//ԭʼ������Ϣ���������������ģ���ֲ�
extern int16_t	OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;		//�ںϺ�Ĺ�����Ϣ���������������ģ���ֲᣨ��λcm/s��
extern uint16_t	OF_ALT,OF_ALT2;		//ԭʼ�߶���Ϣ���ںϺ�߶���Ϣ����λcm��

extern int16_t	OF_GYR_X,OF_GYR_Y,OF_GYR_Z;		//ԭʼ����������
extern int16_t	OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;	//�˲�������������
extern int16_t	OF_ACC_X,OF_ACC_Y,OF_ACC_Z;		//ԭʼ���ٶ�����
extern int16_t	OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;	//�˲�����ٶ�����

extern float	OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;		//ŷ���Ǹ�ʽ����̬����
extern float	OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;//��Ԫ����ʽ����̬����


//��������Ψһһ����Ҫ�ⲿ���õĺ�������Ϊ����ģ���Ǵ����������
//���Ա�������Ҫ�ڴ��ڽ����ж��е��ã�ÿ����һ�ֽ����ݣ����ñ�����һ��
void AnoOF_GetOneByte(uint8_t data);

#endif

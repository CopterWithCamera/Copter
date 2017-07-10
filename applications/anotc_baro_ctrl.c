/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��anotc_baro_ctrl.c
 * ����    ����ѹ�ƿ���
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "anotc_baro_ctrl.h"
#include "ms5611.h"
#include "filter.h"
#include "fly_mode.h"

//�� ��Ч�������� �� lim �ĳ˻� ����ͨ�˲�����������ֵ�����com_val			//��δ�����޵���
float baro_compensate(float dT,float kup,float kdw,float vz,float lim)	//��ѹ����
{
	float z_sin;
	static float com_val,com_tar;
	
	z_sin = my_sqrt(1-my_pow(vz));
	
	//((z_sin/0.44f) *lim) ��2Hz��ͨ �õ� com_tar
	
	//com_tar = (z_sin/0.44f) *lim;
	LPF_1_(2.0f,dT,((z_sin/0.44f) *lim),com_tar);	//��ͨ�˲���
	com_tar = LIMIT(com_tar,0,lim);
	
	if(com_val<(com_tar-100))
	{
		com_val += 1000 *dT *kup;
	}
	else if(com_val>(com_tar+100))
	{
		com_val -= 1000 *dT *kdw;
	}
	return (com_val);
}

//����������ƽ�����˲�����λ��Ϊmm���þ���仯�����ٶȡ����ٶ�
void fusion_prepare(float dT,float av_arr[],u16 av_num,u16 *av_cnt,float deadzone,_height_st *data,_fusion_p_st *pre_data)
{
	//av_cnt ȡƽ������
	
	//�������������ٰѵ�λ������תΪ���ף����ȡ���ƽ��ֵ
	pre_data->dis_deadzone = my_deathzoom(data->relative_height,pre_data->dis_deadzone,deadzone);		//��������
	Moving_Average(av_arr,av_num ,(av_cnt),(10 *pre_data->dis_deadzone ),&(pre_data->displacement));	//ȡƽ��ֵ������->����
	
	pre_data->speed = safe_div(pre_data->displacement - pre_data->displacement_old,dT,0);	//�ٶ� = �����θ߶�-�ϴθ߶ȣ���ʱ��
	pre_data->acceleration = safe_div(pre_data->speed - pre_data->speed_old,dT,0);			//���ٶ�
	
	//��¼������
	pre_data->displacement_old = pre_data->displacement;
	pre_data->speed_old = pre_data->speed;
}

//���ٶ��ں�
void acc_fusion(float dT,_f_set_st *set,float est_acc,_fusion_p_st *pre_data,_fusion_st *fusion)	
{
	fusion->fusion_acceleration.out += est_acc - fusion->est_acc_old; //����
	anotc_filter_1(set->b1,set->g1,dT,pre_data->acceleration,&(fusion->fusion_acceleration));  //pre_data->acceleration //�۲⡢����
	
	fusion->fusion_speed_m.out += 1.1f *my_deathzoom(fusion->fusion_acceleration.out,0,20) *dT;	//���ٶ������ٶ�
	anotc_filter_1(set->b2,set->g2,dT,pre_data->speed,&(fusion->fusion_speed_m));				//pre_data->speed������ͨ�˲��õ�fusion->fusion_speed_m
	anotc_filter_1(set->b2,set->g2,dT,(-pre_data->speed + fusion->fusion_speed_m.out),&(fusion->fusion_speed_me));	//����ͨ�˵����ٶȷ��� ���д�ͨ�˲� �õ� fusion_speed_me
	fusion->fusion_speed_me.out = LIMIT(fusion->fusion_speed_me.out,-200,200);
	fusion->fusion_speed_m.a = LIMIT(fusion->fusion_speed_m.a,-1000,1000);
	
	fusion->fusion_displacement.out += 1.05f *(fusion->fusion_speed_m.out - fusion->fusion_speed_me.out) *dT;
	anotc_filter_1(set->b3,set->g3,dT,pre_data->displacement,&(fusion->fusion_displacement));
	
	fusion->est_acc_old = est_acc;
}

//�������ںϲ���

#define SONAR_AV_NUM 50
float sonar_av_arr[SONAR_AV_NUM];
u16 sonar_av_cnt;

_fusion_p_st sonar;
_fusion_st sonar_fusion;
_f_set_st sonar_f_set = {
													0.2f,
													0.5f,
													0.8f,
													
													0.2f,
													0.5f,
													0.8f
						};


//													0.2f,
//													0.3f,
//													0.5f,
//													
//													0.1f,
//													0.3f,
//													0.5f

//��ѹ���ںϲ���											
#define BARO_AV_NUM 100
float baro_av_arr[BARO_AV_NUM];
u16 baro_av_cnt;
_fusion_p_st baro_p;
_fusion_st baro_fusion;
_f_set_st baro_f_set = {
													0.1f,
													0.2f,
													0.3f,
													
													0.1f,
													0.1f,
													0.2f	
						};

//													0.2f,
//													0.3f,
//													0.5f,
//													
//													0.1f,
//													0.3f,
//													0.5f

float sonar_weight;		//����������Ȩ��
float wz_speed,sonarz_speed;	//wz_speed ����ѹ�����ݵó������׼ȷ�Ĵ�ֱ�ٶ�
void baro_ctrl(float dT,_hc_value_st *height_value)		//��ȡ�߶����ݣ���������2ms��
{
	static float dtime;
	static float baro_com_val;	//��ѹ�Ʋ�������
		
//*******************************************************************************************************************************
//											��ѹ�����ݲɼ�
//*******************************************************************************************************************************
	
	dtime += dT;	//�����dT����ѧ���ǡ�T����λ��s
	if(dtime > 0.01f) //10 ms
	{
		dtime = 0;
 		if( !MS5611_Update() )//����ms5611��ѹ�����ݣ�10ms����һ�Σ�ÿ�������λ��ȡһ����ѹ�ƣ�   MS5611_Update()   0����ѹ   1���¶�
		{
			baro.relative_height = baro.relative_height - 0.1f *baro_com_val;
		}
	}

	//baro.h_dt = 0.02f; //��ѹ�ƶ�ȡ���ʱ��20ms		//�����û���ϣ�
	
	//��ѹ�Ʋ���		//dT >= 2ms��2ms���ң�
	//��δ��������⣬�����߼�
	baro_com_val = baro_compensate(dT,1.0f,1.0f,reference_v.z,3500);	//��ѹ�Ʋ���������ѹ�ƶ�ȡ����ʧ��ʱ����reference_v.z���в���
	
//*******************************************************************************************************************************
//							�߶������ں�У׼�����������������˲�����λ�����ںϼ��ٶ����ݣ�
//*******************************************************************************************************************************
	
	//��ѹ�����ݼ��ٶ��ں�
	fusion_prepare(	dT,			baro_av_arr,	BARO_AV_NUM,		&baro_av_cnt,		2,			&baro,			&baro_p);
	//				ʱ��΢��		��ѹ�ƻ�������	�����������ݸ���		�����˲���������		�������		�ɼ���������		
	acc_fusion(	dT,			&baro_f_set,	acc_3d_hg.z,	&baro_p,			&baro_fusion);
	//		   ʱ��΢��		���ò���			���ٶ�ֵ			׼���õ�����			�ں����
	
	//���������ݼ��ٶ��ں�
	fusion_prepare(dT,sonar_av_arr,SONAR_AV_NUM,&sonar_av_cnt,0,&ultra,&sonar);	//����������ƽ�����˲�����λ��Ϊmm���þ���仯�����ٶȡ����ٶ�
	acc_fusion(	dT,			&sonar_f_set,	acc_3d_hg.z,	&sonar,				&sonar_fusion);				//acc_3d_hg.z��sonar�����룬sonar_fusion�����
	//		   ʱ��΢��		���ò���			���ٶ�ֵ			׼���õ�����			�ں����

//*******************************************************************************************************************************
//	�߶������ںϣ�ѡ��������Դ��
//*******************************************************************************************************************************
	if(ultra.measure_ok == 1)	//������������Ч
	{
		sonar_weight += 0.5f *dtime;	//sonar_weightԽ�󣬳��������ݵ�Ȩ��Խ��
	}
	else
	{
		sonar_weight -= 2.0f *dtime;
	}
	sonar_weight = LIMIT(sonar_weight,0,1);
	
	//ģʽ1Ϊ����ѹ��ģʽ
	if(mode_state == 1)	//ͨ��5ѡ����ѹ��ģʽ�����������ݲ�����߶������ں�
	{
		sonar_weight = 0;
	}

//==========================================================================
	
	//		       �ں��˼��ٶȼ����ݾ�����ͨ�˲�		 ����ͨ�˵�����ѹ���ٶ���������ͨ�˲�
	wz_speed     = baro_fusion.fusion_speed_m.out  - baro_fusion.fusion_speed_me.out;	//wz_speed ����ѹ�����ݵó������׼ȷ�Ĵ�ֱ�ٶ�
	sonarz_speed = sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out;	//���������������������Ծ�ȷ��ֱ�ٶ�
	
	//��ѹ�ơ�������ԭʼ�����ں�
	//�ٶ��ں�
	float m_speed,f_speed;
	m_speed = (1 - sonar_weight) * baro_p.speed + sonar_weight * (sonar.speed);					//����Դ������ƽ�����˲�����λת��Ϊmm���ٶ�ֵ
	f_speed = (1 - sonar_weight) * (wz_speed)   + sonar_weight * (sonarz_speed);				//����Դ���ں��˼��ٶȼƵ�����
	//f_speed = (1 - sonar_weight) * (wz_speed)   + sonar_weight * (sonar_fusion.fusion_speed_m.out - sonar_fusion.fusion_speed_me.out);	
	
	
	//���ٶȲ��þ�����Ч�������������ļ��ٶȼ�����
	//�ٶȲ��ó�����+��ѹ���ںϵ��ٶ�����
	//�߶Ȳ�����ѹ��+���ٶȼ��ںϵ���ѹ�Ƹ߶�
	height_value->m_acc = acc_3d_hg.z;
	height_value->m_speed = m_speed;  //(1 - sonar_weight) *hf1.ref_speed_lpf + sonar_weight *(sonar.speed);	//m_speed
	height_value->m_height = baro_p.displacement;
	
	//���ٶȲ����ں�����ѹ�����ݡ����ٶȼ����ݵļ��ٶ�
	//�ٶ����ݲ��������������޷���ĳ�����+��ѹ���ںϵ��ٶ�����
	//�߶����ݲ����ںϺ����ѹ������
	height_value->fusion_acc = baro_fusion.fusion_acceleration.out;
	height_value->fusion_speed = my_deathzoom(LIMIT( (f_speed),-MAX_VERTICAL_SPEED_DW,MAX_VERTICAL_SPEED_UP),height_value->fusion_speed,10);	//f_speed�޷�+����������仯С������Ϊû�仯��
	height_value->fusion_height = baro_fusion.fusion_displacement.out; 
	
	//return (*height_value);
}



/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

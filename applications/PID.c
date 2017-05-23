/******************** (C) COPYRIGHT 2016 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��PID.c
 * ����    ��PID������
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "PID.h"

//_PID_arg_st	ϵ��
//_PID_val_st	����

float PID_calculate( float T,            //���ڣ���λ���룩
										float in_ff,				//ǰ��ֵ
										float expect,				//����ֵ���趨ֵ��
										float feedback,				//����ֵ����һ�εĵ����ṹ��Ҳ���ǵ�ǰ��״̬���룩
										_PID_arg_st *pid_arg, 		//PID�����ṹ��
										_PID_val_st *pid_val,		//PID���ݽṹ��
										float inte_lim				//integration limit�������޷�
										 )	
{
	float out,differential;
	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);	//k_inc_d_norm �޷�Ϊ 0 -- 1
	
	//����ͬ�ϴβ���ֵ�Ĳ��
	pid_val->feedback_d = (-1.0f) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);	//�󱾴�feedback���ϴ�feedback�Ĳ�൱��΢�֣�d��
	
	//error
	pid_val->err =  (expect - feedback );	//ƫ�� = ���� - ����
	
	//error�Ĳ��
	pid_val->err_d = (pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);
	
	//kd * d_err + kd_feedbeck * d_feedbeck		���� d �ĺͣ������������Ʊ仯���������
	differential = (pid_arg->kd *pid_val->err_d + pid_arg->k_pre_d *pid_val->feedback_d);
	
	LPF_1_(pid_arg->inc_hz,T,differential,pid_val->err_d_lpf);	//�� d ����ֵ differential ���е�ͨ�˲����õ����յ� d
																//���� pid_val->err_d_lpf
	
	pid_val->err_i += (pid_val->err + pid_arg->k_pre_d *pid_val->feedback_d )*T;//)*T;//
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_arg->ki *pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
	
	return (out);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

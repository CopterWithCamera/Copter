#include "ano_of.h"
#include "scheduler.h"
#include "data_transfer.h"

uint8_t		OF_QUA,OF_LIGHT;
int8_t		OF_DX,OF_DY;
int16_t		OF_DX2,OF_DY2,OF_DX2FIX,OF_DY2FIX;
uint16_t	OF_ALT,OF_ALT2;
int16_t		OF_GYR_X,OF_GYR_Y,OF_GYR_Z;
int16_t		OF_GYR_X2,OF_GYR_Y2,OF_GYR_Z2;
int16_t		OF_ACC_X,OF_ACC_Y,OF_ACC_Z;
int16_t		OF_ACC_X2,OF_ACC_Y2,OF_ACC_Z2;
float		OF_ATT_ROL,OF_ATT_PIT,OF_ATT_YAW;
float		OF_ATT_S1,OF_ATT_S2,OF_ATT_S3,OF_ATT_S4;

void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num);
	
void AnoOF_GetOneByte(uint8_t data)
{
	static uint8_t _datatemp[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0xAA)
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3&&data<45)
	{
		state = 4;
		_datatemp[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		_datatemp[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		_datatemp[4+_data_cnt]=data;
		AnoOF_DataAnl(_datatemp,_data_cnt+5);
	}
	else
		state = 0;
}

void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	
	if(*(data_buf+2)==0X51)//光流信息
	{
		if(*(data_buf+4)==0)//原始光流信息
		{
			OF_QUA 		= *(data_buf+5);
			OF_DX  		= *(data_buf+6);
			OF_DY  		= *(data_buf+7);
			OF_LIGHT  	= *(data_buf+8);
		}
		else if(*(data_buf+4)==1)//融合后光流信息
		{
			OF_QUA 		= *(data_buf+5);
			OF_DX2		= (int16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
			OF_DY2		= (int16_t)(*(data_buf+8)<<8)|*(data_buf+9) ;
			OF_DX2FIX	= (int16_t)(*(data_buf+10)<<8)|*(data_buf+11) ;
			OF_DY2FIX	= (int16_t)(*(data_buf+12)<<8)|*(data_buf+13) ;
			OF_LIGHT  	= *(data_buf+14);
			
			loop.flow_data_ok = 1;	//指示光流速度数据已经更新
		}
	}
	if(*(data_buf+2)==0X52)//高度信息
	{
		if(*(data_buf+4)==0)//原始高度信息
		{
			OF_ALT = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
		}
		else if(*(data_buf+4)==1)//融合后高度信息
		{
			OF_ALT2 = (uint16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
		}
	}
	if(*(data_buf+2)==0X53)//惯性数据
	{
		if(*(data_buf+4)==0)//原始数据
		{
			OF_GYR_X = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
			OF_GYR_Y = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			OF_GYR_Z = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			OF_ACC_X = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			OF_ACC_Y = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			OF_ACC_Z = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
		}
		else if(*(data_buf+4)==1)//滤波后数据
		{
			OF_GYR_X2 = (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) ;
			OF_GYR_Y2 = (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			OF_GYR_Z2 = (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			OF_ACC_X2 = (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			OF_ACC_Y2 = (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			OF_ACC_Z2 = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
		}
	}
	if(*(data_buf+2)==0X54)//姿态信息
	{
		if(*(data_buf+4)==0)//欧拉角格式
		{
			OF_ATT_ROL = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.01 ;
			OF_ATT_PIT = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.01 ;
			OF_ATT_YAW = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.01 ;
		}
		else if(*(data_buf+4)==1)//四元数格式
		{
			OF_ATT_S1 = ((int16_t)(*(data_buf+5)<<8)|*(data_buf+6)) * 0.0001 ;
			OF_ATT_S2 = ((int16_t)(*(data_buf+7)<<8)|*(data_buf+8)) * 0.0001 ;
			OF_ATT_S3 = ((int16_t)(*(data_buf+9)<<8)|*(data_buf+10)) * 0.0001 ;
			OF_ATT_S4 = ((int16_t)(*(data_buf+11)<<8)|*(data_buf+12)) * 0.0001 ;
		}
	}
}


//低通滤波器
//dt：采样时间间隔（单位us）
//fc：截止频率
float flow_bias_lpf(float bias ,float dt_us ,float fc,float last_bias_lpf)
{
	float q,out,T;
	
	T = dt_us / 1000000.0f;
	q = 6.28f * T * fc;
	
	if(q >0.95f)
		q = 0.95f;
	
	out = q * bias + (1.0f-q) * last_bias_lpf;
	
	return out;
}

//自己编写的光流数据处理函数

float	OF_DX2_DETECT,		//横滚速度		- <---  ---> +
		OF_DY2_DETECT,		//俯仰速度		+ <前-- --后> -
		OF_DX2FIX_DETECT,
		OF_DY2FIX_DETECT,
		OF_DX2FIX_DETECT_LPF,
		OF_DY2FIX_DETECT_LPF;

//光流数据置信函数，与融合后光流信息接收同频率调用
void flow_data_detect(void)
{
	//本函数的编写基础为每一帧的光流速度数据都有自己相对应的quality数据
	if( OF_QUA > 70 )	
	{
		//可信度很高，直接更新
		OF_DX2_DETECT  = OF_DX2;
		OF_DY2_DETECT  = OF_DY2;
		OF_DX2FIX_DETECT = OF_DX2FIX;
		OF_DY2FIX_DETECT = OF_DY2FIX;
	}
	else if( OF_QUA > 50 )
	{
		//较大程度采纳
		OF_DX2_DETECT  = ((float)OF_DX2) * 0.7f + OF_DX2_DETECT * 0.3f;
		OF_DY2_DETECT  = ((float)OF_DY2) * 0.7f + OF_DY2_DETECT * 0.3f;
		OF_DX2FIX_DETECT = ((float)OF_DX2FIX) * 0.7f + OF_DX2FIX_DETECT * 0.3f;
		OF_DY2FIX_DETECT = ((float)OF_DY2FIX) * 0.7f + OF_DY2FIX_DETECT * 0.3f;
	}
	else if( OF_QUA > 30)
	{
		//较小程度采纳一小部分
		OF_DX2_DETECT  = ((float)OF_DX2) * 0.3f + OF_DX2_DETECT * 0.7f;
		OF_DY2_DETECT  = ((float)OF_DY2) * 0.3f + OF_DY2_DETECT * 0.7f;
		OF_DX2FIX_DETECT = ((float)OF_DX2FIX) * 0.3f + OF_DX2FIX_DETECT * 0.7f;
		OF_DY2FIX_DETECT = ((float)OF_DY2FIX) * 0.3f + OF_DY2FIX_DETECT * 0.7f;
	}
	else
	{
		//可信度过低，不采纳数据
		
	}
	
	OF_DX2_DETECT = -OF_DX2_DETECT;
	OF_DX2FIX_DETECT = -OF_DX2FIX_DETECT;
	
	OF_DX2FIX_DETECT_LPF = flow_bias_lpf(OF_DX2FIX_DETECT,20000,1,OF_DX2FIX_DETECT_LPF);
	
	mydata.d1 = (s16)OF_DX2FIX_DETECT_LPF;	//height
	mydata.d2 = (s16)OF_DX2_DETECT;
	mydata.d3 = (s16)OF_DX2FIX;
	mydata.d4 = (s16)OF_DX2FIX_DETECT;
	mydata.d5 = (s16)OF_DY2FIX_DETECT_LPF;
	mydata.d6 = (s16)OF_DY2_DETECT;
	mydata.d7 = (s16)OF_DY2FIX;
	mydata.d8 = (s16)OF_DY2FIX_DETECT;
	
}


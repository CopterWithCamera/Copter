#include "include.h"
#include "ultrasonic.h"
#include "usart.h"


void Ultrasonic_Init()
{
	Uart5_Init(9600);			//串口5初始化，函数参数为波特率
	
	#if defined(USE_KS103)
	
	//配置电源滤波
	u8 temp[3];
	temp[0] = 0xe8;			//默认地址是0xe8
	temp[1] = 0x02;
	temp[2] = 0x73;			//DC-DC降压模块环境下用72或73（0x70-0x75，其中0x70对应供电稳定性最好、滤波等级最低的情况）
	Uart5_Send(temp ,3);
	
	Delay_ms(2000);			//延时2s，等待KS103超声波传感器设置生效
	
	#elif defined(USE_US100)

	#endif
}

s8 ultra_start_f;

void Ultra_Duty()
{
	u8 temp[3];

	ultra.h_dt = 0.05f; //50ms一次

	//发送测距指令
	#if defined(USE_KS103)
	//	KS103返回16位数据
	//	先返回高八位，后返回低八位
	
	//数据发送时序：
	//串口地址 0xe8
	//延时20 - 100us
	//寄存器 0x02
	//延时20 - 100us
	//探测指令 0xb4（5m）  0xbc（11m）
	
	//串口波特率是9600bps，每秒能发送1200字节，每字节用时0.83ms
	//若要完整发送ks103的控制指令，会耗时3ms，严重影响控制性能
	
	//测距最大消耗时间为87ms
	
	temp[0] = 0xe8;
	temp[1] = 0x02;
	temp[2] = 0xb0;			//0xb0  -- 5m -- 33ms    0xb4 -- 5m -- 78ms   0xbc -- 11m -- 78ms
							//为了保证采集周期不超过50ms，选择0xb0（不带温度补偿）的工作模式
	Uart5_Send(temp ,3);

	#elif defined(USE_US100)
		temp[0] = 0x55;
		Uart5_Send(temp ,1);
		//Usart1_Send(temp ,1);
	#endif

	ultra_start_f = 1;

	if(ultra.measure_ot_cnt<200) //200ms
	{
		ultra.measure_ot_cnt += ultra.h_dt *1000;
	}
	else
	{
		ultra.measure_ok = 0;//超时，复位
	}
}

u16 ultra_distance_old;

_height_st ultra;

//数据接收处理函数，在串口接收中断中自动调用
void Ultra_Get(u8 com_data)
{
	static u8 ultra_tmp;
	
	if( ultra_start_f == 1 )	//如果之前发送了测距命令，且返回了第一个数值（高八位）
	{
		ultra_tmp = com_data;
		ultra_start_f = 2;
	}
	else if( ultra_start_f == 2 )	//返回了第二个数值（低八位）
	{
		ultra.height =  ((ultra_tmp<<8) + com_data)/10;		//单位是cm（传入数据单位是mm，÷10后单位是cm）
		
		#if defined(USE_KS103)
		if(ultra.height < 180) // KS103在1.8m内数据稳定
		{
			ultra.relative_height = ultra.height;	//单位是cm
			ultra.measure_ok = 1;
		}
		#elif defined(USE_US100)
		if(ultra.height < 500) // US100在5m内可用
		{
			ultra.relative_height = ultra.height;	//单位是cm
			ultra.measure_ok = 1;
		}
		#endif
		else
		{
			ultra.measure_ok = 2; //数据超范围
		}
		
		ultra_start_f = 0;
	}
	ultra.measure_ot_cnt = 0; //清除超时计数（喂狗）
	
	ultra.h_delta = ultra.relative_height - ultra_distance_old;
	
	ultra_distance_old = ultra.relative_height;
	
}

/*
//	此版本代码会耗费接近3ms时间用于发送
		UART5->DR = 0xe8;   //ks103地址（可设置）
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
		
		UART5->DR = 0x02;   //++++
		while( (UART5->SR & USART_FLAG_TXE) == 0 );

		UART5->DR = 0xbc;  //70ms,带温度补偿
		while( (UART5->SR & USART_FLAG_TXE) == 0 );
*/

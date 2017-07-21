#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "stm32f4xx.h"
#include "height_ctrl.h"


typedef struct
{
    u8 msg_id;
    u8 msg_data;
    u8 send_check;
    u8 send_version;
    u8 send_status;
    u8 send_senser;
    u8 send_senser2;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_pid5;
    u8 send_pid6;
    u8 send_rcdata;
    u8 send_offset;
    u8 send_motopwm;
    u8 send_power;
    u8 send_user;
    u8 send_speed;
    u8 send_location;

} dt_flag_t;

//用户数据1发送内容结构体
typedef struct
{
	s16 d1;
	s16 d2;
	s16 d3;
	s16 d4;
	s16 d5;
	s16 d6;
	s16 d7;
	s16 d8;
	s16 d9;
	s16 d10;
	s16 d11;
	s16 d12;
}User_Data1;

extern dt_flag_t f;	//发送数据的标志位
extern User_Data1 mydata;	//用户数据1发送内容

void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(u8 data);


#endif


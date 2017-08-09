/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：main.c
 * 描述    ：主循环
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/
#include "include.h"

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
	while (1)
	{

	}
}
#endif
//=======================================================================================


//=======================================================================================
u8 Init_Finish = 0;
int main(void)
{
	Init_Finish = All_Init();		
	while(1)
	{
		Duty_Loop(); 
	}
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/


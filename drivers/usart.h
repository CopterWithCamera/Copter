#ifndef _USART_H
#define _USART_H

#include "stm32f4xx.h"
#include <stdio.h>

void Usart1_Init(u32 br_num);
void Usart1_IRQ(void);
void Usart1_Send(unsigned char *DataToSend ,u8 data_num);

void Usart2_Init(u32 br_num);
void Usart2_IRQ(void);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);

//void Uart4_Init(u32 br_num);
//void Uart4_IRQ(void);
//void Uart4_Send(unsigned char *DataToSend ,u8 data_num);

void Uart5_Init(u32 br_num);
void Uart5_IRQ(void);
void Uart5_Send(unsigned char *DataToSend ,u8 data_num);
#endif

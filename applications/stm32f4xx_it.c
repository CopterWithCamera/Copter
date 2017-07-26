#include "include.h"
#include "led.h"

void NMI_Handler(void)
{
}

//内存异常
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

//总线错误
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

//检测到未定义指令或在存取内存时有未对齐
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

//硬件异常中断
void HardFault_Handler(void)
{
  while (1)
  {
	//停转
	TIM1->CCR4 = 4000;				//1	
	TIM1->CCR3 = 4000;				//2
	TIM1->CCR2 = 4000;				//3	
	TIM1->CCR1 = 4000;				//4
	
	//灭所有灯
	LED1_OFF;
	LED2_OFF;
	LED3_OFF;
	LED4_OFF;
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

//============== 系统中断 ==================

void SysTick_Handler(void)
{
	sysTickUptime++;
	sys_time();
}

//============== 用户中断 ==================

void TIM3_IRQHandler(void)
{
	_TIM3_IRQHandler();

}

void TIM4_IRQHandler(void)
{
	_TIM4_IRQHandler();

}

void USART1_IRQHandler(void)
{
	Usart1_IRQ();
}

void USART2_IRQHandler(void)
{
	Usart2_IRQ();
}

void USART3_IRQHandler(void)
{
	Usart3_IRQ();
}

void UART5_IRQHandler(void)
{
	Uart5_IRQ();
}

#include "include.h"
#include "led.h"

void NMI_Handler(void)
{
}

//�ڴ��쳣
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

//���ߴ���
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

//��⵽δ����ָ����ڴ�ȡ�ڴ�ʱ��δ����
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

//Ӳ���쳣�ж�
void HardFault_Handler(void)
{
  while (1)
  {
	//ͣת
	TIM1->CCR4 = 4000;				//1	
	TIM1->CCR3 = 4000;				//2
	TIM1->CCR2 = 4000;				//3	
	TIM1->CCR1 = 4000;				//4
	
	//�����е�
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

//============== ϵͳ�ж� ==================

void SysTick_Handler(void)
{
	sysTickUptime++;
	sys_time();
}

//============== �û��ж� ==================

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

#include "stm32f10x.h"                  // Device header
void signal_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_b;
	GPIO_b.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_b.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
	GPIO_b.GPIO_Speed =GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_b);
}

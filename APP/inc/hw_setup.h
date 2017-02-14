#ifndef HW_SETUP_H_
#define HW_SETUP_H_

#include "stm32f10x_lib.h"

#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB

#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11

#define USART_DXL			    0
#define USART_ZIGBEE			1
#define USART_PC			    2

#define CCR1_VAL				100 // 1 ms

void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Configuration(u32);
void USART_Configuration(u8, u32);
void SysTick_Configuration(void);
void ADC_Configuration(void);
void Timer_Configuration(void);


#endif

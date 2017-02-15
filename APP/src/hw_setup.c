/************************* (C) COPYRIGHT 2010 ROBOTIS ***************************/
/* Modified by Anton Olsson / Cybercom */

#include "hw_setup.h"

#include "../inc/time.h"

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}

	/* Enable peripheral clocks --------------------------------------------------*/

	/* Enable USART1 and GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);

	/* Enable Timer2 and USART3 clocks */
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART3 | RCC_APB1Periph_UART5 , ENABLE);

	PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
		// Set the Vector Table base location at 0x20000000
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
		// Set the Vector Table base location at 0x08003000
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
	#endif

	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USART1 Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_ENABLE_TXD | PIN_ENABLE_RXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig( GPIO_Remap_USART1, ENABLE);
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_Disable, ENABLE);

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	/* Check the end of ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));


	/* Start ADC1 calibration */
	/* Check the end of ADC1 calibration */
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));


	/* Start ADC1 Software Conversion */
	//ADC_SoftwareStartConvCmd(ADC1, ENABLE);


	/* Init ADC2 with same config as ADC1 */
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 1 , ADC_SampleTime_239Cycles5); // TODO to understand: why ADC_Channel_5 here?
	ADC_Cmd(ADC2, ENABLE);

	ADC_ResetCalibration(ADC2);
	while(ADC_GetResetCalibrationStatus(ADC2));

	ADC_StartCalibration(ADC2);
	while(ADC_GetCalibrationStatus(ADC2));

	//ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}



void USART1_Configuration(u32 baudrate)
{
	USART_Configuration(USART_DXL, baudrate);
}

void USART_Configuration(u8 PORT, u32 baudrate)
{

	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	if( PORT == USART_DXL )
	{
		USART_DeInit(USART1);
		mDelay(10);
		/* Configure the USART1 */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART1 Receive and Transmit interrupts */
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		/* Enable the USART1 */
		USART_Cmd(USART1, ENABLE);
	}

	else if( PORT == USART_PC )
	{

		USART_DeInit(USART3);
		mDelay(10);
		/* Configure the USART3 */
		USART_Init(USART3, &USART_InitStructure);

		/* Enable USART3 Receive and Transmit interrupts */
		//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		//USART_ITConfig(USART3, USART_IT_TC, ENABLE);

		/* Enable the USART3 */
		USART_Cmd(USART3, ENABLE);
	}

	else if (PORT == USART_ZIGBEE) {
		USART_DeInit(UART5);
		mDelay(10);
		/* Configure the UART5 */
		USART_Init(UART5, &USART_InitStructure);

		/* Enable UART5 Receive and Transmit interrupts */
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

		/* Enable the UART5 */
		USART_Cmd(UART5, ENABLE);

	}
}

void SysTick_Configuration(void)
{
	/* Disable SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Disable);
	/* Clear SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Clear);

	/* SysTick end of count event each 1us with input clock equal to 9MHz (HCLK/8, default) */
	SysTick_SetReload(9000);

	/* Enable SysTick interrupt */
	SysTick_ITConfig(ENABLE);

	/* Enable the SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Enable);
}

void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);

	TIM_DeInit(TIM2);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = CCR1_VAL ;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}



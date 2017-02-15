/************************* (C) COPYRIGHT 2010 ROBOTIS ***************************/
/* Modified by Anton Olsson / Cybercom */

/* Modified by Anton Olsson / Cybercom */

#include "hw_functions.h"
#include "hw_setup.h"
#include "rc100.h"

/* Private variables ---------------------------------------------------------*/
volatile byte                   gbpRxInterruptBuffer[256]; // dxl buffer
volatile byte                   gbRxBufferWritePointer,gbRxBufferReadPointer;
volatile vu32                   gwTimingDelay,gw1msCounter;
volatile u16                    capture;
u32                             Baudrate_DXL = 	1000000;
u32                             Baudrate_PC = 57600;
// vu16                            CCR1_Val = 100; 		// 1ms

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void TimerInterrupt_1ms(void) //OLLO CONTROL
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) // 1ms//
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + CCR1_VAL);

		if(gw1msCounter > 0)
			gw1msCounter--;
	}
}

void DisableUSART1(void)
{
	USART_Cmd(USART1, DISABLE);
}

void ClearBuffer256(void)
{
	gbRxBufferReadPointer = gbRxBufferWritePointer = 0;
}

byte CheckNewArrive(void)
{
	if(gbRxBufferReadPointer != gbRxBufferWritePointer)
		return 1;
	else
		return 0;
}

void TxDByte_DXL(byte bTxdData)
{
	GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Disable
	GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Enable

	USART_SendData(USART1,bTxdData);
	while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
}


byte RxDByte_DXL(void)
{
	byte bTemp;

	while(1)
	{
		if(gbRxBufferReadPointer != gbRxBufferWritePointer) break;
	}

	bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
	gbRxBufferReadPointer++;

	return bTemp;
}


void TxDString(byte *bData)
{
	while (*bData)
		TxDByte_PC(*bData++);
}

void TxDWord16(word wSentData)
{
	TxDByte16((wSentData >> 8) & 0xff);
	TxDByte16(wSentData & 0xff);
}

void TxDByte16(byte bSentData)
{
	byte bTmp;

	bTmp = ((byte) (bSentData >> 4) & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
	bTmp = (byte) (bSentData & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
}



void TxDByte_PC(byte bTxdData)
{
	USART_SendData(USART3,bTxdData);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
}

/*__interrupt*/
void RxD0Interrupt(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	    gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}




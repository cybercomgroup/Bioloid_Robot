/*
 * hw_functions.h
 *
 *  Created on: 7 feb. 2017
 *      Author: install
 */

#ifndef APP_INC_HW_FUNCTIONS_H_
#define APP_INC_HW_FUNCTIONS_H_

#include "typedefs.h"

#define word                    u16
#define byte                    u8

void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void __ISR_DELAY(void);
void DisableUSART1(void);
void ClearBuffer256(void);
byte CheckNewArrive(void);
void TxDByte_DXL(byte);
byte RxDByte_DXL(void);
void TxDString(byte*);
void TxDWord16(word);
void TxDByte16(byte);
void TxDByte_PC(byte);
void StartDiscount(s32);
byte CheckTimeOut(void);

#endif /* APP_INC_HW_FUNCTIONS_H_ */

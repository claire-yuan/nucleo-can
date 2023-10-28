/*
 * can.h
 *
 *  Created on: Oct 23, 2023
 *      Author: yuanc
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Global variables ------------------------------------------------------------------*/
extern uint8_t RxData[8];
extern volatile uint8_t CAN_received;

/* Functions */
void MX_CAN_Init(void);
void CAN_set_TxHeader(uint32_t IDE, uint32_t RTR, uint32_t StdId);
uint8_t transmit_CAN_message(uint8_t TxData[8], uint32_t DLC);

#endif /* INC_CAN_H_ */

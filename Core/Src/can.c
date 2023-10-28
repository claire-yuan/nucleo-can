/*
 * can.c
 *
 *  Created on: Oct 23, 2023
 *      Author: yuanc
 */



/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
#include "can.h"

CAN_HandleTypeDef hcan;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;


uint32_t TxMailbox;


uint8_t count = 0;

// Global variables
uint8_t RxData[8];
volatile uint8_t CAN_received;


void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  // CONFIGURE THE CAN FILTER
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh=0x0000;
  canfilterconfig.FilterIdLow=0x0000;
  canfilterconfig.FilterMaskIdHigh=0x0000;
  canfilterconfig.FilterMaskIdLow=0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;

  if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig) != HAL_OK) {
    		Error_Handler();
  }

  // START CAN

  	if (HAL_CAN_Start(&hcan) != HAL_OK) {
  		Error_Handler();
  	}

  	// Initialize CAN Bus Rx Interrupt
  	CAN_received = 0;
  	HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN_Init 2 */

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	CAN_received = 1;
}

void CAN_set_TxHeader(uint32_t IDE, uint32_t RTR, uint32_t StdId) {
	TxHeader.DLC = 0;
	TxHeader.IDE = IDE;
	TxHeader.RTR = RTR;
	TxHeader.StdId = StdId;
	TxHeader.TransmitGlobalTime = DISABLE;
}

/*
 * Returns: error code
 */
uint8_t transmit_CAN_message(uint8_t TxData[8], uint32_t DLC) {
	TxHeader.DLC = DLC;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);

	/* DEBUGGING ERRORS
	https://stackoverflow.com/questions/61376402/stm32-can-loop-back-mode*/

	//waiting for message to leave
	while(HAL_CAN_IsTxMessagePending(&hcan, TxMailbox));

	//waiting for transmission request to be completed by checking RQCPx
	while( !(hcan.Instance->TSR & ( 0x1 << (7 * ( TxMailbox - 1 )))));

	//checking if there is an error at TERRx, may be done with TXOKx as well (i think)
	if ((hcan.Instance->TSR & ( 0x8 << (7 * ( TxMailbox - 1 ))))){
	  //error is described in ESR at LEC last error code
	  return ( hcan.Instance->ESR & 0x70 ) >> 4;
	  //000: No Error
	  //001: Stuff Error
	  //010: Form Error
	  //011: Acknowledgment Error
	  //100: Bit recessive Error
	  //101: Bit dominant Error
	  //110: CRC Error
	  //111: Set by software
	}
}

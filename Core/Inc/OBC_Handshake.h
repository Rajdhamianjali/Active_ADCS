/*
 * OBC_Handshake.h
 *
 *  Created on: Sep 6, 2024
 *      Author: Dell
 */

#ifndef INC_OBC_HANDSHAKE_H_
#define INC_OBC_HANDSHAKE_H_

#include "main.h"
#include "ADCS_Debug.h"

extern UART_HandleTypeDef huart3;

void WAIT_FOR_HANDSHAKE();

#endif /* INC_OBC_HANDSHAKE_H_ */

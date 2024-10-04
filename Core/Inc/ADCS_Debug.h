/*
 * ADCS_Debug.h
 *
 *  Created on: Sep 6, 2024
 *      Author: Dell
 */

#ifndef INC_ADCS_DEBUG_H_
#define INC_ADCS_DEBUG_H_

#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart1;

int bufferSize(char *buff);
void myDebug(const char *fmt, ...);

#endif /* INC_ADCS_DEBUG_H_ */

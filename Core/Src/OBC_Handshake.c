/*
 * OBC_Handshake.c
 *
 *  Created on: Sep 6, 2024
 *      Author: Dell
 */

#include "OBC_Handshake.h"

#define ACK_HEAD	(0x53)
#define ACK_TAIL	(0x7E)

#define ADCS_HEAD_1		(0x0a)
#define ADCS_HEAD_2		(0x0d)
#define ADCS_HEAD_3		(0x0c)

#define ADCS_MODE_1		(0x01) //determination
#define ADCS_MODE_2		(0x02)//detumbling

#define ACK_LENGTH	(7)
#define CMD_LENGTH	(7)

extern uint8_t OBC_HANDSHAKE_FLAG;
uint8_t MainCMDHs[ACK_LENGTH];
uint8_t OBC_CMD[CMD_LENGTH];

int opera_mode = 0;
extern int mode;

void WAIT_FOR_HANDSHAKE() {

	memset(MainCMDHs, '\0', ACK_LENGTH);
	OBC_HANDSHAKE_FLAG = 0;
	if (HAL_UART_Receive(&huart3, MainCMDHs, ACK_LENGTH, 7000) == HAL_OK) {
		myDebug("--> Handshake command received from OBC: 0x%x\r\n");
		for (int i = 0; i < (ACK_LENGTH); i++) {
			myDebug("%02x ", MainCMDHs[i]);
		}
		myDebug("\n");

		uint8_t header = 0x00;

		if (MainCMDHs[0] == header) {

			for (int loop1 = 0; loop1 < sizeof(MainCMDHs); loop1++) {
				MainCMDHs[loop1] = MainCMDHs[loop1 + 1];
			}
		}

		if (MainCMDHs[0] == ACK_HEAD && MainCMDHs[5] == ACK_TAIL) {
			myDebug("--> Command Acknowledged successful!\n");
			if (HAL_UART_Transmit(&huart3, MainCMDHs, ACK_LENGTH, 2000)
					== HAL_OK) {
				myDebug("--> Handshake ACK, re-transmit to OBC: \n");
				for (int i = 0; i < (ACK_LENGTH); i++) {
					myDebug("%02x ", MainCMDHs[i]);
				}
				myDebug("\n");
				OBC_HANDSHAKE_FLAG = 1;
				memset(MainCMDHs, '\0', ACK_LENGTH);
			}
		} else {
			myDebug("*** Unknown Handshake command received!\n");
			if (HAL_UART_Transmit(&huart3, MainCMDHs, ACK_LENGTH, 2000)
					== HAL_OK) {
				myDebug("--> Unknown Handshake ACK, re-transmit to OBC.\n");
				for (int i = 0; i < (ACK_LENGTH); i++) {
					myDebug("%02x ", MainCMDHs[i]);
				}
				myDebug("\n");
				memset(MainCMDHs, '\0', ACK_LENGTH);
				OBC_HANDSHAKE_FLAG = 0;
				WAIT_FOR_HANDSHAKE();
			}
		}
	} else {
		OBC_HANDSHAKE_FLAG = 0;
		myDebug("*** Handshake Command receive failed, try again!\n");
		memset(MainCMDHs, '\0', ACK_LENGTH);
		WAIT_FOR_HANDSHAKE();
	}
}

int GET_COMMAND_OBC() {

	memset(OBC_CMD, '\0', CMD_LENGTH);

	if (HAL_UART_Receive(&huart3, OBC_CMD, CMD_LENGTH, 7000) == HAL_OK) {
		myDebug("--> CMD command received from OBC: 0x%x\r\n");
		for (int i = 0; i < (CMD_LENGTH); i++) {
			myDebug("%02x ", OBC_CMD[i]);
		}
		myDebug("\n");

		uint8_t header = 0x00;

		if (OBC_CMD[0] == header) {

			for (int loop1 = 0; loop1 < sizeof(OBC_CMD); loop1++) {
				OBC_CMD[loop1] = OBC_CMD[loop1 + 1];
			}
		}

		if (OBC_CMD[0] == ACK_HEAD && OBC_CMD[5] == ACK_TAIL) {

			if (OBC_CMD[1] == ADCS_HEAD_1 && OBC_CMD[2] == ADCS_HEAD_2
					&& OBC_CMD[3] == ADCS_HEAD_3) {

				myDebug("--> Command Acknowledged successful!\n");
				switch (OBC_CMD[4]) {

				case ADCS_MODE_1:

					opera_mode = 1;

					mode = 1;

					if (HAL_UART_Transmit(&huart3, OBC_CMD, CMD_LENGTH, 2000)
							== HAL_OK) {
						myDebug("--> ADCS_MODE 1 Executing: \n");
						for (int i = 0; i < (CMD_LENGTH); i++) {
							myDebug("%02x ", OBC_CMD[i]);
						}
						myDebug("\n");
						memset(OBC_CMD, '\0', CMD_LENGTH);

						return opera_mode;
					}

					break;

				case ADCS_MODE_2:
					opera_mode = 2;

					mode = 2;

					if (HAL_UART_Transmit(&huart3, OBC_CMD, CMD_LENGTH, 2000)
							== HAL_OK) {
						myDebug("--> ADCS_MODE 2 Executing: \n");
						for (int i = 0; i < (CMD_LENGTH); i++) {
							myDebug("%02x ", OBC_CMD[i]);
						}
						myDebug("\n");
						memset(OBC_CMD, '\0', CMD_LENGTH);

						return opera_mode;
					}
					break;
				default:
					opera_mode = 1;
					mode = 1;
					if (HAL_UART_Transmit(&huart3, OBC_CMD, CMD_LENGTH, 2000)
							== HAL_OK) {
						myDebug("--> ADCS_MODE 1 Executing: \n");
						for (int i = 0; i < (CMD_LENGTH); i++) {
							myDebug("%02x ", OBC_CMD[i]);
						}
						myDebug("\n");
						memset(OBC_CMD, '\0', CMD_LENGTH);

						return opera_mode;
					}

				}
			}
		} else {
			myDebug("*** Unknown CMD command received!\n");
			if (HAL_UART_Transmit(&huart3, OBC_CMD, CMD_LENGTH, 2000)
					== HAL_OK) {
				myDebug("--> Unknown CMD, re-transmit to OBC.\n");
				for (int i = 0; i < (CMD_LENGTH); i++) {
					myDebug("%02x ", OBC_CMD[i]);
				}
				myDebug("\n");
				memset(MainCMDHs, '\0', ACK_LENGTH);
				opera_mode = 0;
				mode = 0;
				GET_COMMAND_OBC();
			}
		}
	} else {
		myDebug("*** CMD Command receive failed, try again!\n");
		memset(OBC_CMD, '\0', CMD_LENGTH);
		opera_mode = 0;
		mode = 0;
		GET_COMMAND_OBC();
	}

	return opera_mode;
}

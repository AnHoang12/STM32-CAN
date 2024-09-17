#include "main.h"

void Setup_CAN1() {
	CAN1_DATA_TX[0] = NumBytesReq;   // data length
	CAN1_DATA_TX[1] = REQ_BUFFER[0]; // sub-function
	CAN1_DATA_TX[2] = REQ_BUFFER[1];
	CAN1_DATA_TX[3] = REQ_BUFFER[2];
	CAN1_DATA_TX[4] = REQ_BUFFER[3];
	CAN1_DATA_TX[5] = REQ_BUFFER[4];
	CAN1_DATA_TX[6] = REQ_BUFFER[5];
	CAN1_DATA_TX[7] = REQ_BUFFER[6];
	PadCANData(CAN1_DATA_TX, NumBytesReq+1);
	HAL_CAN_AddTxMessage(&hcan1, &CAN1_pHeader, CAN1_DATA_TX, &CAN1_pTxMailbox);
	PrintCANLog(CAN1_pHeader.StdId, &CAN1_DATA_TX[0]);
	HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &CAN2_pHeaderRx, CAN2_DATA_RX);
}

void NRC_33()	//Security access denied
{
	CAN2_DATA_TX[0] = 0x03;
	CAN2_DATA_TX[1] = 0x7F;
	CAN2_DATA_TX[2] = 0x2E;
	CAN2_DATA_TX[3] = 0x33;
	PadCANData(CAN2_DATA_TX, 4);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
	PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
}

void NRC_31() 	//DID not support
{
	CAN2_DATA_TX[0] = 0x03;
	CAN2_DATA_TX[1] = 0x7F;
	CAN2_DATA_TX[2] = REQ_BUFFER[0];
	CAN2_DATA_TX[3] = 0x31;
	PadCANData(CAN2_DATA_TX, 4);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
	PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
}

void NRC_35()	//invalid keys
{
	CAN2_DATA_TX[0] = 0x03;
	CAN2_DATA_TX[1] = 0x7F;
	CAN2_DATA_TX[2] = 0x27;
	CAN2_DATA_TX[3] = 0x35;
	PadCANData(CAN2_DATA_TX, 4);
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
	PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
}

void NRC_13() { // invalid message length
	CAN2_DATA_TX[0] = 0x03;
	CAN2_DATA_TX[1] = 0x7F;
	CAN2_DATA_TX[2] = REQ_BUFFER[0];
	CAN2_DATA_TX[3] = 0x13;
	PadCANData(CAN2_DATA_TX, 4);

	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
	PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
}

void SID_27_01_Practice() {
	Setup_CAN1();
	CAN2_DATA_TX[0] = 0x06;
	CAN2_DATA_TX[1] = 0x67;
	CAN2_DATA_TX[2] = 0x01;
	//Random seed
	CAN2_DATA_TX[3] = 0x03;
	CAN2_DATA_TX[4] = 0x05;
	CAN2_DATA_TX[5] = 0x07;
	CAN2_DATA_TX[6] = 0x09;
	PadCANData(CAN2_DATA_TX, 7);

	HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX, &CAN2_pTxMailbox);
	PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx, CAN1_DATA_RX);
	SID_27_01_OK = 1;
}

void SID_27_02_Practice() {
	Setup_CAN1();
	// Extract SEED values
	uint8_t SEED_0 = 0x03;
	uint8_t SEED_1 = 0x05;
	uint8_t SEED_2 = 0x07;
	uint8_t SEED_3 = 0x09;

	// Calculate KEY values // key: 06 0C 0E 0C
	uint8_t KEY_0 = SEED_0 ^ SEED_1;
	uint8_t KEY_1 = SEED_1 + SEED_2;
	uint8_t KEY_2 = SEED_2 ^ SEED_3;
	uint8_t KEY_3 = SEED_3 + SEED_0;

	// Check if the received key matches the calculated key
	if (CAN2_DATA_RX[3] == KEY_0 && CAN2_DATA_RX[4] == KEY_1 &&
		CAN2_DATA_RX[5] == KEY_2 && CAN2_DATA_RX[6] == KEY_3) {
		CAN2_DATA_TX[0] = 0x02;
		CAN2_DATA_TX[1] = 0x67;
		CAN2_DATA_TX[2] = 0x02;
		PadCANData(CAN2_DATA_TX, 3);

		HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX,
				&CAN2_pTxMailbox);
		PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx,
				CAN1_DATA_RX);

		ECU_Access_OK = 1;
		SID_27_01_OK = 0;

		//Turn on PB0 LED
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	} else {
		NRC_35();
		SID_27_01_OK = 0;
	}
}

void SID_2E_Practice() {
	Setup_CAN1();
	if (ECU_Access_OK == 1) {
		if (CAN2_DATA_RX[2] == 0x01 && CAN2_DATA_RX[3] == 0x23) {
			if (NumBytesReq == 7) {
				//Update new CANID
				new_CANID = (REQ_BUFFER[5] << 8) | REQ_BUFFER[6];
				//positive Response
				CAN2_DATA_TX[0] = 0x01;	//data length
				CAN2_DATA_TX[1] = 0x6E;

				HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX,
						&CAN2_pTxMailbox);
				PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
				HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx,
						CAN1_DATA_RX);
			} else {
				NRC_13();
			}
		} else {
			if (NumBytesReq == 7) {
				NRC_31();
			} else {
				NRC_13();
			}
		}
	} else {
		NRC_33();
	}
}

void SID_22_Practice() {
	Setup_CAN1();
	if (CAN2_DATA_RX[2] == 0x01 && CAN2_DATA_RX[3] == 0x23) {
		if (NumBytesReq == 3) {
			//positive Response
			CAN2_DATA_TX[0] = 0x07;
			CAN2_DATA_TX[1] = 0x62;
			CAN2_DATA_TX[2] = CAN2_DATA_RX[2];
			CAN2_DATA_TX[3] = CAN2_DATA_RX[3];
			CAN2_DATA_TX[4] = 0x55;
			CAN2_DATA_TX[5] = 0x55;

			CAN2_DATA_TX[6] = CAN1_pHeader.StdId >> 8;
			CAN2_DATA_TX[7] = CAN1_pHeader.StdId & (0b11111111);

			HAL_CAN_AddTxMessage(&hcan2, &CAN2_pHeader, CAN2_DATA_TX,
					&CAN2_pTxMailbox);
			PrintCANLog(CAN2_pHeader.StdId, &CAN2_DATA_TX[0]);
			HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &CAN1_pHeaderRx,
					CAN1_DATA_RX);
		} else {
			NRC_13();
		}
	} else {
		if (NumBytesReq == 3) {
			NRC_31();
		} else {
			NRC_13();
		}
	}
}

void PadCANData(uint8_t *data, uint8_t dataLength) {
    for (uint8_t i = dataLength; i < 8; i++) {
        data[i] = 0x55;
    }
}

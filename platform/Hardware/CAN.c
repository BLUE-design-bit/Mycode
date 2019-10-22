#include "CAN.h"
#include "PID.h"
extern CAN_HandleTypeDef hcan1;
extern moto_measure_t motor[3];
int16_t speed_set,speed_left,speed_right;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef CAN_RxHeader;
	uint8_t Rx_Data[8];
	
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN_RxHeader,Rx_Data);
	switch(CAN_RxHeader.StdId){
		case 0x201:
			{
				motor[0].real_current  = (uint16_t)(Rx_Data[2]<<8 | Rx_Data[3]);
				motor[0].speed_rpm = motor[0].real_current;
				speed_right = motor[0].real_current;
				break;
			}
		case 0x202:
			{
				motor[1].real_current  = (uint16_t)(Rx_Data[2]<<8 | Rx_Data[3]);
				motor[1].speed_rpm = motor[0].real_current;
				speed_left = motor[1].real_current;
				break;
			}
			case 0x203:
			{
				motor[2].angle  = (uint16_t)(Rx_Data[0]<<8 | Rx_Data[1]);
				motor[2].speed_rpm  = (uint16_t)(Rx_Data[2]<<8 | Rx_Data[3]);
				break;
			}
		default:
			break;
		}
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4){

	uint32_t   TxMailbox;
	CAN_TxHeaderTypeDef CAN_TxHeader;
	uint8_t Data[8];
	
	CAN_TxHeader.StdId = 0x200;
	CAN_TxHeader.IDE = CAN_ID_STD;
	CAN_TxHeader.RTR = CAN_RTR_DATA;
	CAN_TxHeader.DLC = 0x08;
	
	Data[0] = (iq1 >> 8);
	Data[1] = iq1;
	Data[2] = (iq2 >> 8);
	Data[3] = iq2;
	Data[4] = (iq3 >> 8);
	Data[5] = iq3;
	Data[6] = iq4 >> 8;
	Data[7] = iq4;
	
	HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeader,Data,&TxMailbox);
}

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	CAN_FilterTypeDef CAN_FilterType;
	CAN_FilterType.FilterBank=14;
	CAN_FilterType.FilterIdHigh=0x0000;
	CAN_FilterType.FilterIdLow=0x0000;
	CAN_FilterType.FilterMaskIdHigh=0x0000;
	CAN_FilterType.FilterMaskIdLow=0x0000;
	CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;
	CAN_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;
	CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;
	CAN_FilterType.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(_hcan,&CAN_FilterType);
	HAL_CAN_ActivateNotification(_hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(_hcan);
}



#include "RemoteCtrl.h"
#include "Rc.h"
#include "string.h"

RemoteCtrlStruct ReCtrl;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern volatile uint8_t rx_len;     //额外
extern volatile uint8_t recv_end_flag;  		//额外
extern uint8_t rx_buffer[100];   //额外
#ifndef BUFFER_SIZE
#define BUFFER_SIZE  100	//额外
#endif



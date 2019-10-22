#ifndef __RC_H
#define __RC_H


#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
void Rc_RxCallback(UART_HandleTypeDef *huart);
	
#ifdef __cplusplus
};  
#endif 
#endif



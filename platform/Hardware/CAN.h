#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx_hal.h"
void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
#endif

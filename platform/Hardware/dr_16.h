#ifndef _DR_16_H_ // 意思是：宏开始行，如果还没有定义 _HEADER_One_H_ 则 进入，否则退出
#define _DR_16_H_ //定义 _HEADER_One_H_//
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
 //***********************************dr_16******************************
 typedef struct				 //各个通道的摇杆量
 {
  uint32_t ch0;
  uint32_t ch1;
  uint32_t ch2;
  uint32_t ch3;
  uint32_t s1;
  uint32_t s2;
 }
 dr_16;
 extern dr_16 dr_control;
void  data_config(dr_16 *RC_Ctl,uint8_t * sbus_rx_buffer);

#ifdef __cplusplus
 }
#endif
#endif // 宏结束行

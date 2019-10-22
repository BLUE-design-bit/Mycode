#include <dr_16.h>
dr_16 dr_control;
void  data_config(dr_16 *RC_Ctl,uint8_t * sbus_rx_buffer)
{
 	RC_Ctl->ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;           //!< Channel 0
 	RC_Ctl->ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;   //!< Channel 1
 	RC_Ctl->ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff; //!< Channel 2
 	RC_Ctl->ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;   //!< Channel 3
 	RC_Ctl->s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                          //!< Switch left
 	RC_Ctl->s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);                               //!< Switch right

}

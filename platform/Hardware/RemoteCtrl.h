#ifndef __REMOTECTRL_H
#define __REMOTECTRL_H

#ifdef __cplusplus
extern "C" {
#endif
	
#ifdef __cplusplus
};  
#endif 
#include "stm32f4xx_hal.h"
	
#ifdef __cplusplus


#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)2)
#define RC_SW_DOWN ((uint16_t)3)

#define KEYBOARD_PRESSED_W ((uint16_t)0x01<<0)
#define KEYBOARD_PRESSED_S ((uint16_t)0x01<<1)
#define KEYBOARD_PRESSED_A ((uint16_t)0x01<<2)
#define KEYBOARD_PRESSED_D ((uint16_t)0x01<<3)
#define KEYBOARD_PRESSED_Q ((uint16_t)0x01<<4)
#define KEYBOARD_PRESSED_E ((uint16_t)0x01<<5)
#define KEYBOARD_PRESSED_SHIFT ((uint16_t)0x01<<6)
#define KEYBOARD_PRESSED_CTRL ((uint16_t)0x01<<7)

#define PRESS_LEFT 0
#define PRESS_RIGHT 1


typedef struct 
	{ 
		struct 
		{
			uint16_t ch0,ch1,ch2,ch3,s1,s2;
		}rc;
		struct 
		{
			int16_t x,y,z;
			uint8_t press_l,press_r;
		}mouse;
		struct 
		{
			uint16_t val;
		}keyb;
	}RcTypeDef;
	
class RemoteCtrlStruct
{
private:
	RcTypeDef Rc_Ctl;
public:
	void RcDataHandle(uint8_t *pData);
	uint16_t show_channel(uint16_t channel);
	uint16_t show_skey(uint16_t s);
	int16_t show_mouse_where( int16_t where);
	uint8_t show_mouse_press(uint8_t press);
	uint8_t show_key(void);
};

#endif 
#endif


#include "RemoteCtrl.h"




void RemoteCtrlStruct::RcDataHandle(uint8_t *pData)
{
	if(pData == NULL)
    {
        return;
    }
		Rc_Ctl.rc.ch0 = (pData[0] | pData[1] << 8) & 0x07ff;
    Rc_Ctl.rc.ch1 = (pData[1] >> 3 |pData[2] << 5) & 0x07ff;
		Rc_Ctl.rc.ch2 = (pData[2] >> 6 |pData[3] << 2 | pData[4] << 10) & 0x07ff;
    Rc_Ctl.rc.ch3 = (pData[4] >> 1 |pData[5] << 7) & 0x07ff;
		Rc_Ctl.rc.s1 = ((pData[5] >> 4) &0x000C) >> 2;
		Rc_Ctl.rc.s2 = ((pData[5] >> 4) &0x0003);
		Rc_Ctl.mouse.x = (pData[6] |pData[7] << 8);
		Rc_Ctl.mouse.y = (pData[8] | pData[9] << 8);
		Rc_Ctl.mouse.z   = (pData[10] | pData[11] << 8);
		Rc_Ctl.mouse.press_l = pData[12];
		Rc_Ctl.mouse.press_r = pData[13];
		Rc_Ctl.keyb.val = (pData[14]) |(pData[15] << 8);
}
uint16_t RemoteCtrlStruct::show_channel(uint16_t channel)
{
	switch(channel)
	{
		case 0:
			return Rc_Ctl.rc.ch0;
			break;
		case 1:
			return  Rc_Ctl.rc.ch1;
			break;
		case 2:
			return Rc_Ctl.rc.ch2;
			break;
		case 3:
			return  Rc_Ctl.rc.ch3;
			break;
	}
	return 0;
}
uint16_t RemoteCtrlStruct::show_skey(uint16_t s)
{
	switch(s)
	{
		case 1:
			return Rc_Ctl.rc.s1;
			break;
		case 2:
			return  Rc_Ctl.rc.s2;
			break;
	}
	return 0;
}
int16_t RemoteCtrlStruct::show_mouse_where( int16_t where)
{
	switch(where)
	{
		case 'x':
			return Rc_Ctl.mouse.x ;
			break;
		case 'y':
			return  Rc_Ctl.mouse.y;
			break;
		case 'z':
			return Rc_Ctl.mouse.z;
			break;
	}
	return 0;
}
uint8_t RemoteCtrlStruct::show_mouse_press(uint8_t press)
{
	switch(press)
	{
		case PRESS_LEFT:
			return Rc_Ctl.mouse.press_l;
			break;
		case PRESS_RIGHT:
			return  Rc_Ctl.mouse.press_r;
			break;
	}
	return 0;
}
uint8_t RemoteCtrlStruct::show_key(void)
{
	
	return (Rc_Ctl.keyb.val);

}


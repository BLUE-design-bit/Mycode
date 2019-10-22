#ifndef __PID_H
#define __PID_H
#endif

#include "stm32f4xx_hal.h"

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//???,??NOW, LAST, LLAST???
    float get[3];				//???
    float err[3];				//??
	
    
    float pout;							//p??
    float iout;							//i??
    float dout;							//d??
    
    float pos_out;						//???????
    float last_pos_out;				//????
    float delta_u;						//?????
    float delta_out;					//??????? = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;			
    uint32_t IntegralLimit;		
}pid_t;
void pid_param_init(
    pid_t *pid, 
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd);
		
float pid_calc(pid_t* pid, float get, float set);
		
		typedef struct{
	int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	uint8_t			buf_idx;
//	uint16_t			angle_buf[5];
	uint16_t			fited_angle;
	uint32_t			msg_cnt;
}moto_measure_t;
void get_total_angle(moto_measure_t *p);

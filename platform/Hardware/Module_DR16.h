/**
  ******************************************************************************
  * @file Module_DR16.h
  * @author YuyongHu (867115311@qq.com)
  * @brief  DR16抽象模块，于头文件处声明且源文件处实例化了一个DR16对象,所以不必再实例
  * 化新对象，只需include本头文件即可使用DR16对象。
  * @version 2.0.2 beta
  * @date   2019-4-7
  * @editby 
	

	
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have many 
  * bugs, update the version NO., write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding 
  * through your new brief.
  * 
  * 由于本文件使用了变长参数模板(C++11新功能)，在使用本文件时应当注意，用keil编译时需
  * 在Options for Target 'balabala'(魔术棒)->C/C++->Misc Control处加上--cpp11
  ******************************************************************************
**/


#ifndef _MODULE_DR16_H_
#define _MODULE_DR16_H_

#include <stdint.h>


//提前声明一下类型存在
class DR16_Classdef;
//可引用的全局变量
extern DR16_Classdef DR16;


/********自定义区**********/
#define Ignore_Limit 0.05 //手柄或鼠标的归一化后的绝对值小于此值时自动视为0
/*End Of 自定义区*********/
//键位宏定义
#define _W					0
#define _S					1
#define _A					2
#define _D					3
#define _SHIFT              4
#define _CTRL               5
#define _Q					6
#define _E					7
#define _R					8
#define _F					9
#define _G					10
#define _Z					11
#define _X					12
#define _C					13
#define _V					14
#define _B					15
#define _Mouse_L            16
#define _Mouse_R            17


//DR16数据包内容
__packed struct DR16_DataPack_Typedef
{
	uint64_t ch0:11;
	uint64_t ch1:11;
	uint64_t ch2:11;
	uint64_t ch3:11;
	uint64_t s2:2;
	uint64_t s1:2;
	int64_t x:16;
	int64_t y:16;
	int64_t z:16;
	uint64_t press_l:8;
	uint64_t press_r:8;
	uint64_t key:16;
};

//手柄上面两挡位开关状态
enum SW_Status_Typedef
{
    NONE = 0,
    UP = 1,
    MID = 3,
    DOWN = 2,
};

//按键类型定义
struct Key_Typedef
{
    bool Pressed;   //是否按下
    bool Triggered; //是否触发过函数，用来执行点击事件
};


#ifndef __ENUM_ONlINESTATUS_TYPEDEF__
#define __ENUM_ONlINESTATUS_TYPEDEF__
//在线状态定义
enum OnlineStatus_Typedef
{
    Offline = 0x00,
    Online = 0x01,
};

#endif

//DR16类型
class DR16_Classdef
{
    private:
        OnlineStatus_Typedef Status;  //在线状态
        DR16_DataPack_Typedef DataPack; //数据包
        
        //两个摇杆四个方向与鼠标三个方向速度归一化后的值
        float RX_Norm,RY_Norm,LX_Norm,LY_Norm,MouseX_Norm,MouseY_Norm,MouseZ_Norm; 
        Key_Typedef Key[18];  //16个键的相关信息
        float MouseCoefficient;  //鼠标动作乘的系数
        void Key_Process(void); //按键处理
    public:
        DR16_Classdef();

        void DataCapture(DR16_DataPack_Typedef* captureData);  //抓取并更新数据包

        uint64_t GetCh0(void);
        uint64_t GetCh1(void);
        uint64_t GetCh2(void);
        uint64_t GetCh3(void);
        SW_Status_Typedef GetS2(void);
        SW_Status_Typedef GetS1(void);
        int64_t GetMouseX(void);
        int64_t GetMouseY(void);
        int64_t GetMouseZ(void);
        uint64_t GetPress_L(void);
        uint64_t GetPress_R(void);
        uint64_t Getkey(void);

        //归一化后的通道0123、鼠标XYZ值
        float Get_RX_Norm(void);
        float Get_RY_Norm(void);
        float Get_LX_Norm(void);
        float Get_LY_Norm(void);
        float Get_MouseX_Norm(void);
        float Get_MouseY_Norm(void);
        float Get_MouseZ_Norm(void);

        //用于判断某个按键是否按下
        bool IsKeyPress(int _key);

        /*状态相关操作*/
        void SetStatus(OnlineStatus_Typedef para_status);  //更新接受信号状态
        OnlineStatus_Typedef GetStatus(void);  //获得接受信号状态

        //按键触发，按一下触发一次功能(提供四个重载版本，分别对应是/否调出返回值与类内/通用函数)
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {
                Key[_key].Triggered = true;
                func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {
                
                Key[_key].Triggered = true;
                (Obj.*func)(arg...);
                return true;
            }
            return false;
        }
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Ret_T& retv_recver,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {
                Key[_key].Triggered = true;
                retv_recver = func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Trigger(int _key,Ret_T& retv_recver, Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Triggered == false && Key[_key].Pressed)
            {
                
                Key[_key].Triggered = true;
                retv_recver = (Obj.*func)(arg...);
                return true;
            }
            return false;
        }

        
        //按键绑定，按住就执行功能(提供四个重载版本，分别对应是/否调出返回值与类内/通用函数)
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Pressed)
            {
                func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Pressed)
            {
                (Obj.*func)(arg...);
                return true;
            }
            return false;
        }
        template<typename Ret_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Ret_T& retv_recver,Ret_T(*func)(Arg_Ts...),Arg_Ts... args)
        {
            if(Key[_key].Pressed)
            {
                retv_recver = func(args...);
                return true;
            }
            return false;
        }
        template<typename Ret_T,typename Obj_T, typename... Arg_Ts>
        bool Func_Bind(int _key,Ret_T& retv_recver, Obj_T& Obj, Ret_T(Obj_T::*func)(Arg_Ts...),Arg_Ts... arg)
        {
            if(Key[_key].Pressed)
            {
                retv_recver = (Obj.*func)(arg...);
                return true;
            }
            return false;
        }
};

float DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num);

#endif

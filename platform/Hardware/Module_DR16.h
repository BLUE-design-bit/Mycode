/**
  ******************************************************************************
  * @file Module_DR16.h
  * @author YuyongHu (867115311@qq.com)
  * @brief  DR16����ģ�飬��ͷ�ļ���������Դ�ļ���ʵ������һ��DR16����,���Բ�����ʵ��
  * ���¶���ֻ��include��ͷ�ļ�����ʹ��DR16����
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
  * ���ڱ��ļ�ʹ���˱䳤����ģ��(C++11�¹���)����ʹ�ñ��ļ�ʱӦ��ע�⣬��keil����ʱ��
  * ��Options for Target 'balabala'(ħ����)->C/C++->Misc Control������--cpp11
  ******************************************************************************
**/


#ifndef _MODULE_DR16_H_
#define _MODULE_DR16_H_

#include <stdint.h>


//��ǰ����һ�����ʹ���
class DR16_Classdef;
//�����õ�ȫ�ֱ���
extern DR16_Classdef DR16;


/********�Զ�����**********/
#define Ignore_Limit 0.05 //�ֱ������Ĺ�һ����ľ���ֵС�ڴ�ֵʱ�Զ���Ϊ0
/*End Of �Զ�����*********/
//��λ�궨��
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


//DR16���ݰ�����
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

//�ֱ���������λ����״̬
enum SW_Status_Typedef
{
    NONE = 0,
    UP = 1,
    MID = 3,
    DOWN = 2,
};

//�������Ͷ���
struct Key_Typedef
{
    bool Pressed;   //�Ƿ���
    bool Triggered; //�Ƿ񴥷�������������ִ�е���¼�
};


#ifndef __ENUM_ONlINESTATUS_TYPEDEF__
#define __ENUM_ONlINESTATUS_TYPEDEF__
//����״̬����
enum OnlineStatus_Typedef
{
    Offline = 0x00,
    Online = 0x01,
};

#endif

//DR16����
class DR16_Classdef
{
    private:
        OnlineStatus_Typedef Status;  //����״̬
        DR16_DataPack_Typedef DataPack; //���ݰ�
        
        //����ҡ���ĸ�������������������ٶȹ�һ�����ֵ
        float RX_Norm,RY_Norm,LX_Norm,LY_Norm,MouseX_Norm,MouseY_Norm,MouseZ_Norm; 
        Key_Typedef Key[18];  //16�����������Ϣ
        float MouseCoefficient;  //��궯���˵�ϵ��
        void Key_Process(void); //��������
    public:
        DR16_Classdef();

        void DataCapture(DR16_DataPack_Typedef* captureData);  //ץȡ���������ݰ�

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

        //��һ�����ͨ��0123�����XYZֵ
        float Get_RX_Norm(void);
        float Get_RY_Norm(void);
        float Get_LX_Norm(void);
        float Get_LY_Norm(void);
        float Get_MouseX_Norm(void);
        float Get_MouseY_Norm(void);
        float Get_MouseZ_Norm(void);

        //�����ж�ĳ�������Ƿ���
        bool IsKeyPress(int _key);

        /*״̬��ز���*/
        void SetStatus(OnlineStatus_Typedef para_status);  //���½����ź�״̬
        OnlineStatus_Typedef GetStatus(void);  //��ý����ź�״̬

        //������������һ�´���һ�ι���(�ṩ�ĸ����ذ汾���ֱ��Ӧ��/���������ֵ������/ͨ�ú���)
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

        
        //�����󶨣���ס��ִ�й���(�ṩ�ĸ����ذ汾���ֱ��Ӧ��/���������ֵ������/ͨ�ú���)
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

#include "Module_DR16.h"

//DR16ң�������ݵ�ȫ�ֱ��������ڴ���DR16�������Ϣ
DR16_Classdef DR16;


DR16_Classdef::DR16_Classdef()
{
    MouseCoefficient = 128;    //���ϵ����ʼ��
    Status = Offline;    //״̬��ʼ��
    for(short i=0;i<16;i++)  //��ֵ��ʼ��
    {
        Key[i].Pressed=false;
        Key[i].Triggered=false;
    }
    //ҡ��ֵ��ʼ��
    DataPack.ch0 = 1024;
    DataPack.ch1 = 1024;
    DataPack.ch2 = 1024;
    DataPack.ch3 = 1024;
}

/**
 * @brief  ��ȡ���ݰ���������DR16�������յ������ݰ���ֵ����DR16�ĳ�Ա����DataPack�����м��̰�����������
 * @param captureData:�յ������ݰ�ָ��
 * @retval void
 */
void DR16_Classdef::DataCapture(DR16_DataPack_Typedef* captureData)
{
    DataPack = *captureData;   //DR16���ݰ�����
    //�������Ӱ�ͨ��ֵ��һ������
    RX_Norm = DeadZone_Process((float)(DataPack.ch0-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    RY_Norm = DeadZone_Process((float)(DataPack.ch1-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    LX_Norm = DeadZone_Process((float)(DataPack.ch2-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);
    LY_Norm = DeadZone_Process((float)(DataPack.ch3-1024)/660.0f,-Ignore_Limit,Ignore_Limit,0);

    float temp;

    temp = MouseCoefficient*((float)DataPack.x)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseX_Norm = temp;
    
    temp = MouseCoefficient*((float)DataPack.y)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseY_Norm = temp;

    
    temp = MouseCoefficient*((float)DataPack.z)/32768.0f;
    temp=temp>1?1:temp;
    temp=temp<-1?-1:temp;
    MouseZ_Norm = temp;

    Key_Process();      //��������
}

/**
 * @brief �������� Key Process
 * @param void
 * @retval void
 */
void DR16_Classdef::Key_Process(void)
{
    for(short i=0;i<16;i++)
    {
        //��⵽��Ӧ�������¾���key�ṹ�������λ
        if(DataPack.key & (0x01<<i)) 
            Key[i].Pressed = true;
        else
        {
            Key[i].Pressed = false;
            Key[i].Triggered = false;
        }
    }
    //������Ҽ�����
    if(DataPack.press_l == 0x01)
        Key[_Mouse_L].Pressed = true;
    else
    {
        Key[_Mouse_L].Pressed = false;
        Key[_Mouse_L].Triggered = false;
    }
    if(DataPack.press_r == 0x01)
        Key[_Mouse_R].Pressed = true;
    else
    {
        Key[_Mouse_R].Pressed = false;
        Key[_Mouse_R].Triggered = false;
    }
}

/**
 * @brief  ����һ��Getxxx�������ܶ��ǻ�����ݰ��е�Get��������ݵ�ֵ��
           ���ֱ�ע�ͺö��Ͳ��ظ�ע����
 * @param void
 * @retval Get��������ݵ�ֵ
 */
uint64_t DR16_Classdef::GetCh0(void)
{
    return DataPack.ch0;
}

uint64_t DR16_Classdef::GetCh1(void)
{
    return DataPack.ch1;
}


uint64_t DR16_Classdef::GetCh2(void)
{
    return DataPack.ch2;
}


uint64_t DR16_Classdef::GetCh3(void)
{
    return DataPack.ch3;
}

SW_Status_Typedef DR16_Classdef::GetS2(void)
{
    return (SW_Status_Typedef)DataPack.s2;
}
SW_Status_Typedef DR16_Classdef::GetS1(void)
{
    return (SW_Status_Typedef)DataPack.s1;
}
int64_t DR16_Classdef::GetMouseX(void)
{
    return DataPack.x;
}
int64_t DR16_Classdef::GetMouseY(void)
{
    return DataPack.y;
}
int64_t DR16_Classdef::GetMouseZ(void)
{
    return DataPack.z;
}
uint64_t DR16_Classdef::GetPress_L(void)
{
    return DataPack.press_l;
}
uint64_t DR16_Classdef::GetPress_R(void)
{
    return DataPack.press_r;
}
uint64_t DR16_Classdef::Getkey(void)
{
    return DataPack.key;
}
/**
 * @brief  ��һ�����ͨ��0123�����XYZֵ(Left_X_Axis,Right_Y_Axis,balabala)
 * @param void
 * @retval -1~1֮���ͨ��ֵ
 */
float DR16_Classdef::Get_RX_Norm(void)
{
    return RX_Norm;
}
float DR16_Classdef::Get_RY_Norm(void)
{
    return RY_Norm;
}
float DR16_Classdef::Get_LX_Norm(void)
{
    return LX_Norm;
}
float DR16_Classdef::Get_LY_Norm(void)
{
    return LY_Norm;
}
float DR16_Classdef::Get_MouseX_Norm(void)
{
    return MouseX_Norm;
}
float DR16_Classdef::Get_MouseY_Norm(void)
{
    return MouseY_Norm;
}
float DR16_Classdef::Get_MouseZ_Norm(void)
{
    return MouseZ_Norm;
}

/**
 * @brief  �����ж�ĳ�������Ƿ���
 * @param _key ͷ�ļ��к궨���key������_w,_s��
 * @retval ����Ϊture��û����Ϊfalse
 */
bool DR16_Classdef::IsKeyPress(int _key)
{
    return Key[_key].Pressed;
}

/**
 * @brief  ����DR16��Ա����status��ֵ����������DR16���߻�������
 * @param para_status Ҫ���õ�״̬
 * @retval void
 */
void DR16_Classdef::SetStatus(OnlineStatus_Typedef para_status)
{
    Status = para_status;
}

/**
 * @brief  �õ�DR16��Ա����status��ֵ���������ж�DR16�Ƿ�����
 * @param void
 * @retval Offline DR16����   Online DR16����
 */
OnlineStatus_Typedef DR16_Classdef::GetStatus(void)
{
    return Status;
}


/**
 * @brief  ��������������������㸽����΢С���
* @param num:Ҫ�������; DZ_min,DZ_max:������Χ;DZ_num:����������ʱ���ص�ֵ
 * @retval �����Ľ��
 */
float DeadZone_Process(float num,float DZ_min, float DZ_max, float DZ_num)
{
    //�����������򷵻�����ֵ
    if(num<DZ_max&&num>DZ_min)
    {
        return DZ_num;
    }
    else
        return num;
}


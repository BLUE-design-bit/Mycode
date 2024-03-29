/**
  ******************************************************************************
  * @file   ：GCS_Debug.cpp
  * @brief  ：通过串口用配套的上位机来调整参数。
  * @date   ：2018年12月
  * @author ：华南理工大学机器人实验室（林亮洪）
  
  ==============================================================================
                    ##### How to use this Middlewares #####
  ==============================================================================

  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/ 
#include <GCS_Debug.h>
extern int16_t speed_set,speed_left,speed_right;
extern float total_angle_set;
extern uint8_t shoot_mode;
float debug_target,debug_Kp,debug_Ki,debug_Kd;
uint8_t debug_tune_type,debug_tune_obj;

/***********************上位机调参使用***********************/
#define Sent_Data_Num 9
uint8_t On_Off_flag;


union type_change Sent_data_type[Sent_Data_Num+2];              //传输数据共用体
//uint8_t USART0_Sent_Choose_Data[9]={0,0,0,0,0,0,0,0,0};   //串口选择发送的数据标志
uint8_t USART0_Sent_Choose_Data[9]={0,1,2,3,4,5,6,7,8};   //串口选择发送的数据标志
/* Functions ----------------------------------------------------------------*/
/**
* @brief  串口发送设置函数,用于设置DMA串口的数据
* @param  data:需要传输的数组指针
* @return None.
*/
void GCSDebug_Sent_Set(float *data)
{
  uint8_t j;
  Sent_data_type[0].change_u8[3]=0xfd;                          //发送数据头
  for(j=1;j<Sent_Data_Num+1;j++)                                //数据体
  {
    Sent_data_type[j].change_float=data[j-1];
  }
  Sent_data_type[Sent_Data_Num+1].change_u8[0]=Sent_Data_Num;   //数据尾
  Sent_data_type[Sent_Data_Num+1].change_u8[1]=0xfe;            //校验位
}

/**
* @brief  串口发送参数选择函数(要观看的曲线),用于选择需要传输的数据
* @param  data:需要传输的数组指针
* @return None.
*/
void GCSDebug_Sent_Choose(float * data)
{
	#define DEBUG
  uint8_t i;
  for(i=0;i<Sent_Data_Num;i++)
  {
    switch(USART0_Sent_Choose_Data[i])
    {
      //输入捕获
#ifdef DEBUG
      /* 电机输出 */
      case 0: data[i]= speed_set;
          break;
      case 1: data[i]= speed_left;
          break;
      case 2: data[i]= speed_right;
          break;
//      case 3: data[i]= 0x25f;
//          break;
//      case 4: data[i]= 0x25f;
//          break;
//      case 5: data[i]= 0x25f;
//          break;
//      /* 电机转速 */
//      case 6: data[i]= 0x25f;
//          break;
//      case 7: data[i]= 0x25f;
//          break;
//      case 8: data[i]= 0x25f;
			
			
			
//          break;
//      case 9: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpm[3];
//          break;
//      case 10: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpm[4];
//          break;
//      case 11: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpm[5];
//          break;
//      /* 目标转速*/
//      case 12: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpmOut[0];
//          break;
//      case 13: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpmOut[1];
//          break;
//      case 14: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpmOut[2];
//          break;
//      case 15: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpmOut[3];
//          break;
//      case 16: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpmOut[4];
//          break;
//      case 17: data[i]= Engineer_Master.Lunar_Chassis.wheel_rpmOut[5];
//          break;
//      /* 当前位姿 */
//      case 18: data[i]= Engineer_Master.Lunar_Chassis.Command_Pos.x;
//          break;
//      case 19: data[i]= Engineer_Master.Lunar_Chassis.Command_Pos.y;
//          break;
//      case 20: data[i]= Engineer_Master.Lunar_Chassis.Command_Pos.yaw;
//          break;
//      case 21: data[i]= Engineer_Master.Lunar_Chassis.Command_Pos.roll;
//          break;
//      case 22: data[i]= Engineer_Master.Lunar_Chassis.Command_Pos.pitch;
//          break;
#endif
      default:break;
    }
  }
}

/**
* @brief  上位机参数转变成浮点数函数
* @param  PARAMETER：指令数组指针，用于读取指令
* @return None.
*/
float PARAMETER_Change_float(uint8_t * PARAMETER)
{
  uint8_t i=0;
  union type_change Sent_data_temp;                       //传输数据共用体
  for(i=0;i<4;i++)
  {
    Sent_data_temp.change_u8[i]=PARAMETER[3-i];           //转换成共用体数据类型
  }
  return Sent_data_temp.change_float;                     //返回共用体转化后的数据
}

/**
* @brief  上位机参数修改函数（要调的参数）
* @param  PARAMETER：指令数组指针，用于读取指令
* @return None.
*/
void PARAMETER_MODIFICATION(uint8_t * PARAMETER)
{
  switch(PARAMETER[0])
  {
    /* 以下部分用于机器人控制器参数整定 */
    case 0x00: speed_set =PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x01: total_angle_set =PARAMETER_Change_float(PARAMETER+1);
          break;
		case 0x02: shoot_mode = PARAMETER_Change_float(PARAMETER+1);
					break;
    case 0x04: debug_Kp =PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x05: debug_Ki =PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x06: debug_Kd =PARAMETER_Change_float(PARAMETER+1);
          break;
    /* 以上部分用于机器人控制器参数整定 */
    default:break;
  }
}

/**
* @brief  上位机参数修改函数
* @param  PARAMETER： 指令数组指针，用于读取指令
* @return None.
*/
void MODE_MODIFICATION(uint8_t * PARAMETER)
{
  switch(PARAMETER[0])
  {
    case 0x00: USART0_Sent_Choose_Data[0]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x01: USART0_Sent_Choose_Data[1]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x02: USART0_Sent_Choose_Data[2]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x03: USART0_Sent_Choose_Data[3]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x04: USART0_Sent_Choose_Data[4]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x05: USART0_Sent_Choose_Data[5]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x06: USART0_Sent_Choose_Data[6]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x07: USART0_Sent_Choose_Data[7]=PARAMETER_Change_float(PARAMETER+1);
          break;
    case 0x08: USART0_Sent_Choose_Data[8]=PARAMETER_Change_float(PARAMETER+1);
          break;
    default:break;
  }
}

/**
* @brief  发送数据函数
* @param  None.
* @return None.
*/
void Sent_Contorl(UART_HandleTypeDef* huart_x)
{
  float temp[Sent_Data_Num];
  GCSDebug_Sent_Choose(temp);                            //选择要传输的数据
  GCSDebug_Sent_Set(temp);                            //发送数据转换格式
  HAL_UART_Transmit_DMA(huart_x,(uint8_t*)Sent_data_type+3,39);
}

uint8_t  USART_Interrupt_flag=0xff;           //串口中断标志位
uint8_t  USART_Get_Num_Flag=0;                //串口数据获取标志
uint8_t  USART_receive[5]={0};                //串口接收缓存数组
int len=0;
/**
* @brief  串口接收解析函数
* @param  data_buf：接收到的数据指针
          length  ：数据长度
* @return None.
*/
void RecHandle(uint8_t *data_buf,uint16_t length)
{
  uint8_t Temp=0;
    len=length;
  for(int i=0;i<length;i++)
  {
    Temp=data_buf[i]; 
    switch(USART_Interrupt_flag)
    {
      case 0xff:  //USART0_Interrupt_flag==0xff时为等待模式，等待指令头输入
            if(Temp==0xf0)                  //指令头，识别上位机发送了修改指令
              USART_Interrupt_flag=0xf0;    //下一个指令将进入模式选择模式
            break;
      case 0xf0:                            //进入模式选择
            if(Temp==0x00)                  //修改参数
            {
              USART_Interrupt_flag=0x00;    //进入参数修改模式
              USART_Get_Num_Flag=0;
            }
            else if(Temp==0x01)             //修改模式
            {
              USART_Interrupt_flag=0x01;    //进入模式修改模式
              USART_Get_Num_Flag=0;
            }
            else if(Temp==0x02)
            {
              USART_Interrupt_flag=0x02;    //进入模式修改模式
              USART_Get_Num_Flag=0;
            }
            break;
      case 0x00:
            USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //参数处理
            {
              PARAMETER_MODIFICATION(USART_receive);
              USART_Interrupt_flag=0xff;    //回到等待模式
            }
            break;
      case 0x01:
            USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //参数处理
            {
              MODE_MODIFICATION(USART_receive);
              USART_Interrupt_flag=0xff;    //回到等待模式
            }
            break;
      case 0x02:  USART_receive[USART_Get_Num_Flag]=Temp;
            USART_Get_Num_Flag++;
            if(USART_Get_Num_Flag>4)        //参数处理
            {
              if(USART_receive[0]==0x0a)
              {
                for(int j=1;j<5;j++)
                {
                  if(USART_receive[j]!=0x0a)
                    USART_Interrupt_flag=0xff;    //回到等待模式
                }
                if(USART_Interrupt_flag==0x02)
                {
                  On_Off_flag =1;
                  USART_Interrupt_flag=0xff;    //回到等待模式
                }
              }
              else if(USART_receive[0]==0xb0)
              {
                for(int j=1;j<5;j++)
                {
                  if(USART_receive[j]!=0xb0)
                    USART_Interrupt_flag=0xff;   //回到等待模式
                }
                if(USART_Interrupt_flag==0x02)
                {
                  On_Off_flag =0;
                  USART_Interrupt_flag=0xff;    //回到等待模式
                }
              }
              else 
                USART_Interrupt_flag=0xff;     //回到等待模式
            }
            break;
            
      default:  USART_Interrupt_flag=0xff;    //回到等待模式
            break;
    }
  }
}

/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/

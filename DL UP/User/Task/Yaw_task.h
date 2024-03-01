#ifndef FIX_YAW_H
#define FIX_YAW_H

#include "INS_task.h"
#include "PID.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "exchange.h"

//声明一些其他文件的全局变量
extern ins_data_t ins_data;		//这是陀螺仪解算出来的数据
extern RC_ctrl_t rc_ctrl;		//遥控器数据
typedef struct
{
      
      int16_t speed;
      int16_t sendspeed;
      int16_t target_speed;
      int16_t current; 
      float target_angle;
      int16_t bmz;
      float angle;
      float anglen;
      int16_t temp;
      int16_t torque_current;
      

} motor;

//定义一些宏定义
void pitch_cal();
void Yaw_angle_task(void const *pvParameters);

//定义函数
void diy_control();
void Yaw_task(void const *pvParameters);
void Yaw_angle_task(void const *pvParameters);

#endif
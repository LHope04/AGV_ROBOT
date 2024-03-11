/*
*@author     CandyL
*@brief      舵轮底盘控制代码  
*@date       2023.12.30
*@param      None
*@return     None
*@warning    该文件为主函数，功能可根据需求修改，具体功能函数在"motion_overlay.c"中。
*/

#include "freertos.h"
#include "can.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "user_can.h"
#include "user_pid.h"
#include "rc_potocal.h"
#include "handle_value.h"
#include "motion_overlay.h"
#include "channel_changes.h"
#include "INS_task.h"
#include "usbd_cdc.h"


extern int Up_ins_yaw;
extern motor_info motor[8]; //底盘电机数据
extern RC_ctrl_t rc_ctrl; //遥控器数据
extern fp32 INS_angle[3]; //下C板陀螺仪数据
extern up_data UpData; //上C板数据
extern int16_t motor_angle[4]; //6020角度 在motion_overlay.c中计算 作为全局变量
extern int16_t motor_speed[4]; //3508速度
extern int omega; 
		uint8_t Buf1[10];
uint16_t initial_angle[4];
int16_t Max_out_a = 20000;
int16_t Max_iout_a = 20000;
int16_t Max_out_s = 15000; //电压控制转速，电流控制扭矩
int16_t Max_iout_s = 2000;
pidTypeDef PID_angle[4];
pidTypeDef PID_speed_3508[4];
pidTypeDef PID_speed_6020[4];
extern fp32 yaw_err ;
fp32 error_theta; //云台坐标系与底盘坐标系间夹角(此时为0~360度) 后期接收后需要对所得theta进行处理
extern float Hero_chassis_power;
void Yaw_Diff()
{
	UpData.yaw_up = Up_ins_yaw ; //测试舵轮 可删
	//UpData.yaw_up = Up_ins_yaw;
	error_theta = UpData.yaw_up - INS_angle[0]+yaw_err; 
	error_theta = error_theta*3.1415926/180; //转化为弧度制
}
USBD_HandleTypeDef axy;
void Chassis(void const * argument)
{
	float PID_s[3] = {10,0.05,0};
	float PID_a[3] = {35,0,3};
	float PID[3] = {5,0,0};
	
	int m = 0;
	
	for(int i=0;i<4;i++){
		pid_init(&PID_speed_6020[i],PID_s[0],PID_s[1],PID_s[2]);
		pid_init(&PID_angle[i],PID_a[0],PID_a[1],PID_a[2]);
		pid_init(&PID_speed_3508[i],PID[0],PID[1],PID[2]);
	}
	USBD_CDC_SetRxBuffer(&axy,Buf1);
  for(;;)
  {
//**************************************读取6020初始角度**************************************//
//		if(m==0){
//			HAL_Delay(10);
//			for(int i=0;i<4;i++)
//			{
//				initial_angle[i] = motor[i].angle; //读取电机初始角度 0~8192
//			}
//			m++;
//		}
//********************************************************************************************//

		//设置初始角度		
		if(m==0&&rc_ctrl.rc.s[0]==1){
			initial_angle[0] = 3759; //初始角度（底盘正前方各轮子角度）
			initial_angle[1] = 1723;
			initial_angle[2] = 1070;
			initial_angle[3] = 1726;
			m++;
		}
		
		Yaw_Diff(); //得到上C板与下C板间yaw的差值

		//遥控器控制底盘不同运动
		//具体实现方式在"motion_overlay.c"

			compound_control(); //旋转加平移运动


			//CDC_Transmit_FS(&Buf1, 10);
		
		
		//can_cmd_send_6020_2(10000,1000,1000,1000);
    osDelay(10);
  }
}
extern uint16_t Hero_chassis_power_limit;
void supercap()
{
	while(1)
	{
		//int power=1000;
		//printf("%s\n", power);
		int power = (int)Hero_chassis_power_limit;
		uint8_t iuy[7] = "P100P\r\n";
		
    char buffer[20]; // 保证足够的缓冲区大小以容纳您的数字

    // 将 power 格式化为字符串并将其存储在 buffer 中
    sprintf(buffer, "%d", power);

    // 输出格式化后的字符串

		HAL_UART_Transmit(&huart1,(uint8_t *)iuy,7,0xff);
		
		
//    printf("%s", "40");
//		printf("%s\n", "P");
		
		osDelay(1000);
	}
}
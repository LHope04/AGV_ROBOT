#include "Yaw_task.h"
#include "Exchange_task.h"
#include "stm32f4xx_it.h"
//	思路：利用速度环作基础PID反馈给云台，用云台陀螺仪的yaw值不变作角度环进行补偿
//  注意：云台往左转是负数
//  这一版本是针对遥控器控制的版本
//  YAW轴是6020电机，采用上C板CAN_1，电机ID为6

//	定义一些全局变量
extern int16_t mouse_x;
extern int16_t mouse_y;
extern pid_struct_t angle_pid;
  fp32 ga6020_pid [3]={10,0,3};	
    fp32 gs6020_pid [3]={10,0.1,0};
		pid_struct_t motor_pid_gimbal_a[2];//???????PID
pid_struct_t motor_pid_gimbal_s[2];//???????PID
		    
int8_t yaw_choice_flag = 0;
int8_t yaw_mode = 1;
	
fp32 ins_yaw;
fp32 ins_yaw_update = 0;
fp32 Driftring_yaw = 0;
fp32 ins_pitch;
fp32 target_angleyaw = 0;
		
		extern float yaw_ss,pitch_ss;
		float angle =0 ;
fp32 ins_row;
fp32 init_yaw;	//记住init_yaw初值
int yaw_model_flag = 1;
fp32 err_yaw;		//锁住的角度
fp32 angle_weight = 3;	//角度环->速度环，映射的权重
fp32 ins_gyro;
fp32 pitch_target;
int pitch_send;
int Yaw_count = 0;
extern Vision_Recv_s recv;
extern int16_t yaw_s,pitch_s;
extern int8_t errt;
extern float yaw_send;
//前馈控制变量
int16_t Rotate_w;
int16_t Rotate_W;
int angle_pause = 0;
int yaw_a = 0;
					int yaw_p = 0;
#define Rotate_gain 1.38f
#define Chassis_R	30.0f
#define Chassis_r 7.5f//7.5f

#define valve 50		//阈值
#define base 1024		//遥控器的回中值
#define base_max 1684		
#define base_min 364
#define angle_valve 1		//角度阈值，在这个范围内就不去抖动了
#define mouse_x_valve 10
#define mouse_x_weight 0.5f
#define Yaw_minipc_valve 1
#define Yaw_minipc_weight 1.0f

//校正漂移标志位
int8_t Update_yaw_flag = 0;

//定义一些函数
float pitch_target_p = 0;

//初始化PID参数
static void Yaw_init();	

//每次循环初始化
static void Yaw_loop_init();

//读取imu参数
static void Yaw_read_imu();

//模式选择
static void Yaw_choice();
static void Yaw_angle_choice();

//陀螺仪锁云台（回中）
static void Yaw_fix();
static void Yaw_tuoluo_fix();

//Mode_1下的控制算法
static void Yaw_mode_1();
static void Yaw_angle_mode_1();

//Mode_2下的控制算法
static void Yaw_mode_2();
static void Yaw_angle_mode_2();

//前馈控制
static void Yaw_Rotate();

//鼠标控制叠加
static void Yaw_mouse();
static void Yaw_angle_mouse();

//PID计算和发送
static void Yaw_can_send();

//叠加视觉自瞄
static void Yaw_minipc_control();
static void Yaw_angle_init();
//视觉清零
static void Yaw_minipc_zero();


float yaw_err = 0;
float yaw_n = 0;
float pitch_n = 0;
void Yaw_angle_task(void const *pvParameters)
{
  //初始化设置
	
	Yaw_angle_init();
	pitch_target_p = 0;
	target_angleyaw= 0;

	
	//循环任务运行
  for(;;)
  {
		Yaw_loop_init();
		Yaw_read_imu();
		diy_control();
		//模式判断,左上角开关开到最下方
			Yaw_angle_mode_2();
		if((rc_ctrl.rc.s[1] == 3||rc_ctrl.rc.s[1] == 2)&&recv.cheak)
				{

					Yaw_minipc_control();
				}

		pitch_cal();


		angle = (float)motor_info[4].rotor_angle/27;
		pitch_target_p =- pid_calc1(&motor_pid_gimbal_a[0],pitch_target,INS.Pitch);			
		pitch_send = pid_calc(&motor_pid_gimbal_s[0], pitch_target_p ,-9.55f*INS.Gyro[0]);
		
		Yaw_can_send();
    osDelay(1);
  }

}
void pitch_cal()
{
					target_speed[5] = pid_calc1(&angle_pid,target_angleyaw,ins_yaw);


		motor_info[5].set_voltage = pid_calc(&motor_pid[5], target_speed[5],19.8f*ins_gyro );
		

					if(pitch_target>30)
		{
		pitch_target = 30;
		}
		if(pitch_target<-45)
		{
		pitch_target = -45;
		}//0-50 240-0
}

void diy_control()
{
					if(rc_ctrl.rc.s[0] == 1)
		{
			
				yaw_n = yaw_ss + yaw_err ; 
			 pitch_n = pitch_ss;
		if(errt != 0)
		{
			yaw_err =  (ins_yaw - yaw_ss);
			if(errt > 0)
			{
			yaw_n += 3;
			yaw_err += 3
			
			
			;}
			if(errt < 0 )
			
					{
			yaw_n -= 3;
					yaw_err -= 3;}

		
		
		}

		target_angleyaw = yaw_n;
		pitch_target = -pitch_n;
		
		
		
		}
}
//初始化PID参数
static void Yaw_init()	
{
	//id为can1的5号
	pid_init(&motor_pid[5],85,3,40,15000,15000);//85,7,5//110,1,40,15000,15000
}
static void Yaw_angle_init()	
{
	//id为can1的5号
	pid_init(&motor_pid[5],700,5,0,15000,20000);//85,7,5//110,1,40,15000,15000
	pid_init(&angle_pid,4,0,10,15000,20000);//85,7,5//110,1,40,15000,15000
		pid_init(&motor_pid_gimbal_a[0],4,0,10,15000,30000);//85,7,5//110,1,40,15000,15000
	pid_init(&motor_pid_gimbal_s[0],600,0.5,0,15000,30000);//85,7,5//110,1,40,15000,15000
	
	
}


//循环初始化
static void Yaw_loop_init()
{
	target_speed[5]=0;
}


//读取imu参数
static void Yaw_read_imu()
{
		//三个角度值读取
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_row = ins_data.angle[2];
	  ins_gyro=ins_data.gyro[0];

		
		//校正
		ins_yaw_update = ins_yaw ;
}

static void Yaw_can_send()
{
	
	
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
//	tx_header.StdId = 0x1ff;
//  tx_header.IDE   = CAN_ID_STD;//标准帧
//  tx_header.RTR   = CAN_RTR_DATA;//数据帧
//  tx_header.DLC   = 8;		//发送数据长度（字节）

//	tx_data[0] = (pitch_send>>8)&0xff;	//先发高八位		
//  tx_data[1] = (pitch_send)&0xff;
//  tx_data[2] = 0x00;	
//  tx_data[3] = 0x00;//pitch_send
//  tx_data[4] = 0x00;
//  tx_data[5] = 0x00;
//  tx_data[6] = 0x00;
//  tx_data[7] = 0x00;
//  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	osDelay(1);
//	motor_info[5].set_voltage = 30000;
	
	tx_header.StdId = 0x1ff;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (pitch_send>>8)&0xff;//先发高八位		
  tx_data[1] = (pitch_send)&0xff;
  tx_data[2] =(motor_info[5].set_voltage>>8)&0xff;	
  tx_data[3] = (motor_info[5].set_voltage)&0xff;//pitch_send
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

static void Yaw_Rotate()
{
	Rotate_W = (Rotate_gain * Rotate_w * Chassis_r) / Chassis_R;
	target_speed[5]+=Rotate_W;
}
//陀螺仪锁云台
static void Yaw_fix()
{
	//遥感回中锁云台(一般回中锁)
					if(yaw_model_flag == 1)		//移动云台后重新记住云台的初始位置的值
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw_update - init_yaw;		//用实时数据减初始数据
					
					//越界处理,保证转动方向不变
					if(err_yaw < -180)	//	越界时：180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	越界时：-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//阈值判断
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[5] -= err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[5] = 0;
					}
				
}
static void Yaw_angle_fix()
{
	//遥感回中锁云台(一般回中锁)
					if(yaw_model_flag == 1)		//移动云台后重新记住云台的初始位置的值
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
					err_yaw = ins_yaw_update - init_yaw;
						
					
					if(err_yaw < -180)	//	越界时：180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	越界时：-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//阈值判断
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_angleyaw = init_yaw;		//用实时数据减初始数据
					}
					
					else
					{
						target_speed[5] = 0;
					}
				
}
static void Yaw_tuoluo_fix()
{
	//遥感回中锁云台(一般回中锁)
					if(yaw_model_flag == 1)		//移动云台后重新记住云台的初始位置的值
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw_update - init_yaw;		//用实时数据减初始数据
					
					//越界处理,保证转动方向不变
					if(err_yaw < -180)	//	越界时：180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	越界时：-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//阈值判断
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[5] -= err_yaw * 2;
					}
					
					else
					{
						target_speed[5] = 0;
					}
				
}
static void Yaw_mouse()
{
	if(mouse_x > mouse_x_valve || mouse_x < -mouse_x_valve)
	{
		yaw_model_flag = 1;
		target_speed[5] -= (fp32)mouse_x * mouse_x_weight;
	}
}
static void Yaw_angle_mouse()
{
	if(mouse_x > mouse_x_valve || mouse_x < -mouse_x_valve)
	{
		yaw_model_flag = 1;
		target_angleyaw -= (fp32)mouse_x * 0.002;
		pitch_target -= (fp32)mouse_y * 0.002;
	}
}

static void Yaw_mode_1()
{
				
				if(rc_ctrl.rc.ch[2] > base-valve && rc_ctrl.rc.ch[2] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					Yaw_fix();
				}
				//不回中的时候可以移动云台
				else if( (rc_ctrl.rc.ch[2] >= base+valve && rc_ctrl.rc.ch[2] <= base_max) || (e_flag))
				{
					target_speed[5] -= 120;
					yaw_model_flag = 1;
				}
				else if( (rc_ctrl.rc.ch[2] >= base_min && rc_ctrl.rc.ch[2]<base - valve ) || (q_flag))
				{
					target_speed[5] += 60;
					yaw_model_flag = 1;
				}
				if(rc_ctrl.rc.ch[3] > base-valve && rc_ctrl.rc.ch[3] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					
				}
				//不回中的时候可以移动云台
				else if( (rc_ctrl.rc.ch[3] >= base+valve && rc_ctrl.rc.ch[3] <= base_max) || (e_flag))
				{
					pitch_target -= 120;
					
				}
				else if( (rc_ctrl.rc.ch[3] >= base_min && rc_ctrl.rc.ch[3]<base - valve ) || (q_flag))
				{
					pitch_target += 60;
					
				}
				
				Yaw_mouse();
				//右键触发自瞄
				if(press_right)
				{
					Yaw_minipc_control();
				}
											

}
static void Yaw_angle_mode_1()
{
				
				if(rc_ctrl.rc.ch[2] > base-valve && rc_ctrl.rc.ch[2] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					Yaw_angle_fix();
				}
				//不回中的时候可以移动云台
				else if( (rc_ctrl.rc.ch[2] >= base+valve && rc_ctrl.rc.ch[2] <= base_max) || (e_flag))
				{
					target_angleyaw -= 0.3;//da
					yaw_model_flag = 1;
				}
				else if( (rc_ctrl.rc.ch[2] >= base_min && rc_ctrl.rc.ch[2]<base - valve ) || (q_flag))
				{
					target_angleyaw += 0.3;
					yaw_model_flag = 1;
				}
								if(rc_ctrl.rc.ch[3] > base-valve && rc_ctrl.rc.ch[3] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					
				}
				//不回中的时候可以移动云台
				else if( (rc_ctrl.rc.ch[3] >= base+valve && rc_ctrl.rc.ch[3] <= base_max) || (e_flag))
				{
					pitch_target -= 0.3;
					
				}
				else if( (rc_ctrl.rc.ch[3] >= base_min && rc_ctrl.rc.ch[3]<base - valve ) || (q_flag))
				{
					pitch_target += 0.3;
					
				}
				
				Yaw_angle_mouse();
				//右键触发自瞄
				if(press_right)
				{
					Yaw_minipc_control();
				}
											

}
static void Yaw_mode_2()
{
	
		if(rc_ctrl.rc.ch[2] > base-valve && rc_ctrl.rc.ch[2] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
	{
		//Yaw_fix();
	}
	else if( (rc_ctrl.rc.ch[2] >= base+valve && rc_ctrl.rc.ch[2] <= base_max) || (e_flag) )
	{
		target_speed[5] -= 60;
		yaw_model_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[2] >= base_min && rc_ctrl.rc.ch[2]<base - valve ) || (q_flag) )
	{
		target_speed[5] += 60;
		yaw_model_flag = 1;
	}
	Yaw_mouse();
					//右键触发自瞄
	if(press_right)
	{
		Yaw_minipc_control();
	}


}
static void Yaw_angle_mode_2()
{
	
		if(rc_ctrl.rc.ch[2] > base-valve && rc_ctrl.rc.ch[2] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
	{
		//Yaw_angle_fix();
	}
	else if( (rc_ctrl.rc.ch[2] >= base+valve && rc_ctrl.rc.ch[2] <= base_max) || (e_flag) )
	{
		target_angleyaw -= 0.3;
		yaw_model_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[2] >= base_min && rc_ctrl.rc.ch[2]<base - valve ) || (q_flag) )
	{
		target_angleyaw += 0.3;
		yaw_model_flag = 1;
	}
					if(rc_ctrl.rc.ch[3] > base-valve && rc_ctrl.rc.ch[3] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					
				}
				//不回中的时候可以移动云台
				else if( (rc_ctrl.rc.ch[3] >= base+valve && rc_ctrl.rc.ch[3] <= base_max) || (e_flag))
				{
					pitch_target -= 0.3;
					
				}
				else if( (rc_ctrl.rc.ch[3] >= base_min && rc_ctrl.rc.ch[3]<base - valve ) || (q_flag))
				{
					pitch_target += 0.3;
					
				}
	Yaw_angle_mouse();
					//右键触发自瞄
	if(press_right)
	{
		Yaw_minipc_control();
	}


}

//下降延触发
static void Yaw_choice()
{
	if(r_flag)
	{
		yaw_choice_flag = 1;
	}
	
	if( (!r_flag) && (yaw_choice_flag == 1) )	
	{
		yaw_choice_flag = 0;
		if(yaw_mode == 1)
		{
			yaw_mode = 2;
		}
		else if(yaw_mode == 2)
		{
			yaw_mode = 1;
		}
	}
}
static void Yaw_angle_choice()
{
	if(r_flag)
	{
		yaw_choice_flag = 1;
	}
	
	if( (!r_flag) && (yaw_choice_flag == 1) )	
	{
		yaw_choice_flag = 0;
		if(yaw_mode == 1)
		{
			yaw_mode = 2;
		}
		else if(yaw_mode == 2)
		{
			yaw_mode = 1;
		}
	}
}

static void Yaw_minipc_control()
{
	
		yaw_model_flag = 1;
		//target_angleyaw = yaw_send;
	target_angleyaw = recv.yaw;
		pitch_target = recv.pitch;
		
	
}

static void Yaw_minipc_zero()
{
	
}
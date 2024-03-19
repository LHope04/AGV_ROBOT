#include "Yaw_task.h"
#include "Exchange_task.h"
#include "stm32f4xx_it.h"
#include "Can_user.h"
//	˼·�������ٶȻ�������PID��������̨������̨�����ǵ�yawֵ�������ǶȻ����в���
//  ע�⣺��̨����ת�Ǹ���
//  ��һ�汾�����ң�������Ƶİ汾
//  YAW����6020�����������C��CAN_1�����IDΪ6

//	����һЩȫ�ֱ���
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
fp32 init_yaw;	//��סinit_yaw��ֵ
int yaw_model_flag = 1;
fp32 err_yaw;		//��ס�ĽǶ�
fp32 angle_weight = 3;	//�ǶȻ�->�ٶȻ���ӳ���Ȩ��
fp32 ins_gyro;
fp32 pitch_target;
int pitch_send;
int Yaw_count = 0;
extern Vision_Recv_s recv;
extern int16_t yaw_s,pitch_s;
extern int8_t errt;
extern float yaw_send;
//ǰ�����Ʊ���
int16_t Rotate_w;

int16_t Rotate_W;
int angle_pause = 0;
int yaw_a = 0;
					int yaw_p = 0;
#define Rotate_gain 1.38f
#define Chassis_R	30.0f
#define Chassis_r 7.5f//7.5f

#define valve 50		//��ֵ
#define base 1024		//ң�����Ļ���ֵ
#define base_max 1684		
#define base_min 364
#define angle_valve 1		//�Ƕ���ֵ���������Χ�ھͲ�ȥ������
#define mouse_x_valve 0.1
#define mouse_y_valve 0.1
#define mouse_x_weight 0.5f
#define Yaw_minipc_valve 1
#define Yaw_minipc_weight 1.0f

//У��Ư�Ʊ�־λ
int8_t Update_yaw_flag = 0;

//����һЩ����
float pitch_target_p = 0;

//��ʼ��PID����
static void Yaw_init();	

//ÿ��ѭ����ʼ��
static void Yaw_loop_init();

//��ȡimu����
static void Yaw_read_imu();

//ģʽѡ��
static void Yaw_choice();
static void Yaw_angle_choice();

//����������̨�����У�
static void Yaw_fix();
static void Yaw_tuoluo_fix();

//Mode_1�µĿ����㷨
static void Yaw_mode_1();
static void Yaw_angle_mode_1();

//Mode_2�µĿ����㷨
static void Yaw_mode_2();
static void Yaw_angle_mode_2();

//ǰ������
static void Yaw_Rotate();

//�����Ƶ���
static void Yaw_mouse();
static void Yaw_angle_mouse();

//PID����ͷ���
static void Yaw_can_send();

//�����Ӿ�����
static void Yaw_minipc_control();
static void Yaw_angle_init();
//�Ӿ�����
static void Yaw_minipc_zero();

uint8_t to_down_c_buf[8]={0,0,0,0,0};

float yaw_err = 0;
float yaw_n = 0;
float pitch_n = 0;
void Yaw_angle_task(void const *pvParameters)
{
  //��ʼ������
	
	Yaw_angle_init();
	pitch_target_p = 0;
	target_angleyaw= 0;

	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();
		Yaw_read_imu();
		//diy_control();
		//ģʽ�ж�,���Ͻǿ��ؿ������·�
			Yaw_angle_mode_2();
		if((rc_ctrl.rc.s[1] == 3||rc_ctrl.rc.s[1] == 2||press_right)&&recv.cheak)
				{

					Yaw_minipc_control();
				}

		pitch_cal();

		angle = (float)motor_info[4].rotor_angle/27;
		pitch_target_p =- pid_calc1(&motor_pid_gimbal_a[0],pitch_target,INS.Pitch);			
		pitch_send = pid_calc(&motor_pid_gimbal_s[0], pitch_target_p ,-9.55f*INS.Gyro[0]);
		
		Yaw_can_send();

memcpy((void*)&(to_down_c_buf[0]),(const void *)(&motor_info[5].rotor_angle),2);
can_remote(to_down_c_buf,0x47);
    osDelay(1);
  }

}
void pitch_cal()
{
	if(target_angleyaw > 180)
	{
	target_angleyaw -= 360;
	
	}
		if(target_angleyaw < -180)
	{
	target_angleyaw += 360;
	
	}
					target_speed[5] += pid_calc1(&angle_pid,target_angleyaw,ins_yaw);


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
//��ʼ��PID����
static void Yaw_init()	
{
	//idΪcan1��5��
	pid_init(&motor_pid[5],85,3,40,15000,15000);//85,7,5//110,1,40,15000,15000
}
static void Yaw_angle_init()	
{
	//idΪcan1��5��
	pid_init(&motor_pid[5],700,5,0,15000,20000);//85,7,5//110,1,40,15000,15000
	pid_init(&angle_pid,5,0,10,15000,20000);//85,7,5//110,1,40,15000,15000
//		pid_init(&motor_pid[5],300,3,0,15000,20000);//85,7,5//110,1,40,15000,15000
//	pid_init(&angle_pid,2,0,10,15000,20000);//85,7,5//110,1,40,15000,15000
		//pid_init(&motor_pid[5],100,5,0,18000,20000);//85,7,5//110,1,40,15000,15000
	//pid_init(&angle_pid,7,0,400,18000,20000);//85,7,5//110,1,40,15000,15000
//		pid_init(&motor_pid_gimbal_a[0],5,0,10,15000,30000);//85,7,5//110,1,40,15000,15000
//	pid_init(&motor_pid_gimbal_s[0],800,5,0,15000,30000);//85,7,5//110,1,40,15000,15000
			pid_init(&motor_pid_gimbal_a[0],3,0,10,15000,30000);//85,7,5//110,1,40,15000,15000
pid_init(&motor_pid_gimbal_s[0],400,5,0,15000,30000);//85,7,5//110,1,40,15000,15000
	
}


//ѭ����ʼ��
static void Yaw_loop_init()
{
	target_speed[5]=0;
}


//��ȡimu����
static void Yaw_read_imu()
{
		//�����Ƕ�ֵ��ȡ
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_row = ins_data.angle[2];
	  ins_gyro=ins_data.gyro[0];

		
		//У��
		ins_yaw_update = ins_yaw ;
}

static void Yaw_can_send()
{
	
	
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
//	tx_header.StdId = 0x1ff;
//  tx_header.IDE   = CAN_ID_STD;//��׼֡
//  tx_header.RTR   = CAN_RTR_DATA;//����֡
//  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

//	tx_data[0] = (pitch_send>>8)&0xff;	//�ȷ��߰�λ		
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
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (pitch_send>>8)&0xff;//�ȷ��߰�λ		
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
//����������̨
static void Yaw_fix()
{
	//ң�л�������̨(һ�������)
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw_update - init_yaw;		//��ʵʱ���ݼ���ʼ����
					
					//Խ�紦��,��֤ת�����򲻱�
					if(err_yaw < -180)	//	Խ��ʱ��180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	Խ��ʱ��-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//��ֵ�ж�
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
	//ң�л�������̨(һ�������)
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
					err_yaw = ins_yaw_update - init_yaw;
						
					
					if(err_yaw < -180)	//	Խ��ʱ��180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	Խ��ʱ��-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//��ֵ�ж�
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_angleyaw = init_yaw;		//��ʵʱ���ݼ���ʼ����
					}
					
					else
					{
						target_speed[5] = 0;
					}
				
}
static void Yaw_tuoluo_fix()
{
	//ң�л�������̨(һ�������)
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw_update - init_yaw;		//��ʵʱ���ݼ���ʼ����
					
					//Խ�紦��,��֤ת�����򲻱�
					if(err_yaw < -180)	//	Խ��ʱ��180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	Խ��ʱ��-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//��ֵ�ж�
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

		yaw_model_flag = 1;
		target_angleyaw -=  (rc_ctrl.mouse.x / 16384.00 * 200);

	
//	if(mouse_x < -mouse_x_valve)
//	{target_angleyaw +=  1;}
		if(mouse_y < -mouse_y_valve || mouse_y > mouse_y_valve)
	{
		yaw_model_flag = 1;

		pitch_target -= (fp32)mouse_y * 0.002;//up - down +
	}
}

static void Yaw_mode_1()
{
				
				if(rc_ctrl.rc.ch[2] > base-valve && rc_ctrl.rc.ch[2] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					Yaw_fix();
				}
				//�����е�ʱ������ƶ���̨
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
				//�����е�ʱ������ƶ���̨
				else if( (rc_ctrl.rc.ch[3] >= base+valve && rc_ctrl.rc.ch[3] <= base_max) || (e_flag))
				{
					pitch_target -= 120;
					
				}
				else if( (rc_ctrl.rc.ch[3] >= base_min && rc_ctrl.rc.ch[3]<base - valve ) || (q_flag))
				{
					pitch_target += 60;
					
				}
				
				Yaw_mouse();
				//�Ҽ���������
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
				//�����е�ʱ������ƶ���̨
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
				//�����е�ʱ������ƶ���̨
				else if( (rc_ctrl.rc.ch[3] >= base+valve && rc_ctrl.rc.ch[3] <= base_max) || (e_flag))
				{
					pitch_target -= 0.3;
					
				}
				else if( (rc_ctrl.rc.ch[3] >= base_min && rc_ctrl.rc.ch[3]<base - valve ) || (q_flag))
				{
					pitch_target += 0.3;
					
				}
				
				Yaw_angle_mouse();
				//�Ҽ���������
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
					//�Ҽ���������
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
				//�����е�ʱ������ƶ���̨
				else if( (rc_ctrl.rc.ch[3] >= base+valve && rc_ctrl.rc.ch[3] <= base_max) || (e_flag))
				{
					pitch_target -= 0.3;
					
				}
				else if( (rc_ctrl.rc.ch[3] >= base_min && rc_ctrl.rc.ch[3]<base - valve ) || (q_flag))
				{
					pitch_target += 0.3;
					
				}
	Yaw_angle_mouse();
					//�Ҽ���������



}

//�½��Ӵ���
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
		pitch_target = recv.pitch+1;
		
	
}

static void Yaw_minipc_zero()
{
	
}
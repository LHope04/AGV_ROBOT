#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Friction_task.h"
#include "Exchange_task.h"
#include "tim.h"
//�������дĦ���ֺͲ���
//Ħ���ֵ�ID�ֱ���2��3
//���̵�ID��5
uint16_t muzzle_heat;
uint16_t muzzle_heat_limit;
int Frictionerror = 0;
int16_t bopan = -10*19;
int bopan_s = 10*36;
extern int8_t shoot;
int16_t trigger_angle = 0; //Ҧצʵ݊އ׈
int16_t angle_dif = 0;
uint8_t friction_flag = 0; 
uint16_t press_time;//Ɛ׏дݼдЂʱӤ
int16_t trigger_target_angle;
uint16_t pos = 8191/8;
int8_t shoot_continue;
uint8_t err1  = 0;
uint8_t turns = 0; //ת֯Ȧ˽
uint8_t pre_mouse_l=0;
uint8_t mouse_state_check=0; //ɧڻΪ0ղ״̬һҤìΪ1ղ״̬لҤ
//ׁȡ˳Ҫ״̬
int8_t pre_key_l=0;
int8_t key_state_check; //ɧڻΪ0ղ״̬һҤìΪ1ղ״̬لҤ
pid_struct_t trigger_pid[7];//Ҧƌpid

uint16_t m2006_angle_flag=0;
uint16_t m2006_last_angle;
uint16_t m2006_angle;
static void model_choice();
//PID��ʼ��
static void Friction_init();

//Ħ���ּ��ٸ�ֵ
static void Friction_calc();

//Ħ�����ٶ�����
static void Friction_limit();

//Ħ���ּ��ٸ�ֵ
static void Friction_down();
static void trigger_init();
//Ħ���ֳ����ж�
static bool Friction_judeg();

//Ħ����Pid���ֵ����
static void Friction_send();
static void motor_2006_calc(uint16_t i);

//����Pid���ֵ����ͷ���
static void Bopan_send(int16_t speed);
static void trigger_mode_choose();
static void trigger_task();
void danger_mode();
 void ifdanger();

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */

	//TIM1 ->CCR1 = 500;
trigger_init();
	Friction_init();
m2006_last_angle = motor_info[2].rotor_angle;
  for(;;)
  {
		ifdanger();
	motor_2006_calc(2);
		model_choice();
				 danger_mode();
	ifdanger();
		
		 Friction_send();
if(r_flag)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,500);
}
else
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,2000);
}
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}
int trr = 0;
void danger_mode()
{
	if(muzzle_heat>(muzzle_heat_limit-10))
			{
				target_speed[2] =0;
				
			}
		if(b_flag)
		{target_speed[2] = -400*36;
		}
				if(rc_ctrl.rc.s[1] == 1&&rc_ctrl.rc.s[0] == 3)
		{target_speed[2] = -100*36;
		}
}
 void ifdanger()
{
//	if(motor_info[2].torque_current> 8000)
//	{
//		target_speed[2] = -50*36;
//		motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
//	}
		if(c_flag)
	{
		target_speed[2] = 10*36;
		motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	}

//	else if(motor_info[2].torque_current<-8000)
//	{
//		target_speed[2] = 50*36;
//		motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
//	}
		else
	{
		motor_info[2].set_voltage = pid_calc(&motor_pid[2], target_speed[2], motor_info[2].rotor_speed);
	}
	
}
static void trigger_init()
{
		pid_init(&trigger_pid[2], 10,  0.3, 1, 16384, 16384);	
	
		
		
}
static void friction_enable()
{

		target_speed[0] = -19*400;//-8700
    target_speed[1] =  19*400;// 8700
	
		motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
}

static void friction_disable()
{
	
	  target_speed[0] = 0;
    target_speed[1] = 0;
	  
		
	  motor_info[0].set_voltage = pid_calc(&motor_pid[0], target_speed[0], motor_info[0].rotor_speed);
    motor_info[1].set_voltage = pid_calc(&motor_pid[1], target_speed[1], motor_info[1].rotor_speed);
	
	
}
static void model_choice()
{
	//friction_flag_change(); //ׁȡдݼˇرдЂ
	//if(friction_flag||rc_ctrl.rc.s[0] == 1)
	if(rc_ctrl.rc.s[1] == 3||rc_ctrl.rc.s[1] == 1||rc_ctrl.rc.s[0] == 1)
	{
		friction_enable();
	}
	else //ǤԠȩ࠶֧ܺת̙׃0
	{
		friction_disable();
	}
  trigger_mode_choose();
	trigger_task();
}
//2006电机处理函数，将减速比消除
static void motor_2006_calc(uint16_t i)
{
	angle_dif = motor_info[i].rotor_angle - motor_info[i].last_angle;  //2006转过的角度差
	motor_info[i].last_angle = motor_info[i].rotor_angle;              //记录上次的角度
	
	//越界处理
        if(angle_dif>8191/2)
	{
		angle_dif -= 8191;
	}
	else if(angle_dif<- 8191/2)
	{
		angle_dif +=8191;
	}
	
	angle_dif/=36;               //减速比  把2006的角度差转换为拨爪的角度差
	
	trigger_angle += angle_dif;  //角度差累加,得到拨爪的绝对角度
	
        //越界处理
	if(trigger_angle>8191)
	{
		trigger_angle -= 8191;
	}
	else if(trigger_angle<0)
	{
		trigger_angle +=8191;
	}
																
}
static void trigger_task()
{
	m2006_angle = motor_info[2].rotor_angle;
	
	if(shoot_continue==0)      //եע
	{
		if(m2006_angle_flag<5)
		{
			target_speed[2] = -80*36;
		}
		else
		{
			target_speed[2] = 0;
			shoot_continue=2;
		}
		if(m2006_angle>m2006_last_angle)
		{
			m2006_angle_flag++;
		}
	}
		
		
	
	if(shoot_continue==1)      //lʤ
	{
		target_speed[2] = -80*36;
		m2006_angle_flag=0;
	}
	else if(shoot_continue==2) //ͣת
	{
		m2006_angle_flag=0;
		target_speed[2] = 0;
	}
	
	m2006_last_angle = m2006_angle;
	
	
}
static void trigger_mode_choose()
{
    //记录鼠标、遥控器状态是否发生变化（即是否由按下变为松开）
	key_state_check = pre_key_l -  rc_ctrl.rc.s[1];  
  	pre_key_l = rc_ctrl.rc.s[1];
	
	
	
    //判断按压时长
	if((press_time > 0 && press_time < 250)) //短按
	{
		if( rc_ctrl.mouse.press_l ==0 )
		{
			shoot_continue = 0;
			press_time = 0;
		}
	}
	
	if(press_time > 250)//长按
	{		
        	//设置发射状态为连发
			shoot_continue = 1;
	}
    
    
    //当按下鼠标，press_time开始记录按压时长
  if(rc_ctrl.mouse.press_l )
	{
		press_time += 1;
			
	}
	else if(rc_ctrl.mouse.press_l==0)//松开鼠标
	{ 
		if(press_time > 250 ) 
		 {
             //计数清0
      		 press_time = 0;
             //停转
      		 shoot_continue = 2;
         }
    }
}
static void Friction_init()
{
	pid_init(&motor_pid[0],40,0.8,1,16384,16384);
	pid_init(&motor_pid[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid[2],20,0.0f,0.5,10000,10000);
	motor_info[2].last_angle = motor_info[2].rotor_angle; //初始化，令之前角度==当前角度
}


static void Friction_calc()
{

	
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info[0].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info[1].rotor_speed);
}


static void Friction_down()
{
	target_speed_can_2[1]=0;
	target_speed_can_2[2]=0;
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[0].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[1].rotor_speed);
}

static bool Friction_judeg()
{
	if(	(motor_info_can_2[1].rotor_speed>=8500) && (motor_info_can_2[2].rotor_speed<=-8500) )
	{
		return true;
	}
	return false;
}

static void Friction_send()
{
	
		set_motor_voltage_can_2(0, 
                       
                      motor_info[0].set_voltage, 
                      motor_info[1].set_voltage,
motor_info[2].set_voltage,	
                      0);
}
//static void Friction_send()
//{
//	motor_info[4].set_voltage=pid_calc(&motor_pid[4],100,motor_info[0].rotor_speed);
//		set_motor_voltage_can_2(0, 
//                      1000, 
//                     1000, 
//                     1000, 
//                      0);
//}

static void Bopan_send(int16_t speed)
{
		motor_info[4].set_voltage=pid_calc(&motor_pid[4],speed,motor_info[4].rotor_speed);
		set_motor_voltage(0, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
}

static void Friction_limit()
{

}
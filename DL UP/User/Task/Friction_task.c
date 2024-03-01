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

//�������дĦ���ֺͲ���
//Ħ���ֵ�ID�ֱ���2��3
//���̵�ID��5
int Frictionerror = 0;
int16_t bopan = -10*19;
int bopan_s = 10*36;
extern int8_t shoot;
//PID��ʼ��
static void Friction_init();

//Ħ���ּ��ٸ�ֵ
static void Friction_calc();

//Ħ�����ٶ�����
static void Friction_limit();

//Ħ���ּ��ٸ�ֵ
static void Friction_down();

//Ħ���ֳ����ж�
static bool Friction_judeg();

//Ħ����Pid���ֵ����
static void Friction_send();

//����Pid���ֵ����ͷ���
static void Bopan_send(int16_t speed);

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();
  for(;;)
  {

//		if(rc_ctrl.rc.s[1] == 1 || press_left)
//		{	
//			Bopan_send(bopan);
//		}
//		else
//		{			
//			Bopan_send(0);
////			Friction_down();			
//		}
		if(rc_ctrl.rc.s[1] == 1)
		{
			if(shoot)
			{
			target_speed_can_2[1]=-19*300;//258
	target_speed_can_2[2]=19*300;
			bopan_s = -30*36;
			}
			else
			{
			bopan_s =0 ;
					target_speed_can_2[1]=0;//258
	target_speed_can_2[2]=0;
			}
		}
		else if(rc_ctrl.rc.s[1] == 3||shoot)
		{
//			target_speed_can_2[1]=19*310;//258
//	target_speed_can_2[2]=-19*310;
						target_speed_can_2[1]=-19*300;//258
	target_speed_can_2[2]=19*300;
		bopan_s =0 ;
		

		}
	else	if(rc_ctrl.rc.s[1] == 2||shoot)
		{
		bopan_s = -30*36;}
		else
		{
		
		bopan_s =0 ;
					target_speed_can_2[1]=0;//258
	target_speed_can_2[2]=0;
		}
			
				Friction_calc();//Ħ����һֱת
		Friction_limit();
		Friction_send();
		Frictionerror++;


		
//		if(Friction_judeg())
//		{
//			Bopan_send(bopan);
//		}
//		else
//		{
//			Bopan_send(0);
//		}
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

static void Friction_init()
{
	pid_init(&motor_pid_can_2[2],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid[4],20,0.03,0.5,16384,16384);
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
	
	motor_info[4].set_voltage=pid_calc(&motor_pid[4],bopan_s,motor_info[2].rotor_speed);
		set_motor_voltage_can_2(0, 
                       
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage,
motor_info[4].set_voltage,	
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
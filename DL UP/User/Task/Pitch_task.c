#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Pitch_task.h"
#include "Exchange_task.h"
#include "exchange.h"

//��������������̨����ģʽѡ�񣬿��ƣ�У׼��
//��װһЩ�����������п��Ƶ���
//Pitch������C��CAN_2�����IDΪ5
//�������⣺��Ҫ����̨Pitch�������·���������Pitch�ܸ�Ƶ���Ӱ�죬�����ٶ�̫��

//����һЩ����
//��λ��������е������
#define Up_inf 2
#define Down_inf -2
#define mouse_y_valve 15
#define mouse_y_weight 10.0f
#define Pitch_minipc_valve 1
#define Pitch_minipc_weight	100.0f

//imu����
fp32 Err_pitch;
int16_t Up_pitch;
int16_t Down_pitch;
uint16_t Remember_pitch = 0;
uint8_t Remember_pitch_flag = 1;
extern ins_data_t ins_data;
extern int16_t mouse_y;
int Pitcherror = 0;
//��ʼ��PID����
static void gimbal_init();	

//У�����ӳɹ�
static bool gimbal_judge();	

//��ȡimu����
static void gimbal_read_imu();

//ģʽѡ��
static void gimbal_choice();

//Mode_1�µĿ����㷨
static void gimbal_mode_1();

//PID����ͷ���
static void gimbal_can_send();

//��λ��������������ݣ�
static void gimbal_imu_limit();

//��ת��λ
static void gimbal_imu_limit_2();

//������Pitch(����)
static void gimbal_mouse();

//��������
static void gimbal_minipc_control();
// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

	gimbal_init();	
	
  for(;;)
  {
		if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1	|| rc_ctrl.rc.s[1]==2)
		{
			gimbal_mode_1();
			gimbal_mouse();
			
			//�Ҽ���������
	  	if(press_right)
		 {
		  	gimbal_minipc_control();
		 }
		  gimbal_imu_limit();			
		}	
		gimbal_can_send();
		Pitcherror++;
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//��ʼ��PID����
static void gimbal_init()	
{
		pid_init(&motor_pid_can_2[4],18,0.1,5,1900,1900);
}


//У�����ӳɹ�
static bool gimbal_judge()
{

}


//��ȡimu����
static void gimbal_read_imu()
{
	//��ȡDown_pitch(д����can�Ľ��պ�������)
	
	//��-��
	Up_pitch = (int)ins_data.angle[1];
	Err_pitch = Down_pitch - Up_pitch ;//��-��Ϊ��ֵ
	Err_pitch += Remember_pitch;
}


//ģʽѡ��
static void gimbal_choice()
{

}


//Mode_1�㷨����򵥵���̨���ƣ��ٶȻ���
static void gimbal_mode_1()
{
		if( rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684  )
		{
			target_speed_can_2[4]=1500;//1250
		}
		else if(rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974)
		{
			target_speed_can_2[4]=-1500;//1250
		}
		else
		{
			target_speed_can_2[4]=0;
		}
}	

//PID����ͷ���
static void gimbal_can_send()
{
		
    motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], motor_info_can_2[4].rotor_speed);//????PID???
		
		set_motor_voltage_can_2(1, 
                      motor_info_can_2[4].set_voltage, 
                      motor_info_can_2[5].set_voltage, 
                      motor_info_can_2[6].set_voltage, 
                      0);
}


//��������λ����ԣ�,ֹͣ��λ
static void gimbal_imu_limit()
{
	gimbal_read_imu();
	if( (Up_pitch < Down_inf) &&((rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (shift_flag) || (mouse_y < -mouse_y_valve)))
	{
		target_speed_can_2[4]=0;
	}
	
	else if( (Up_pitch > Up_inf) &&  ((rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (ctrl_flag) || (mouse_y > mouse_y_valve)) )
	{
		target_speed_can_2[4]=0;
	}
}

//��ת��λ��
static void gimbal_imu_limit_2()
{
	gimbal_read_imu();
	if( (Err_pitch > Up_inf) && ((rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (ctrl_flag)))
	{
		target_speed_can_2[4]=-400;
	}
	
	else if( (Err_pitch < Down_inf) && ( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (shift_flag)) )
	{
		target_speed_can_2[4]=400;
	}
}

//������
static void gimbal_mouse()
{
	if(mouse_y > mouse_y_valve || mouse_y < -mouse_y_valve)
	{
		target_speed_can_2[4] += (fp32)mouse_y * mouse_y_weight;
	}
}

//����
static void gimbal_minipc_control()
{
	if(Pitch_minipc > Pitch_minipc_valve || Pitch_minipc < -Pitch_minipc_valve)
	{
		target_speed_can_2[4] -= ((fp32)Pitch_minipc) * Pitch_minipc_weight;
	}
}



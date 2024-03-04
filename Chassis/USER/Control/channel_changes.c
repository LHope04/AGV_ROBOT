#include "channel_changes.h"
#include "rc_potocal.h"
#include "chassis.h"
#include "motion_overlay.h"
#include "handle_value.h"
#include "user_can.h"
#include "user_pid.h"
#include "NRF24L01.h"
extern RC_ctrl_t rc_ctrl;
extern uint16_t initial_angle[4];
extern motor_info motor[8];
extern pidTypeDef PID_angle[4];
extern pidTypeDef PID_speed_3508[4];
extern pidTypeDef PID_speed_6020[4];
extern int16_t motor_angle[4]; //6020angle 设置为全局变量在"motion_overlay.c"中计算
extern int16_t motor_speed[4]; //3508speed
extern int16_t Max_out_a;
extern int16_t Max_iout_a;
extern int16_t Max_out_s;
extern int16_t Max_iout_s;
extern int Up_ins_yaw ;
extern int8_t vx1,vy1,vw;
int16_t get_6020[4];
int16_t speed_6020[4];
int16_t output_6020[4];
int16_t output_3508[4];
extern int omega; 
fp32 yaw_err = 0;

//将3508和6020运动模式结合，形成底盘控制

/********************************************************底盘平移控制********************************************************/
void translational_control()
{
	translate_6020((int16_t)vx1*3, (int16_t)vy1*3);
	for(int i=0;i<4;i++){
		//电机角度：get;将当前电机角度投影范围为 0 至 180/0 至 -180
		//加负号：使得电机解算方向与遥控器相符合（解算+-180时的小bug）
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		//get_6020[i]=int Up_ins_yaw ;
		
		//PID计算(get,set)->(当前值，目标值)
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	
	translate_3508((int16_t)vx1*3, (int16_t)vy1*3);
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s); //3508????
	}
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/


/***********************************************************底盘旋转控制*****************************************************/
void rotate_control()
{
	rotate_6020(); 
	for(int i=0;i<4;i++){
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	rotate_3508(rc_ctrl.rc.ch[4]);
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s); //3508????
	}
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/
int vxd,vyd;
extern int8_t shoot,xtl;

/******************************************************底盘旋转+平移控制*****************************************************/
void compound_control()
{
	//设置6020的旋转和平移角度，计算公式为motor_angle[4]
	
	if(rc_ctrl.rc.s[0]==1)
	{
	vxd = (int16_t)vy1*3;
	vyd = (int16_t)vx1*3;
	
	if(xtl)
	{
	omega = 10;
	yaw_err += 0.01916; 
	}
	else
	{
	omega = 0;
	}
	}
	 if(rc_ctrl.rc.s[0]==3)
	{
	vxd = rc_ctrl.rc.ch[0];
	vyd = rc_ctrl.rc.ch[1];
	
	if(a_flag)
	{
		vxd = -400;
	
	}
	else if(d_flag)
	{
		vxd = 400;
	
	}
		else if (!a_flag&&!d_flag&&rc_ctrl.rc.ch[0] == 0){
	vxd = 0;

	}
	
	if(w_flag)
	{
		vyd = 400;
		
	}
	else if(s_flag)
	{
		vyd = -400;
	}
else if (!w_flag&&!s_flag&&rc_ctrl.rc.ch[1] == 0){
	vyd = 0;

	}
	if(shift_flag)
	{
	omega = 10;
	yaw_err += 0.01916; 
	}
	else
	{
	
	omega = 0;
	}
	}
	 if(rc_ctrl.rc.s[0]==2)
	{
		omega = 10;
	vxd = rc_ctrl.rc.ch[0];
	vyd = rc_ctrl.rc.ch[1];
	yaw_err += 0.01916; 
	}
	 if(rc_ctrl.rc.s[0]==3&&!shift_flag){
	
	
	omega = 0;}
	
	
	compound_movement_6020(vxd, vyd); 
	for(int i=0;i<4;i++){
		get_6020[i] = -motor_value(initial_angle[i],motor[i+4].angle);
		speed_6020[i] = pid_cal_a(&PID_angle[i],get_6020[i],motor_angle[i],Max_out_a,Max_iout_a); 
		output_6020[i] = pid_cal_s(&PID_speed_6020[i],motor[i+4].speed,speed_6020[i],Max_out_s,Max_iout_s);
	}
	
	//设置3508的旋转和平移速度，计算公式为motor_speed[4]
	compound_movement_3508(vxd, vyd);
	for(int i=0;i<4;i++){
		output_3508[i] = pid_cal_s(&PID_speed_3508[i],motor[i].speed,motor_speed[i],Max_out_s,Max_iout_s);
	}
	can_cmd_send_6020(output_6020[0],output_6020[1],output_6020[2],output_6020[3]);
	can_cmd_send_3508(output_3508[0],output_3508[1],output_3508[2],output_3508[3]);
}
/****************************************************************************************************************************/
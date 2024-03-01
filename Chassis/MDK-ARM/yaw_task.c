
#include "rc_potocal.h"
#include "yaw_task.h"
#include "user_can.h"
  fp32 ga6020_pid [3]={10,0,3};	
    fp32 gs6020_pid [3]={10,0.1,0};
      fp32 gay6020_pid [3]={1,0,5};	
    fp32 gsy6020_pid [3]={200,0.01,0};
    fp32 fashe1_pid [3]={20,0,0};
    fp32 fashe2_pid [3]={20,0,0};
    fp32 bopan_pid [3]={20,0,0};
		 extern RC_ctrl_t rc_ctrl;
int vgx ;
    int vgy ;
pid_struct_t motor_pid_gimbal_a[2];//???????PID
pid_struct_t motor_pid_gimbal_s[2];//???????PID
pid_struct_t motor_pid_fashe1[1];//???????PID
pid_struct_t motor_pid_fashe2[1];//???????PID
pid_struct_t motor_pid_bopan[1];//???????PID
motor1 motor_gimbal[4];
motor1 motor_fashe[3];
#define beishu 0.002
#define beishu_p 0.002

void gambal_init()
{
    pid_init1(&motor_pid_gimbal_a[0], gay6020_pid, 10000, 30000);
    pid_init1(&motor_pid_gimbal_a[1], gay6020_pid, 10000, 30000);
    pid_init1(&motor_pid_gimbal_s[0], gsy6020_pid, 10000, 30000);
    pid_init1(&motor_pid_gimbal_s[1], gsy6020_pid, 10000, 30000);
    pid_init1(&motor_pid_fashe1[0], fashe1_pid, 10000, 10000);
    pid_init1(&motor_pid_fashe2[0], fashe2_pid, 10000, 10000);
    pid_init1(&motor_pid_bopan[0], bopan_pid, 10000, 10000);

    
    motor_gimbal[0].angle = motor_gimbal[0].bmz;
    motor_gimbal[0].angle /= 27;
    motor_gimbal[1].angle = motor_gimbal[1].bmz;
    motor_gimbal[1].angle /= 27;

   motor_gimbal[1].target_angle = 250;//180-300 //-660-660
    motor_gimbal[0].target_angle = 270;

}
void gimbal_control()
{
        motor_gimbal[0].angle = motor_gimbal[0].bmz;
    motor_gimbal[0].angle /= 27;
    motor_gimbal[1].angle = motor_gimbal[1].bmz;
    motor_gimbal[1].angle /= 27;
    //????,pid??

     vgx = rc_ctrl.rc.ch[2];
     vgy = rc_ctrl.rc.ch[3];
     if(vgx>0)
     {
         motor_gimbal[0].target_angle += 0.1;
     }
         if(vgx<0)
     {
         motor_gimbal[0].target_angle -= 0.1;
     }
     if(vgy>0)
     {
         motor_gimbal[1].target_angle += 0.1;
     }
          if(vgy<0)
     {
         motor_gimbal[1].target_angle -= 0.1;
     }
    if(motor_gimbal[0].target_angle>360)
    {
        motor_gimbal[0].target_angle -= 360;
    }
    if(motor_gimbal[0].target_angle<0)
    {
        motor_gimbal[0].target_angle += 360;
    }
    

    motor_gimbal[0].target_speed = pid_calc_a(&motor_pid_gimbal_a[0], motor_gimbal[0].angle, motor_gimbal[0].target_angle);
    motor_gimbal[1].target_speed = pid_calc_a(&motor_pid_gimbal_a[1], motor_gimbal[1].angle, motor_gimbal[1].target_angle);
    motor_gimbal[0].sendspeed = pid_calc(&motor_pid_gimbal_s[0], motor_gimbal[0].speed, motor_gimbal[0].target_speed);
    motor_gimbal[1].sendspeed = pid_calc(&motor_pid_gimbal_s[1], motor_gimbal[1].speed, motor_gimbal[1].target_speed);
    
    //gb6020_send(1,0, 0,0,0);


    


    
}
void fashe()
{
    
           if(rc_ctrl.rc.s[0]==1)
       {
                  motor_fashe[0].target_speed = 0;
    motor_fashe[1].target_speed = 0;
    motor_fashe[2].target_speed = 0;
}
                  if(rc_ctrl.rc.s[0]==3)
       {
                  motor_fashe[0].target_speed = 0;
    motor_fashe[1].target_speed = 10000;
    motor_fashe[2].target_speed = 10000;
}
                  if(rc_ctrl.rc.s[0]==2)
       {
                  motor_fashe[0].target_speed = -1000;
    motor_fashe[1].target_speed = 10000;
    motor_fashe[2].target_speed = 10000;
}
    motor_fashe[0].sendspeed = pid_calc(&motor_pid_fashe1[0], motor_fashe[0].speed, motor_fashe[0].target_speed);
    motor_fashe[1].sendspeed = pid_calc(&motor_pid_fashe2[0], motor_fashe[1].speed, motor_fashe[1].target_speed);
    motor_fashe[2].sendspeed = pid_calc(&motor_pid_bopan[0], motor_fashe[2].speed, motor_fashe[2].target_speed);
       //mt3508_send_can2(0,motor_fashe[0].sendspeed,motor_fashe[1].sendspeed,motor_fashe[2].sendspeed,0);

}
void control_angle()
{
    if(motor_gimbal[1].target_angle<180&&vgy<0)
    {
        motor_gimbal[1].sendspeed = 0;
    }
        if(motor_gimbal[1].target_angle>300&&vgy>0)
    {
        motor_gimbal[1].sendspeed = 0;
    }
}
void gimbal_task()
{
    gambal_init();
    for(;;)
    {
        gimbal_control();
        control_angle();
       gb6020_send(1,motor_gimbal[0].sendspeed, 0,0,0);

        fashe();
        osDelay(1);
    }
}
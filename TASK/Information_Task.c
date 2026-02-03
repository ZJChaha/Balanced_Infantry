//
// Created by ZJC on 2024/11/3.
//

#include "Information_Task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "Chassis_Task.h"
#include "dm_motor_drv.h"
#include "Controller.h"
#include "observer.h"
#include "Detect_Task.h"
#include "imu_temp_ctrl.h"
#include "Buzzer_Task.h"
#include "Gimbal_task.h"

uint8_t Mode,last_Mode;
uint8_t state_flag;


void Information_Task(void const * argument)
{

    while(!IMU_init_finish)
    { osDelay(1);}

    while(!gimbal_init_finish)
    { osDelay(1);}

    buzzer_flag=5;

    while(rc_ctrl.rc.s[0]!=3)
    { osDelay(1);}

    buzzer_flag=6;

    for(;;)
    {
        set_mode(&Mode);




        if((Mode==Chassis_disable && last_Mode==Chassis_enable)||(Mode==Chassis_brake && last_Mode==Chassis_enable))
        {
            state_flag=2;

        }else if((Mode==Chassis_enable && last_Mode==Chassis_disable)||(Mode==Chassis_enable && last_Mode==Chassis_brake))
        {
            state_flag=1;
        }


        last_Mode=Mode;

        switch (state_flag)
        {
            case 0:
            {

            }
                break;
            case 1://使能底盘的初始化项目
            {

                state_vector[left].feedback.x_dot=0;
                state_vector[right].feedback.x_dot=0;
                theta_init[right]=state_vector[right].feedback.theta;
                theta_init[left]=state_vector[left].feedback.theta;
                state_vector[left].set.x=0;
                state_vector[right].set.x=0;
                enable_all_motor_os();
                defend_code=0;
                state_flag=0;
            }
            break;
            case 2://失能底盘需要清空的标志
            {
                defend_code=0;
                stand_up_finish=0;
                state_vector[left].feedback.x_dot=0;
                state_vector[right].feedback.x_dot=0;
                state_vector[left].set.x=0;
                state_vector[right].set.x=0;
                disable_all_motor_os();
                state_flag=0;
            }
            break;
            default:
            {


            }
        }
        osDelay(5);
    }

}


void set_mode(uint8_t* m)
{
    if(rc_ctrl.rc.s[0]==0x02 )//右下  刹车
        *m=Chassis_brake;
    else if( rc_ctrl.rc.s[0]==0x03)//右中  使能底盘，正常工作，底盘跟随云台
    {
        *m = Chassis_enable;
    }
    else if( rc_ctrl.rc.s[0]==0x01)//右上  失能底盘和云台
        *m=Chassis_disable;

}



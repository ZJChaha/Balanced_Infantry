//
// Created by ZJC on 2024/12/7.
//
#include "Shoot_Task.h"
#include "bsp_motor.h"
#include "Controller.h"
#include "dm_motor_drv.h"
#include "cmsis_os.h"

#include "bsp_usart.h"
#include "bsp_motor.h"
#include "Controller.h"
#include "dm_motor_drv.h"
#include "Information_Task.h"
#include "remote_control.h"
#include "observer.h"
#include "Detect_Task.h"



shoot_t Shoot;
pid_type_def shoot_motor_p;
pid_type_def shoot_motor_v;

float shoot_motor_p_pid[3]={15.f,0,0};
float shoot_motor_v_pid[3]={0.15f,0.004f,0};

int fire_cnt;
uint8_t last_key_state;

void Shoot_Task(void const * argument)
{

    PID_init(&shoot_motor_p,PID_POSITION,shoot_motor_p_pid,40,0);
    PID_init(&shoot_motor_v,PID_POSITION,shoot_motor_v_pid,1.f,0.2f);

    Shoot.rub_wheel_state=-1;

    for(;;)
    {

        Shoot.rub_cnt++;

        shoot_information_update(&Shoot);
        if(Shoot.rub_wheel_state==-1)//摩擦轮停止
        {
            if(Shoot.rub_cnt>20)
            {
                spd_ctrl(&hfdcan3,0x11,0.f);
                osDelay(1);
                spd_ctrl(&hfdcan3,0x22,0.f);
                Shoot.rub_cnt=0;
            }
            Shoot.shoot_motor.pos_set=Shoot.shoot_motor.final_pos;
        }
        else if(Shoot.rub_wheel_state==1)//摩擦轮开启
        {
            if(Shoot.rub_cnt>20)
            {
                spd_ctrl(&hfdcan3, 0x11, 35.5f);
                osDelay(1);
                spd_ctrl(&hfdcan3, 0x22, -35.5f);
                Shoot.rub_cnt=0;
            }

            if(Shoot.fire==1)
            {
                fire_cnt++;
                if(fire_cnt==19)
                {
                    Shoot.shoot_motor.pos_set+=0.785398f;
                    fire_cnt=0;
                }
            }
            else
            {
                Shoot.shoot_motor.pos_set=Shoot.shoot_motor.final_pos;//松开左键立即停止
                fire_cnt=0;
            }
        }

        Shoot.shoot_motor.vel_set= PID_calc(&shoot_motor_p,Shoot.shoot_motor.final_pos,Shoot.shoot_motor.pos_set);
        Shoot.shoot_motor.torque_set= PID_calc(&shoot_motor_v,Shoot.shoot_motor.vel,Shoot.shoot_motor.vel_set);

        shoot_motor_send_data();
        osDelay(5);
    }
}

void shoot_information_update(shoot_t *para)
{
    if(key_boad_vlue.key_R==1&&last_key_state==0)
    {
        para->rub_wheel_state*=-1;
    }

    last_key_state=key_boad_vlue.key_R;


    if(rc_ctrl.mouse.press_l==1)
    {
        para->fire=1;
    }
    else
    {
        para->fire=0;
    }
}

void shoot_motor_send_data(void)
{

    uint8_t data[8]={0};
    short temp=(short)(Shoot.shoot_motor.torque_set/0.00018f);//  /((10/10000)*0.18)

    data[0]=temp>>8;
    data[1]=temp;

    fdcanx_send_data(&hfdcan3,0x200,data,8);

}

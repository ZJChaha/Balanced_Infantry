//
// Created by ZJC on 2024/10/27.
//
#include "Gimbal_task.h"
#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "Buzzer_Task.h"
#include "All_struct.h"
#include "bsp_can.h"
#include "fdcan.h"
#include "QuaternionEKF.h"
#include "pid.h"
#include "bsp_usart.h"
#include "dm_motor_drv.h"
#include "Information_Task.h"
#include "remote_control.h"
#include "Detect_Task.h"
#include "imu_temp_ctrl.h"
#include "Controller.h"


gimbal_t Gimbal;


pid_type_def pitch_motor_p;
pid_type_def pitch_motor_v;
pid_type_def yaw_motor_p;
pid_type_def yaw_motor_v;

float pitch_motor_p_pid[3]={8.f,0,0};
float pitch_motor_v_pid[3]={0.3f,0,0.0f};
float yaw_motor_p_pid[3]={14.f,0,4.f};
float yaw_motor_v_pid[3]={15000.f,20.f,0};

uint8_t gimbal_init_finish;
uint8_t yaw_to_zero;
//yaw电压环，pitch电流环

void Gimbal_Task(void const * argument)
{

    gimbal_motor_pid_init();
    Gimbal.pitch_motor.pos_set=0.57f;

    while(!IMU_init_finish)
    { osDelay(1);}



    while(1)
    {
        gimbal_move_to_zero();
        gimbal_data_send();
        if(fabsf(Gimbal.yaw_motor.pos)<0.06f)  break;
        osDelay(5);
    }

    Gimbal.yaw_motor.pos_set=QEKF_INS.YawTotalAngle;

    gimbal_init_finish=1;


    for(;;)
    {

        if(Mode==Chassis_enable&&stand_up_finish==0)
        {
            gimbal_move_to_zero();
            if(fabsf(Gimbal.yaw_motor.pos)<0.06f)
            {
                yaw_to_zero = 1;
                Gimbal.yaw_motor.pos_set=QEKF_INS.YawTotalAngle;
            }
        }
        else
        {
            gimbal_information_update(&Gimbal);
            gimbal_output_calc(&Gimbal);
        }

        gimbal_data_send();
        osDelay(5);
    }

}

void gimbal_motor_pid_init(void)
{
    PID_init(&pitch_motor_p,PID_POSITION,pitch_motor_p_pid,10.f,0);
    PID_init(&pitch_motor_v,PID_POSITION,pitch_motor_v_pid,2.1f,0.2f);

    PID_init(&yaw_motor_p,PID_POSITION,yaw_motor_p_pid,8.f,0);
    PID_init(&yaw_motor_v,PID_POSITION,yaw_motor_v_pid,25000,5000);
}

void gimbal_information_update(gimbal_t *gimbal)
{
    if(Mode==Chassis_enable||Mode==Chassis_brake)
    {
        if(rc_ctrl.mouse.x==0 || rc_ctrl.mouse.y==0)
        {
            gimbal->pitch_motor.pos_set+=((float)rc_ctrl.rc.ch[3]/660.f)*0.015f;
            gimbal->yaw_motor.pos_set-=((float)rc_ctrl.rc.ch[2]/660.f)*0.025f;
        }
        else
        {
            gimbal->pitch_motor.pos_set-=((float)rc_ctrl.mouse.y)*0.0001f;
            gimbal->yaw_motor.pos_set-=((float)rc_ctrl.mouse.x)*0.0001f;
        }
    }
    else
    {
        gimbal->pitch_motor.pos_set=gimbal->pitch_motor.pos;
        gimbal->yaw_motor.pos_set=QEKF_INS.YawTotalAngle;
    }
    limit(&gimbal->pitch_motor.pos_set,1.f,0.3f);
}

void gimbal_output_calc(gimbal_t *gimbal)
{
    if(Mode==Chassis_enable||Mode==Chassis_brake)
    {
        gimbal->pitch_motor.vel_set=PID_calc(&pitch_motor_p,gimbal->pitch_motor.pos,gimbal->pitch_motor.pos_set);
        gimbal->pitch_motor.torque_set=PID_calc(&pitch_motor_v,-QEKF_INS.Gyro[0],gimbal->pitch_motor.vel_set);

        gimbal->yaw_motor.vel_set=PID_calc(&yaw_motor_p,QEKF_INS.YawTotalAngle,gimbal->yaw_motor.pos_set);
        gimbal->yaw_motor.voltage_set=PID_calc(&yaw_motor_v,QEKF_INS.Gyro[2],gimbal->yaw_motor.vel_set);
    }
    else if(Mode==Chassis_disable)
    {
        gimbal->pitch_motor.vel_set=0;
        gimbal->pitch_motor.torque_set=0;
        gimbal->yaw_motor.vel_set=0;
        gimbal->yaw_motor.voltage_set=0;
    }
}
void gimbal_data_send(void)
{
    uint8_t data[8]={0};

    short yaw_send_temp=(short)Gimbal.yaw_motor.voltage_set;
    data[0]=yaw_send_temp>>8;
    data[1]=yaw_send_temp;
    fdcanx_send_data(&hfdcan1,0x1FF,data,8);

    short pitch_send_temp=(short)(Gimbal.pitch_motor.torque_set/(TORQUE_CONSTANT/CUR_CONSTANT));
    data[2]=pitch_send_temp>>8;
    data[3]=pitch_send_temp;
    fdcanx_send_data(&hfdcan3,0x1FE,data,8);
}


void gimbal_move_to_zero(void)
{
    Gimbal.pitch_motor.vel_set=PID_calc(&pitch_motor_p,Gimbal.pitch_motor.pos,Gimbal.pitch_motor.pos_set);
    Gimbal.pitch_motor.torque_set=PID_calc(&pitch_motor_v,-QEKF_INS.Gyro[0],Gimbal.pitch_motor.vel_set);

    Gimbal.yaw_motor.vel_set=PID_calc(&yaw_motor_p,Gimbal.yaw_motor.pos,0);
    LimitMax(Gimbal.yaw_motor.vel_set,3)
    Gimbal.yaw_motor.voltage_set=PID_calc(&yaw_motor_v,QEKF_INS.Gyro[2],Gimbal.yaw_motor.vel_set);

}

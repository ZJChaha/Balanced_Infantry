//
// Created by ZJC on 2024/10/25.
//

#include "All_struct.h"
#include "bsp_can.h"
#include "Chassis_Task.h"
#include "Detect_Task.h"
#include "cmsis_os.h"
#include "Buzzer_Task.h"
#include "Information_Task.h"
#include "dm_motor_drv.h"
#include "imu_temp_ctrl.h"


update_flag_t update_flag;
offline_cnt_t offline_cnt;
uint32_t rc_offline_cnt=0;
uint8_t err_code;
uint8_t defend_code;


void Detect_Task(void const * argument)
{





    for(;;)
    {
        err_code=0;
        offline_detect();

        switch (err_code)
        {
            case remoter_offline:
            {

                rc_offline_cnt++;
                if(Mode==Chassis_enable)
                {
                    buzzer_flag=2;
                }

                //可扩展操作

            }break;
            case joint_motor_offline:
            {
                buzzer_flag=5;
                //可扩展操作
                disable_all_motor_os();
                defend_code=brake;
            }break;
            case hub_motor_offline:
            {
                buzzer_flag=3;
                defend_code=brake;
                //可扩展操作

            }break;
            case imu_offline:
            {
                buzzer_flag=3;
                defend_code=brake;
                //可扩展操作

            }break;
            default:
            {
                rc_offline_cnt=0;
            }
        }


        if(Mode==Chassis_enable)
        {
            for(uint8_t i=0;i<4;i++)
            {
                if(dm_motor[i].para.state==disonline)//关节电机timeout后尝试自救
                {
                    clear_err(&hfdcan1,i,MIT_MODE);
                    osDelay(1);
                    enable_motor_mode(&hfdcan1,i,MIT_MODE);
                    osDelay(1);
                    offline_cnt.timeout_cnt[i]++;
                }
                else
                {offline_cnt.timeout_cnt[i]=0;}

                if(offline_cnt.timeout_cnt[i]>=20)
                {Mode=Chassis_brake;}
            }
        }


        osDelay(5);
    }
}




void offline_detect(void)
{
    if(update_flag.rc==0)
    {offline_cnt.rc_cnt++;}
    else{offline_cnt.rc_cnt=0;}
    if(offline_cnt.rc_cnt>=30)
    {err_code=remoter_offline;}

    if(update_flag.euler==0)
    {offline_cnt.imu_cnt++;}
    else{offline_cnt.imu_cnt=0;}
    if(offline_cnt.imu_cnt>=10)
    {err_code=imu_offline;}

    if(update_flag.hub_left==0)
    {offline_cnt.hub_left_cnt++;}
    else{offline_cnt.hub_left_cnt=0;}
    if(offline_cnt.hub_left_cnt>=20)
    {err_code=hub_motor_offline;}

    if(update_flag.hub_right==0)
    {offline_cnt.hub_right_cnt++;}
    else{offline_cnt.hub_right_cnt=0;}
    if(offline_cnt.hub_right_cnt>=20)
    {err_code=hub_motor_offline;}

    for(uint8_t i=0;i<4;i++)
    {
        if(update_flag.joint_motor[i]==0)
        {offline_cnt.joint_motor_cnt[i]++;}
        else{offline_cnt.joint_motor_cnt[i]=0;}

        if(offline_cnt.joint_motor_cnt[i]>=10)
        {err_code=joint_motor_offline;}
    }

    update_flag.rc=0;
    update_flag.hub_right=0;
    update_flag.hub_left=0;
    update_flag.euler=0;
    update_flag.acc=0;
    update_flag.gyro=0;
    update_flag.joint_motor[Motor1]=0;
    update_flag.joint_motor[Motor2]=0;
    update_flag.joint_motor[Motor3]=0;
    update_flag.joint_motor[Motor4]=0;
}




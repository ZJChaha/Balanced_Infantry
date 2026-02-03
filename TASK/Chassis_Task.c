//
// Created by ZJC on 2024/4/19.
//
#include "Chassis_Task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "Buzzer_Task.h"
#include "All_struct.h"
#include "bsp_can.h"
#include "QuaternionEKF.h"
#include "tim.h"
#include "pid.h"
#include "bsp_usart.h"
#include "bsp_motor.h"
#include "Controller.h"
#include "dm_motor_drv.h"
#include "Information_Task.h"
#include "remote_control.h"
#include "observer.h"
#include "Detect_Task.h"
#include "imu_temp_ctrl.h"
#include "Gimbal_task.h"

short test_current;
uint32_t steady_cnt=0;
uint8_t stand_up_finish=0;
uint8_t leg_in_place_flag=0;

uint8_t theta_init_flag=0;

uint32_t motor_index=0;
float roll_gain=0,yaw_gain=0;
float limit_splits_out=0;
float theta_init[2];
chassis_para Chassis;
float P_K1=0.03f,P_K2=4.9f,P_a=0.8f;


void Chassis_Task(void const * argument)
{
    Mode=Chassis_brake;
    controller_init();
    memset(&Chassis,0, sizeof(Chassis));
    Chassis.yaw_set=rm_imu_data.yaw_all;


    leg_para_init(&leg_para[left]);
    leg_para_init(&leg_para[right]);

    /*********************************获取机体初始状态*********************************/
    vel_observe();//速度观测器
    vmc_para_update(leg_para, dm_motor);
    vmc_calc(&leg_para[left]);//vmc计算
    vmc_calc(&leg_para[right]);
    state_vector_update(&state_vector[left], &leg_para[left]);//状态更新
    state_vector_update(&state_vector[right], &leg_para[right]);
    support_force_calc(&state_vector[left], &leg_para[left]);//支持力解算
    support_force_calc(&state_vector[right], &leg_para[right]);
    /******************************************************************************/
    state_vector[left].set.L0=leg_para[left].L0;
    state_vector[right].set.L0=leg_para[right].L0;
    theta_init[left]=state_vector[left].feedback.theta;
    theta_init[right]=state_vector[right].feedback.theta;


    static portTickType xLastWakeTime;
    const portTickType xFrequency = pdMS_TO_TICKS(PERIOD);

    for(;;)
    {
        lqr_para_update();
        vel_observe();//速度观测器
        vmc_para_update(leg_para, dm_motor);//vmc参数更新
        vmc_calc(&leg_para[left]);//vmc计算
        vmc_calc(&leg_para[right]);
        state_vector_update(&state_vector[left], &leg_para[left]);//状态更新
        state_vector_update(&state_vector[right], &leg_para[right]);
        support_force_calc(&state_vector[left], &leg_para[left]);//支持力解算
        support_force_calc(&state_vector[right], &leg_para[right]);

        chassis_information_update(&Chassis, Mode);//底盘遥控信息更新

         if(Mode==Chassis_brake || defend_code==brake)
        {
            emergency_braking();

        }
         else if(Mode==Chassis_enable)
        {

            if(stand_up_finish==1)
            {
                Chassis_control();
            }
            else
            {
                if(yaw_to_zero==1)
                {Chassis_stand_up();}
            }

        }
        hub_motor_send_data(&Chassis,Mode);//轮毂电机发送函数，一控二
        vTaskDelayUntil(&xLastWakeTime,xFrequency);//延时
    }
}


void TIM6_DAC_IRQHandler(void) //关节电机can发送,1ms发两个包，关节控制频率500hz
{
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    /* USER CODE END TIM6_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

    if(Mode!=Chassis_enable)
    {
        for(uint8_t i=0;i<4;i++)
        {
            Chassis.joint_torque_out[i]=0.0f;
        }
    }
/****************************测试用**********************************/
//    for(uint8_t i=0;i<4;i++)
//    {
//        Chassis.joint_torque_out[i]=0.0f;
//    }
/*******************************************************************/
    if(motor_index%2==0)
    {
        LimitMax(Chassis.joint_torque_out[Motor1],MAX_TORQUE)
        LimitMax(Chassis.joint_torque_out[Motor2],MAX_TORQUE)
        mit_ctrl(&hfdcan1, &dm_motor[Motor1], 0x01, 0.f, 0.f, 0.f, 0.f,Chassis.joint_torque_out[Motor1]);
        mit_ctrl(&hfdcan1, &dm_motor[Motor2], 0x02, 0.f, 0.f, 0.f, 0.f,Chassis.joint_torque_out[Motor2]);
    }
    else
    {
        LimitMax(Chassis.joint_torque_out[Motor3],MAX_TORQUE)
        LimitMax(Chassis.joint_torque_out[Motor4],MAX_TORQUE)
        mit_ctrl(&hfdcan1, &dm_motor[Motor3], 0x03, 0.f, 0.f, 0.f, 0.f,Chassis.joint_torque_out[Motor3]);
        mit_ctrl(&hfdcan1, &dm_motor[Motor4], 0x04, 0.f, 0.f, 0.f, 0.f,Chassis.joint_torque_out[Motor4]);
    }

    motor_index++;


    /* USER CODE END TIM6_DAC_IRQn 1 */
}

void chassis_information_update(chassis_para *para,uint8_t mode)
{
    if(mode==Chassis_enable)
    {
        if(rc_ctrl.rc.s[1]==3)//底盘跟随云台
        {
            para->vel.y=((float)rc_ctrl.rc.ch[1]/660.0f)*MOTION_VEL_MAX;
            if(key_boad_vlue.key_W==1)
            {

                first_order_filter_cali(&speed_slow,1.2f);
                para->vel.y=speed_slow.out;
            }
            else if(key_boad_vlue.key_S==1)
            {

                first_order_filter_cali(&speed_slow,-1.2f);
                para->vel.y=speed_slow.out;
            }

//            para->yaw_set=0;
            state_vector[left ].set.x+=(state_vector[left ].feedback.x_dot-para->vel.y)*PERIOD*0.001f;
            state_vector[right].set.x+=(state_vector[right].feedback.x_dot-para->vel.y)*PERIOD*0.001f;




        }
        else if(rc_ctrl.rc.s[1]==1)//全向移动(标准运动学模式)
        {
            para->vel.x=((float)rc_ctrl.rc.ch[0]/660.0f)*MOTION_VEL_MAX;
            para->vel.y=((float)rc_ctrl.rc.ch[1]/660.0f)*MOTION_VEL_MAX;
            arm_atan2_f32(para->vel.x,para->vel.y,&para->yaw_set);
            float velocity= sqrtf(para->vel.x*para->vel.x+para->vel.y*para->vel.y);

            Find_min_Angle(para->yaw_set,Gimbal.yaw_motor.pos);



            state_vector[left ].set.x+=(state_vector[left ].feedback.x_dot-velocity)*PERIOD*0.001f;
            state_vector[right].set.x+=(state_vector[right].feedback.x_dot-velocity)*PERIOD*0.001f;

        }


        state_vector[left].set.L0-=((float)rc_ctrl.rc.ch[4]/660.0f)*GAIN;
        state_vector[right].set.L0-=((float)rc_ctrl.rc.ch[4]/660.0f)*GAIN;
        limit(&state_vector[left].set.L0,0.35f,0.25f);
        limit(&state_vector[right].set.L0,0.35f,0.25f);//机械限位为0.20~0.40
    }
    else if(mode==Chassis_lock || mode==Chassis_disable)
    {
        para->vel.x=0;
        para->vel.y=0;
        para->vel.z=0;
    }

}

void hub_motor_send_data(chassis_para *para,uint8_t mode)
{
    uint8_t data[8]={0};

    if(mode==Chassis_disable)
    {
        data[0]=0;
        data[1]=0;
        data[3]=0;
        data[4]=0;
    }
    else
    {
        LimitMax(para->hub_torque_out[left],1999)
        LimitMax(para->hub_torque_out[right],1999)   //    1999!
        data[0] = para->hub_torque_out[right];
        data[1] = para->hub_torque_out[right]>>8;
        data[2] = para->hub_torque_out[left];
        data[3] = para->hub_torque_out[left]>>8;
    }

    fdcanx_send_data(&hfdcan2,0x280,data,8);
}

void Chassis_control(void)
{

    leg_length_control(&state_vector[left], &leg_para[left], left);//腿长控制
    leg_length_control(&state_vector[right], &leg_para[right], right);


    lqr_calc(&state_vector[left], &leg_para[left], LQR_K);//lqr计算
    lqr_calc(&state_vector[right], &leg_para[right], LQR_K);

    if(ground_detection(&leg_para[left])&&ground_detection(&leg_para[right]))//离地检测
    {
        leg_para[left].T=0;
        leg_para[right].T=0;
    }
    else
    {
        if(key_boad_vlue.key_E==1)
        {
            Chassis.yaw_set+=0.015f;
        }
        else
        {
            Chassis.yaw_set=0;
        }

        yaw_gain = -PID_calc(&yaw_p_pid_t, zero_process(Gimbal.yaw_motor.pos,Chassis.yaw_set), Chassis.yaw_set);//yaw轴补偿
        leg_para[left].T += yaw_gain;
        leg_para[right].T -= yaw_gain;

        roll_gain = PID_calc(&roll_pid_t, rm_imu_data.euler_angle_fp32[1], Chassis.roll_set);//roll轴补偿
        leg_para[left].F += roll_gain;
        leg_para[right].F -= roll_gain;
    }


    limit_splits_out = PID_calc(&limit_splits,
                                      state_vector[left].feedback.theta - state_vector[right].feedback.theta
                                        ,0);//防劈叉补偿
    leg_para[left].Tp -= limit_splits_out;
    leg_para[right].Tp += limit_splits_out;

    power_control(&power_model);
    convert_torque(&leg_para[left]);
    convert_torque(&leg_para[right]);//计算关节输出扭矩
    /*********************************赋值*****************************/
#if CHASSIS_WORK==1
    Chassis.joint_torque_out[Motor1]=leg_para[left].T1;
    Chassis.joint_torque_out[Motor2]=leg_para[left].T2;
    Chassis.joint_torque_out[Motor3]=-leg_para[right].T2;
    Chassis.joint_torque_out[Motor4]=-leg_para[right].T1;
    Chassis.hub_torque_out[left] =set_torque(-leg_para[left].T);
    Chassis.hub_torque_out[right]=set_torque(leg_para[right].T);
#else
    Chassis.joint_torque_out[Motor1]=0;
    Chassis.joint_torque_out[Motor2]=0;
    Chassis.joint_torque_out[Motor3]=0;
    Chassis.joint_torque_out[Motor4]=0;
    Chassis.hub_torque_out[left] =0;
    Chassis.hub_torque_out[right]=0;
#endif
    /*****************************************************************/
}
void emergency_braking(void)
{
    float vel_temp=(float)rc_ctrl.rc.ch[4]/660.f*50.f;

    leg_para[left].T=-PID_calc(&brake_pid_t,LK_motor[left].vel,vel_temp);
    leg_para[right].T=PID_calc(&brake_pid_t,LK_motor[right].vel,-vel_temp);
    Chassis.hub_torque_out[left] =set_torque(-leg_para[left].T);
    Chassis.hub_torque_out[right]=set_torque(leg_para[right].T);
}

void Chassis_stand_up(void)
{

    if(leg_in_place_flag==0)
    {
        if(theta_init[right]<0)
        {
            if(!(state_vector[right].feedback.theta<1.28f && state_vector[right].feedback.theta>-1.f))
            {state_vector[right].set.L0=0.38f;}
            else {state_vector[right].set.L0=0.25f;}
            if(state_vector[right].feedback.theta<1.38f && state_vector[right].feedback.theta>0)
            {state_vector[right].set.theta_dot = 0;}
            else
            {state_vector[right].set.theta_dot = -2.f;}
            leg_length_control_standup(&state_vector[right], &leg_para[right], right);
            leg_theta_vel_control(&state_vector[right], &leg_para[right],right);
        }
        else
        {
            state_vector[right].set.theta=1.32f;
            leg_theta_pos_control(&state_vector[right], &leg_para[right],right);
        }

        if(theta_init[left]<0)
        {
            if(!(state_vector[left].feedback.theta<1.28f && state_vector[left].feedback.theta>-1.f))
            {state_vector[left].set.L0=0.38f;}
            else {state_vector[left].set.L0=0.25f;}
            if(state_vector[left].feedback.theta<1.38f && state_vector[left].feedback.theta>0)
            {state_vector[left].set.theta_dot = 0;}
            else
            {state_vector[left].set.theta_dot = -2.f;}
            leg_length_control_standup(&state_vector[left], &leg_para[left], left);
            leg_theta_vel_control(&state_vector[left], &leg_para[left],left);
        }
        else
        {
            state_vector[left].set.theta=1.32f;
            leg_theta_pos_control(&state_vector[left], &leg_para[left],left);
        }
    }
    else
    {
        state_vector[left].set.theta=0.f;
        state_vector[right].set.theta=0.f;
        state_vector[left].set.theta_dot=0.f;
        state_vector[right].set.theta_dot=0.f;
        state_vector[left].set.L0=0.23f;
        state_vector[right].set.L0=0.23f;

        leg_length_control_standup(&state_vector[left], &leg_para[left], left);
        leg_length_control_standup(&state_vector[right], &leg_para[right], right);

        leg_theta_vel_control(&state_vector[right], &leg_para[right],right);
        leg_theta_vel_control(&state_vector[left], &leg_para[left],left);

        leg_theta_pos_control(&state_vector[left], &leg_para[left],left);
        leg_theta_pos_control(&state_vector[right], &leg_para[right],right);

        if(state_vector[left].feedback.theta<0.8f && state_vector[right].feedback.theta<0.8f)
        {
            stand_up_finish=1;
            yaw_to_zero=0;
            leg_in_place_flag=0;
            Chassis.yaw_set=rm_imu_data.yaw_all;
            state_vector[left].set.x=0;
            state_vector[right].set.x=0;
            state_vector[left].set.L0=0.25f;
            state_vector[right].set.L0=0.25f;
        }

    }

    if(fabsf(state_vector[left].feedback.theta_dot)<0.2f
       &&fabsf(state_vector[right].feedback.theta_dot)<0.2f
       &&state_vector[left].feedback.theta>1.2f
       &&state_vector[right].feedback.theta>1.2f
       &&state_vector[left].feedback.theta<1.4f
       &&state_vector[right].feedback.theta<1.4f)
    {
        leg_in_place_flag=1;
    }


    convert_torque(&leg_para[left]);
    convert_torque(&leg_para[right]);//计算关节输出扭矩
    /*********************************赋值*****************************/
#if CHASSIS_WORK==1
    Chassis.joint_torque_out[Motor1]=leg_para[left].T1;
    Chassis.joint_torque_out[Motor2]=leg_para[left].T2;
    Chassis.joint_torque_out[Motor3]=-leg_para[right].T2;
    Chassis.joint_torque_out[Motor4]=-leg_para[right].T1;
    Chassis.hub_torque_out[left] =0;
    Chassis.hub_torque_out[right]=0;
#else
    Chassis.joint_torque_out[Motor1]=0;
    Chassis.joint_torque_out[Motor2]=0;
    Chassis.joint_torque_out[Motor3]=0;
    Chassis.joint_torque_out[Motor4]=0;
    Chassis.hub_torque_out[left] =0;
    Chassis.hub_torque_out[right]=0;
#endif
    /*****************************************************************/

}


fp32 Find_min_Angle(fp32 angle1,fp32 angle2)
{
    float err;
    if(angle2 - angle1 < -PI)  err = PI*2 + (angle2 - angle1);
    else if(angle2 - angle1 > PI) err = -PI*2 + (angle2 - angle1);
    else err = angle2 -angle1;

    if(fabsf(err) > PI )
    {
        err = PI * 2.0f - fabsf(err);

    }
    return err;
}

float zero_process(float feedback,float set)
{
    float temp_val;
    if(set-feedback>PI)
    {
        temp_val=feedback+2*PI;
    }
    else if(set-feedback<-PI)
    {
        temp_val=feedback-2*PI;
    }else
    {
        temp_val=feedback;
    }

    return temp_val;
}

//float get_theta_init(void)
//{
//
//
//}


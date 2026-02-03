//
// Created by ZJC on 2024/11/13.
//
#include "observer.h"
#include "kalman_filter.h"
#include "Controller.h"
#include "Chassis_Task.h"
#include "arm_math.h"
#include "bsp_motor.h"

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.002f,
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f,
                           0.0f, 0.5f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                           0.0f,  100.0f};

float vaEstimateKF_K[4];

const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量


float vel_acc[2];
float wheel_vel;

void vel_observe(void)
{
    Bodyframe_to_Worldframe(&rm_imu_data);

    float l_v=(LK_motor[left].vel-LK_motor[right].vel)*radius/2+leg_para[left].L0*state_vector[left].feedback.theta_dot*
              arm_cos_f32(state_vector[left].feedback.theta)+leg_para[left].L0_dot * arm_sin_f32(state_vector[left].feedback.theta);
    float r_v=(LK_motor[left].vel-LK_motor[right].vel)*radius/2+leg_para[right].L0*state_vector[right].feedback.theta_dot*
              arm_cos_f32(state_vector[right].feedback.theta)+leg_para[right].L0_dot * arm_sin_f32(state_vector[right].feedback.theta);

    wheel_vel=(l_v+r_v)/2;
    xvEstimateKF_Update(&vaEstimateKF,rm_imu_data.world_acc_x,wheel_vel);

    state_vector[left].feedback.x_dot=vel_acc[0];
    state_vector[right].feedback.x_dot=vel_acc[0];
}


void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{

    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void Bodyframe_to_Worldframe(rm_imu_data_t *imu)
{

    imu->world_acc_x=imu->accel_fp32[1]*arm_cos_f32(imu->euler_angle_fp32[2])
                     -imu->accel_fp32[2]*arm_sin_f32(imu->euler_angle_fp32[2]);

    imu->world_acc_y=imu->accel_fp32[0]* arm_cos_f32(imu->euler_angle_fp32[1])
                     +imu->accel_fp32[1]*arm_sin_f32(imu->euler_angle_fp32[1])*arm_sin_f32(imu->euler_angle_fp32[2])
                     +imu->accel_fp32[2]*arm_sin_f32(imu->euler_angle_fp32[1])*arm_cos_f32(imu->euler_angle_fp32[2]);

    imu->world_acc_z=-imu->accel_fp32[0]* arm_sin_f32(imu->euler_angle_fp32[1])
                     +imu->accel_fp32[1]*arm_cos_f32(imu->euler_angle_fp32[1])*arm_sin_f32(imu->euler_angle_fp32[2])
                     +imu->accel_fp32[2]*arm_cos_f32(imu->euler_angle_fp32[2])*arm_cos_f32(imu->euler_angle_fp32[1]);
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度

    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}





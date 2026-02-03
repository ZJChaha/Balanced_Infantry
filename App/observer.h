//
// Created by ZJC on 2024/11/13.
//

#ifndef BALANCED_INFANTRY_OBSERVER_H
#define BALANCED_INFANTRY_OBSERVER_H


#include "bsp_can.h"
#include "kalman_filter.h"


extern KalmanFilter_t vaEstimateKF;
extern float wheel_vel;
void vel_observe(void);

void Bodyframe_to_Worldframe(rm_imu_data_t *imu);
void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);





#endif //BALANCED_INFANTRY_OBSERVER_H

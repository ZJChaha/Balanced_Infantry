//
// Created by ZJC on 2024/10/27.
//

#ifndef BALANCED_INFANTRY_GIMBAL_TASK_H
#define BALANCED_INFANTRY_GIMBAL_TASK_H

#include "pid.h"
#include "bsp_motor.h"




extern pid_type_def pitch_motor_p;
extern pid_type_def pitch_motor_v;

extern pid_type_def yaw_motor_p;
extern pid_type_def yaw_motor_v;
extern uint8_t gimbal_init_finish;
extern uint8_t yaw_to_zero;
void Gimbal_Task(void const * argument);

void gimbal_motor_pid_init(void);
void gimbal_information_update(gimbal_t *gimbal);
void gimbal_output_calc(gimbal_t *gimbal);
void gimbal_data_send(void);
void gimbal_move_to_zero(void);

#endif //BALANCED_INFANTRY_GIMBAL_TASK_H

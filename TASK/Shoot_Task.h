//
// Created by ZJC on 2024/12/7.
//

#ifndef BALANCED_INFANTRY_SHOOT_TASK_H
#define BALANCED_INFANTRY_SHOOT_TASK_H


#include "bsp_motor.h"
#include "pid.h"


extern pid_type_def shoot_motor_p;
extern pid_type_def shoot_motor_v;






void Shoot_Task(void const * argument);



void shoot_information_update(shoot_t *para);


void shoot_motor_send_data(void);


#endif //BALANCED_INFANTRY_SHOOT_TASK_H

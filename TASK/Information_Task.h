//
// Created by ZJC on 2024/11/3.
//

#ifndef BALANCED_INFANTRY_INFORMATION_TASK_H
#define BALANCED_INFANTRY_INFORMATION_TASK_H
#include "stdint.h"


#define CHASSIS_WORK 1
//置1底盘工作


enum
{
Chassis_enable=1,
Chassis_lock,
Chassis_disable,
Chassis_brake
};

enum CHASSIS_MOTION_MODE
{
    Follow_gimbal=1,    //跟随云台
    Kinematic,          //标准运动学xyz轴速度
    Small_gyroscope     //啸陀螺
};


extern uint8_t Mode;
extern uint8_t state_flag;
void Information_Task(void const * argument);
void set_mode(uint8_t* m);


#endif //BALANCED_INFANTRY_INFORMATION_TASK_H

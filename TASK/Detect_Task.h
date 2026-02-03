//
// Created by ZJC on 2024/10/25.
//

#ifndef DM_BOARD_CONFIG_DETECT_TASK_H
#define DM_BOARD_CONFIG_DETECT_TASK_H
#include "stdint.h"

typedef enum
{
    remoter_offline=1,
    joint_motor_offline,
    hub_motor_offline,
    imu_offline,

}body_errcode;

typedef enum
{

    brake=3,


}body_defendcode;



typedef struct
{
    uint8_t gyro;
    uint8_t acc;
    uint8_t euler;
    uint8_t hub_left;
    uint8_t hub_right;
    uint8_t rc;
    uint8_t joint_motor[4];

}update_flag_t;

typedef struct
{
    uint32_t rc_cnt;
    uint32_t imu_cnt;
    uint32_t hub_left_cnt;
    uint32_t hub_right_cnt;
    uint32_t joint_motor_cnt[4];
    uint32_t timeout_cnt[4];
}offline_cnt_t;



extern update_flag_t update_flag;
extern uint8_t err_code;

extern offline_cnt_t offline_cnt;
extern uint8_t defend_code;





void Detect_Task(void const * argument);

void offline_detect(void);


#endif //DM_BOARD_CONFIG_DETECT_TASK_H

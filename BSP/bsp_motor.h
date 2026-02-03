//
// Created by ZJC on 2024/10/30.
//

#ifndef BALANCED_INFANTRY_BSP_MOTOR_H
#define BALANCED_INFANTRY_BSP_MOTOR_H

#include "stdint.h"

#define K_CUR_T 0.0025781248f
#define MAX_TORQUE 30.f
#define CUR_CONSTANT 5461.33333f
#define TORQUE_CONSTANT 0.741f

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }



//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern motor_measure_t motor_gimbal[2];//云台
extern motor_measure_t motor_shoot;


typedef struct
{
    /*************************原生数据**************************/

    uint8_t cmd; // 命令字节
    int8_t Temperature; // 电机温度， 1℃/LSB
    int16_t Torque_iq; // 转矩电流值iq,范围-2048~2048，对应实际转矩电流范围-16.5A~16.5A
    int16_t Speed; // 电机转速, 1dps/LSB
    uint16_t Location; // 编码器的数值，范围 0~65535(16bit)
    /*************************扩展数据**************************/
    float Torque;//电机实际转矩,单位:NM
    float vel;//速度，单位:rad/s
    float pos;//位置，单位:rad
    float last_pos;//上一时刻位置，单位:rad
    int turns;//圈数
    float muti_turns_pos;//多圈位置
} LKmotorFeedbackTypeDef;

extern LKmotorFeedbackTypeDef LK_motor[2];



typedef struct
{
    float pos;//位置
    float last_pos;//上一时刻位置
    float turns;//圈数
    float muti_turns_pos;//多圈位置
    float final_pos;//输出轴端位置
    float vel;//速度
    float torque;//力矩
    float temparature;//温度
    uint8_t err_code;//错误码

    float pos_set;
    float vel_set;
    float torque_set;
    float voltage_set;

}test_motor_t;


typedef struct
{
    test_motor_t yaw_motor;
    test_motor_t pitch_motor;
}gimbal_t;

extern gimbal_t Gimbal;

typedef struct
{

    test_motor_t shoot_motor;

    int rub_wheel_state;
    uint8_t rub_cnt;//通过累加计数降低发送频率
    uint8_t fire;

}shoot_t;

extern shoot_t Shoot;

void can_LKmotor_data_decode(uint8_t *buf , LKmotorFeedbackTypeDef *str);
float convert(uint16_t raw_angle);
short set_torque(float T);
void muti_turns_pos_calc(test_motor_t *motor);
float NormalizeAngle(const float angle);





#endif //BALANCED_INFANTRY_BSP_MOTOR_H

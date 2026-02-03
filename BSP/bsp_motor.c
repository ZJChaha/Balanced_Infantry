//
// Created by ZJC on 2024/10/30.
//

#include "bsp_motor.h"
#include "bsp_can.h"
#include "string.h"
#include "arm_math.h"


motor_measure_t motor_gimbal[2];//云台
motor_measure_t motor_shoot;//拨弹
LKmotorFeedbackTypeDef LK_motor[2];//轮毂









short set_torque(float T)
{
    return (short)(T/K_CUR_T);
}




float convert(uint16_t raw_angle)
{
    float val;
    val=(float)raw_angle*2*PI/8191.0f-PI;
    return val;
}


/**
* @brief 瓴控电机电机数据解码、转移
* @param[in] CanRxMsg，符合电机类的结构体变量
* @retval 无
*/
void can_LKmotor_data_decode(uint8_t *buf , LKmotorFeedbackTypeDef *str)
{
    /*************************原生数据**************************/
    str->cmd = buf[0];
    str->Temperature = buf[1];
    str->Torque_iq = (buf[3]<<8)|buf[2];
    str->Speed = (buf[5]<<8)|buf[4];
    str->Location = (buf[7]<<8)|buf[6];
    /*************************二次数据**************************/
    str->pos=(float)str->Location*0.00009587526f-PI;
    str->Torque=(float)str->Torque_iq*K_CUR_T;
    str->vel=(float)str->Speed*0.0175f;
}



void muti_turns_pos_calc(test_motor_t *motor)
{
    float diff = motor->pos - motor->last_pos;
    if(diff < -PI)
    {
        motor->turns++;
    }
    else if(diff>PI)
    {
        motor->turns--;
    }
    motor->muti_turns_pos = motor->pos + motor->turns*2*PI;
}


float NormalizeAngle(const float angle) {
    float a = fmodf(angle + PI, 2.f * PI);
    if (a < 0.0f) {
        a += (2.0f * PI);
    }
    return a - PI;
}



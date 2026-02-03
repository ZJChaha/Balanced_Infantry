//
// Created by ZJC on 2024/4/19.
//

#ifndef DM_BOARD_CONFIG_CHASSIS_TASK_H
#define DM_BOARD_CONFIG_CHASSIS_TASK_H
#include "stdint.h"
#include "bsp_can.h"
#include "pid.h"


#define Mg     220.f    //机体质量
#define mg     25.f     //轮毂质量
#define radius 0.065f  //轮毂半径
#define PERIOD 2

#define MOTION_VEL_MAX 1.8f
#define ROTATION_VEL_MAX 0
#define GAIN 0.001f
#define ROLL_RANGE 0.3f

//#define P_K1 0
//#define P_K2 0
//#define P_a 0
extern float P_K1,P_K2,P_a;




extern float length_gain;
extern float roll_gain,yaw_gain,x_init;
extern uint8_t stand_up_finish;
extern float theta_init[2];
extern float limit_splits_out;
enum
{
    left,
    right
};

typedef struct
{
    float x;
    float y;
    float z;
}velocity_para;



typedef struct
{
    float relative_yaw_pos;
    float yaw_set;
    float roll_set;
    float joint_torque_out[4];
    short hub_torque_out[2];
    velocity_para vel;

}chassis_para;

extern chassis_para Chassis;


void Chassis_Task(void const * argument);

void chassis_information_update(chassis_para *para,uint8_t mode);

void hub_motor_send_data(chassis_para *para,uint8_t mode);



void Chassis_control(void);
void emergency_braking(void);
void Chassis_stand_up(void);
fp32 Find_min_Angle(fp32 angle1,fp32 angle2);
float zero_process(float feedback,float set);

#endif //DM_BOARD_CONFIG_CHASSIS_TASK_H

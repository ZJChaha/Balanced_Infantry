//
// Created by ZJC on 2024/10/30.
//

#ifndef BALANCED_INFANTRY_CONTROLLER_H
#define BALANCED_INFANTRY_CONTROLLER_H

#include "dm_motor_drv.h"
#include "pid.h"

extern pid_type_def roll_pid_t;
extern pid_type_def yaw_p_pid_t;
extern pid_type_def leg_length[2];
extern pid_type_def limit_splits;
extern pid_type_def brake_pid_t;
extern pid_type_def theta_p_pid_t[2];
extern pid_type_def theta_v_pid_t[2];

extern float LQR_K[12];
extern float lqr_para_temp[48];
extern int lqr_para_update_flag;
typedef struct
{
    float theta;
    float theta_dot;
    float theta_init;
    float phi;
    float phi_dot;
    float x;
    float x_dot;
}feedback_t;

typedef struct
{
    float theta;
    float theta_dot;
//    float phi;
//    float phi_dot;
    float x;
    float x_dot;
    float L0;
}set_t;



typedef struct
{
    feedback_t feedback;
    set_t set;

}state_vector_t;
extern state_vector_t state_vector[2];

typedef struct
{
    float phi0, phi1, phi2, phi3, phi4, last_phi0, phi0_dot,phi1_dot, phi2_dot, phi4_dot;
    float L0, l1, l2, l3, l4, l5,l_bd,L0_dot,last_L0,last_L0_dot,L0_2dot;
    float T1, T2,T;
    float A0, B0, C0;
    float Xb, Xc, Xd, Yb, Yc, Yd;
    float Xb_dot, Xd_dot, Yb_dot, Yd_dot;
    float final_matrix[2][2];
    float final_matrix_inv[2][2];
    float F,Tp;
    float F_feedback,Tp_feedback,P;
    float T1_feedback,T2_feedback;
    float theta_2dot,last_theta_dot;
    float Zw_2dot;

}leg_para_t;
extern leg_para_t leg_para[2];

typedef struct
{

    float power_predict[2];//单个电机的预测功率
    float power_limit[2];//单个电机被限制后的功率
    float power_predict_all;//预测总功率
    float power_MAX;//功率限制值
    float K_scale;//缩放系数
    float Torque_limit[2];//最终的输出力矩
    float real_power;
}power_model_t;

typedef struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;



extern power_model_t power_model;

extern float speed_slow_k[1];
extern first_order_filter_type_t speed_slow;

void controller_init(void);
void state_vector_update(state_vector_t *vector,leg_para_t *l);
float get_K(float *coe,float length);
void lqr_calc(state_vector_t *vector,leg_para_t *l,float *gain_K);
void leg_para_init(leg_para_t *l);
void vmc_para_update(leg_para_t *l,dm_motor_t *m);
void vmc_calc(leg_para_t *l);
void leg_length_control(state_vector_t *vector,leg_para_t *l,uint8_t flag);
void leg_length_control_standup(state_vector_t *vector,leg_para_t *l,uint8_t flag);
void leg_theta_pos_control(state_vector_t *vector,leg_para_t *l,uint8_t flag);
void leg_theta_vel_control(state_vector_t *vector,leg_para_t *l,uint8_t flag);
void convert_torque(leg_para_t *l);
void support_force_calc(state_vector_t *vector,leg_para_t *l);
int ground_detection(leg_para_t *l);
void limit(float *value,float max,float min);
void lqr_para_update(void);
void power_control(power_model_t* para);

//一阶滤波初始化
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);
//一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);

#endif //BALANCED_INFANTRY_CONTROLLER_H

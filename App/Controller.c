//
// Created by ZJC on 2024/10/30.
//
#include "Controller.h"
#include "arm_math.h"
#include "bsp_can.h"
#include "bsp_motor.h"
#include "Chassis_Task.h"
#include "observer.h"
#include "kalman_filter.h"
#include "pid.h"
#include "remote_control.h"


float Polynomial_coefficients[12][4]={
        {-353.6436f,386.4815f,-191.0637f,-3.1700f},
        {-5.1393f,5.3746f,-16.1808f,0.4858f},
        {-103.0930f,101.0350f,-35.5083f,-2.5713f},
        {-93.2334f,92.7907f,-35.8785f,-2.3021f},
        {-238.8995f,273.7449f,-123.1140f,26.3224f},
        {-22.2970f,26.8136f,-12.8820f,3.1760f},
        {116.7303f,-64.6478f,-13.0604f,21.3236f},
        {29.1776f,-28.1749f,9.3805f,0.9604f},
        {-162.9675f,185.1784f,-82.1857f,16.9442f},
        {-136.5547f,156.5784f,-70.5491f,15.2721f},
        {602.4523f,-592.5158f,209.5696f,12.5999f},
        {72.3055f,-72.0846f,26.0907f,0.2773f}
        //[1 1 800 200 6000 1]
        //[15 0;0 4]
};
float lqr_para_temp[48]={0};
int lqr_para_update_flag=0;
pid_type_def yaw_p_pid_t;
pid_type_def yaw_v_pid_t;
pid_type_def roll_pid_t;
pid_type_def brake_pid_t;
pid_type_def theta_p_pid_t[2];
pid_type_def theta_v_pid_t[2];
float yaw_p_pid_para[3]={5.2f,0,360.f};
float roll_pid_para[3]={500.f,0,10000.f};
float brake_pid_para[3]={0.4f,0,0};
float theta_p_pid_para[3]={8.f,0,1.f};
float theta_v_pid_para[3]={12.f,1.5f,0};

power_model_t power_model;
state_vector_t state_vector[2];
leg_para_t leg_para[2];
pid_type_def leg_length[2];
float leglength_pid_para[3]={1300.f,0,50000.f};//1300.f,0,55000.f
pid_type_def limit_splits;
float limit_splits_pid_para[3]={25.f,0,1.f};
float LQR_K[12]={0};
float speed_slow_k[1] = {0.5f};
first_order_filter_type_t speed_slow;


void controller_init(void)
{
    memset(&state_vector[left],0,sizeof(state_vector[left]));
    memset(&state_vector[right],0,sizeof(state_vector[right]));
    memset(&leg_para[left],0,sizeof(leg_para[left]));
    memset(&leg_para[right],0,sizeof(leg_para[right]));
    memset(&leg_length[left],0, sizeof(leg_length[left]));
    memset(&leg_length[right],0, sizeof(leg_length[right]));
    PID_init(&yaw_p_pid_t,PID_POSITION,yaw_p_pid_para,4,0);
    PID_init(&leg_length[left],PID_POSITION,leglength_pid_para,240,0);
    PID_init(&leg_length[right],PID_POSITION,leglength_pid_para,240,0);
    PID_init(&limit_splits,PID_POSITION,limit_splits_pid_para,20,0);
    PID_init(&roll_pid_t,PID_POSITION,roll_pid_para,130,0);
    PID_init(&brake_pid_t,PID_POSITION,brake_pid_para,2.f,0);
    PID_init(&theta_p_pid_t[left],PID_POSITION,theta_p_pid_para,3.f,0);
    PID_init(&theta_p_pid_t[right],PID_POSITION,theta_p_pid_para,3.f,0);
    PID_init(&theta_v_pid_t[left],PID_POSITION,theta_v_pid_para,30.f,0);
    PID_init(&theta_v_pid_t[right],PID_POSITION,theta_v_pid_para,30.f,0);

    first_order_filter_init(&speed_slow,0.002f,speed_slow_k);
    xvEstimateKF_Init(&vaEstimateKF);

//    state_vector[left].set.L0=0.25f;
//    state_vector[right].set.L0=0.25f;

}

void state_vector_update(state_vector_t *vector,leg_para_t *l)
{

    vector->feedback.phi=rm_imu_data.euler_angle_fp32[2];
    vector->feedback.phi_dot=rm_imu_data.gyro_fp32[0];

    float temp=fmodf(-PI/2-vector->feedback.phi-l->phi0,2*PI);
    vector->feedback.theta=(temp>0?temp:(temp+2*PI))-PI;

    vector->feedback.theta_dot=-vector->feedback.phi_dot-l->phi0_dot;
    //vector->feedback.x=(LK_motor[left].muti_turns_pos-LK_motor[right].muti_turns_pos)*radius/2-x_init;

    l->theta_2dot=(vector->feedback.theta_dot-l->last_theta_dot)/(PERIOD*0.001f);
    l->last_theta_dot=vector->feedback.theta_dot;
}



float get_K(float *coe,float len)
{
    return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}

void lqr_calc(state_vector_t *vector,leg_para_t *l,float *gain_K)
{
    for(uint8_t i=0;i<12;i++)
    {
        gain_K[i]=get_K(&Polynomial_coefficients[i][0],l->L0);
    }


        if(ground_detection(l))
        {
            l->T=0;
            l->Tp=vector->feedback.theta*gain_K[6]
                  +vector->feedback.theta_dot*gain_K[7];
        }
        else
        {
            l->T = (vector->feedback.theta-0.12f) * gain_K[0]
                   + vector->feedback.theta_dot * gain_K[1]
                   + vector->set.x * gain_K[2]
                   + vector->feedback.x_dot * gain_K[3]
                   + (vector->feedback.phi) * gain_K[4]
                   + vector->feedback.phi_dot * gain_K[5];

            l->Tp = (vector->feedback.theta-0.12f) * gain_K[6]
                    + vector->feedback.theta_dot * gain_K[7]
                    + vector->set.x * gain_K[8]
                    + vector->feedback.x_dot * gain_K[9]
                    + (vector->feedback.phi) * gain_K[10]
                    + vector->feedback.phi_dot * gain_K[11];
        }


}

void leg_para_init(leg_para_t *l)
{
    l->l1=0.215f;
    l->l2=0.26f;
    l->l3=0.26f;
    l->l4=0.215f;
    l->l5=0.f;

}
void vmc_para_update(leg_para_t *l,dm_motor_t *m)
{
    l[right].phi1=m[Motor3].para.pos+PI;//单位：rad
    l[right].phi1_dot=m[Motor3].para.vel;//单位：rad/s
    l[right].T1_feedback=m[Motor3].para.tor;//单位：NM
    l[right].phi4=m[Motor4].para.pos;
    l[right].phi4_dot=m[Motor4].para.vel;
    l[right].T2_feedback=m[Motor4].para.tor;

    l[left].phi1=PI-m[Motor2].para.pos;//单位：rad
    l[left].phi1_dot=-m[Motor2].para.vel;//单位：rad/s
    l[left].T1_feedback=-m[Motor2].para.tor;
    l[left].phi4=-m[Motor1].para.pos;
    l[left].phi4_dot=-m[Motor1].para.vel;
    l[left].T2_feedback=-m[Motor1].para.tor;

}
void vmc_calc(leg_para_t *l)
{
    l->Xb = l->l1 * arm_cos_f32(l->phi1);
    l->Xd = l->l5 + l->l4 * arm_cos_f32(l->phi4);
    l->Yb = l->l1 * arm_sin_f32(l->phi1);
    l->Yd = l->l4 * arm_sin_f32(l->phi4);
    arm_sqrt_f32((l->Xd - l->Xb)*((l->Xd - l->Xb)) + (l->Yd - l->Yb)*(l->Yd - l->Yb),&l->l_bd);

    l->A0 = 2 * l->l2 * (l->Xd - l->Xb);
    l->B0 = 2 * l->l2 * (l->Yd - l->Yb);
    l->C0 = l->l2*l->l2+l->l_bd*l->l_bd-l->l3*l->l3;

    float sqrt_temp_1,phi2_half;
    arm_sqrt_f32(l->A0*l->A0+l->B0*l->B0-l->C0*l->C0,&sqrt_temp_1);
    arm_atan2_f32(l->B0 + sqrt_temp_1,l->A0 + l->C0,&phi2_half);
    l->phi2=2*phi2_half;
    l->phi3 = PI - l->phi2  + l->phi1 + l->phi4;

    l->Xc=l->l1*arm_cos_f32(l->phi1)+l->l2*arm_cos_f32(l->phi2);
    l->Yc=l->l1*arm_sin_f32(l->phi1)+l->l2*arm_sin_f32(l->phi2);


    arm_sqrt_f32((l->Xc-l->l5/2)*(l->Xc-l->l5/2)+l->Yc*l->Yc,&l->L0); //L0
    arm_atan2_f32(l->Yc,l->Xc-l->l5/2,&l->phi0);                      //phi0
    l->phi0_dot=(l->phi0-l->last_phi0)/(PERIOD*0.001f);               //phi0_dot
    l->last_phi0=l->phi0;
    l->L0_dot=(l->L0-l->last_L0)/(PERIOD*0.001f);                     //L0_dot
    l->last_L0=l->L0;

    l->L0_2dot=(l->L0_dot-l->last_L0_dot)/(PERIOD*0.001f);            //L0_2dot
    l->last_L0_dot=l->L0_dot;


    l->final_matrix[0][0]=(l->l1*arm_sin_f32(l->phi0-l->phi3)* arm_sin_f32(l->phi1-l->phi2))/ arm_sin_f32(l->phi3-l->phi2);
    l->final_matrix[0][1]=(l->l1*arm_cos_f32(l->phi0-l->phi3)* arm_sin_f32(l->phi1-l->phi2))/(l->L0*arm_sin_f32(l->phi3-l->phi2));
    l->final_matrix[1][0]=(l->l4*arm_sin_f32(l->phi0-l->phi2)* arm_sin_f32(l->phi3-l->phi4))/ arm_sin_f32(l->phi3-l->phi2);
    l->final_matrix[1][1]=(l->l4*arm_cos_f32(l->phi0-l->phi2)* arm_sin_f32(l->phi3-l->phi4))/(l->L0*arm_sin_f32(l->phi3-l->phi2));

    l->final_matrix_inv[0][0]= -arm_cos_f32(l->phi0-l->phi2)/(l->l1*arm_sin_f32(l->phi1-l->phi2));
    l->final_matrix_inv[0][1]= arm_cos_f32(l->phi0-l->phi3)/(l->l4*arm_sin_f32(l->phi3-l->phi4));
    l->final_matrix_inv[1][0]= (l->L0*arm_sin_f32(l->phi0-l->phi2))/(l->l1*arm_sin_f32(l->phi1-l->phi2));
    l->final_matrix_inv[1][1]= -(l->L0*arm_sin_f32(l->phi0-l->phi3))/(l->l4*arm_sin_f32(l->phi3-l->phi4));


}


void leg_length_control(state_vector_t *vector,leg_para_t *l,uint8_t flag)
{
    if(vector->feedback.theta<1.3f && vector->feedback.theta>-1.3f)
    {
        l->F = (Mg/2) / arm_cos_f32(vector->feedback.theta) +
               PID_calc(&leg_length[flag], l->L0, vector->set.L0);//触地时前馈补偿机体重力
    }

}

void leg_length_control_standup(state_vector_t *vector,leg_para_t *l,uint8_t flag)//倒地起身时的腿长控制
{
        l->F=-mg*arm_cos_f32(vector->feedback.theta)+PID_calc(&leg_length[flag], l->L0, vector->set.L0);
}


void leg_theta_pos_control(state_vector_t *vector,leg_para_t *l,uint8_t flag)//摆杆角度控制，仅在未起立的情况下使用
{
//过零处理，重力补偿
    float feedforward=-mg*arm_sin_f32(vector->feedback.theta)*l->L0;
    float temp_val;


    if(vector->set.theta-vector->feedback.theta>PI)
    {
        temp_val=vector->feedback.theta+2*PI;
    }
    else if(vector->set.theta-vector->feedback.theta<-PI)
    {
        temp_val=vector->feedback.theta-2*PI;
    }else
    {
        temp_val=vector->feedback.theta;
    }

    l->Tp=feedforward-PID_calc(&theta_v_pid_t[flag],vector->feedback.theta_dot,
                               PID_calc(&theta_p_pid_t[flag],temp_val,vector->set.theta));
}

void leg_theta_vel_control(state_vector_t *vector,leg_para_t *l,uint8_t flag)//摆杆速度控制，仅在未起立的情况下使用
{
    float feedforward=-mg*arm_sin_f32(vector->feedback.theta)*l->L0;
    l->Tp=feedforward-PID_calc(&theta_v_pid_t[flag],vector->feedback.theta_dot,vector->set.theta_dot);
}



void convert_torque(leg_para_t *l)
{

    l->T1=l->final_matrix[0][0]*l->F-l->final_matrix[0][1]*l->Tp;
    l->T2=l->final_matrix[1][0]*l->F-l->final_matrix[1][1]*l->Tp;
}

void support_force_calc(state_vector_t *vector,leg_para_t *l)
{
    l->F_feedback=l->final_matrix_inv[0][0]*l->T1_feedback+l->final_matrix_inv[0][1]*l->T2_feedback;
    l->Tp_feedback=l->final_matrix_inv[1][0]*l->T1_feedback+l->final_matrix_inv[1][1]*l->T2_feedback;
    l->P=l->F_feedback* arm_cos_f32(vector->feedback.theta)+l->Tp_feedback*arm_sin_f32(vector->feedback.theta)/l->L0;


    l->Zw_2dot=rm_imu_data.world_acc_z-l->L0_2dot*arm_cos_f32(vector->feedback.theta)
            +2*l->L0_dot*vector->feedback.theta_dot* arm_sin_f32(vector->feedback.theta)
            +l->L0*l->theta_2dot* arm_sin_f32(vector->feedback.theta)
            +l->L0*vector->feedback.theta_dot*vector->feedback.theta_dot* arm_cos_f32(vector->feedback.theta);


}


void power_control(power_model_t* para)
{

    para->power_predict[left]=LK_motor[left].vel*LK_motor[left].Torque
            +P_K1*LK_motor[left].vel*LK_motor[left].vel
            +P_K2*leg_para[left].T*leg_para[left].T
            +P_a;
    para->power_predict[right]=LK_motor[right].vel*LK_motor[right].Torque
                           +P_K1*LK_motor[right].vel*LK_motor[right].vel
                           +P_K2*leg_para[right].T*leg_para[right].T
                           +P_a;
    para->power_predict_all=para->power_predict[0]+para->power_predict[1];



    if(para->power_predict_all>para->power_MAX)
    {

        for (int i = 0; i < 2; ++i)
        {
            if(para->power_predict[i]>0)
            {
                para->K_scale=para->power_MAX/para->power_predict_all;
                para->power_limit[i]=para->power_predict[i]*para->K_scale;
                float b=LK_motor[i].vel;
                float c=P_K1*LK_motor[i].vel+P_a-para->power_limit[i];
                if(leg_para[i].T>=0)
                    para->Torque_limit[i]=(-b+ sqrtf(b*b-4*P_K2*c))/2*P_K2;
                else
                    para->Torque_limit[i]=(-b- sqrtf(b*b-4*P_K2*c))/2*P_K2;
            }
            else
            {para->Torque_limit[i]=leg_para[i].T;}
        }
    }
    else
    {
        para->Torque_limit[left]=leg_para[left].T;
        para->Torque_limit[right]=leg_para[right].T;
    }

}







int ground_detection(leg_para_t *l)
{
    if(l->P<20.f)
    {
        return 1;
    }else
    {
        return 0;
    }
}

void limit(float *value,float max,float min)
{
    if(max>min)
    {
        if(*value>=max)
        {*value=max;}

        if(*value<min)
        {*value=min;}
    }
}

void lqr_para_update(void)
{
    if(lqr_para_update_flag==1)
    {
        for (int i = 0; i < 12; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                Polynomial_coefficients[i][j]=lqr_para_temp[i*4+j];
            }
        }
        lqr_para_update_flag=0;
    }
}

/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
            first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}




#include "vofa.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "math.h"
#include "usart.h"
#include "All_struct.h"
#include "Chassis_Task.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_motor.h"
#include "Controller.h"
#include "remote_control.h"
#include "observer.h"
#include "Detect_Task.h"

#include "QuaternionEKF.h"
#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:		   void
* @retval:     void
* @details:    �������ݸ���λ��
***********************************************************************
**/
void vofa_start(void)
{
    vofa_demo();		// demoʾ��

//	vofa_sendframetail();
}

/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    �޸�ͨ�Ź��ߣ�USART����USB
***********************************************************************
**/
void vofa_transmit(uint8_t* buf, uint16_t len)
{
	HAL_UART_Transmit_DMA(&huart7, (uint8_t *)buf, len);
//    CDC_Transmit_HS((uint8_t *)buf, len);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: ���ݱ�� data: ����
* @retval:     void
* @details:    ���������ݲ�ֳɵ��ֽ�
***********************************************************************
**/
void vofa_send_data(uint8_t num, float data)
{
    send_buf[cnt++] = byte0(data);
    send_buf[cnt++] = byte1(data);
    send_buf[cnt++] = byte2(data);
    send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL
* @retval     void
* @details:   �����ݰ�����֡β
***********************************************************************
**/
void vofa_sendframetail(void)
{
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;


    vofa_transmit((uint8_t *)send_buf, cnt);
    cnt = 0;
}
/**
***********************************************************************
* @brief      vofa_demo(void)
* @param      NULL
* @retval     void
* @details:   demoʾ��
***********************************************************************
**/
void vofa_demo(void)
{

//    vofa_send_data(0,rm_imu_data.euler_angle_fp32[0]);
//	vofa_send_data(1,rm_imu_data.euler_angle_fp32[1]);
//    vofa_send_data(2,rm_imu_data.euler_angle_fp32[2]);
//    vofa_send_data(3,rm_imu_data.accel_fp32[0]);
//    vofa_send_data(4,rm_imu_data.accel_fp32[1]);
//    vofa_send_data(5,rm_imu_data.accel_fp32[2]);
//    vofa_send_data(6,rm_imu_data.gyro_fp32[0]);
//    vofa_send_data(7,rm_imu_data.gyro_fp32[1]);
//    vofa_send_data(8,rm_imu_data.gyro_fp32[2]);



//    vofa_send_data(3,Gimbal.pitch_motor.pos);
//    vofa_send_data(4,Gimbal.pitch_motor.torque_set);
//    vofa_send_data(5,QEKF_INS.YawTotalAngle);
//    vofa_send_data(6,QEKF_INS.Gyro[0]);
//    vofa_send_data(6,QEKF_INS.Gyro[2]);

    vofa_send_data(0,dm_motor[0].para.tor);
    vofa_send_data(1,dm_motor[1].para.tor);
    vofa_send_data(2,dm_motor[2].para.tor);
    vofa_send_data(3,dm_motor[3].para.tor);

    vofa_send_data(4,dm_motor[0].para.pos);
    vofa_send_data(5,dm_motor[1].para.pos);
    vofa_send_data(6,dm_motor[2].para.pos);
    vofa_send_data(7,dm_motor[3].para.pos);

//    vofa_send_data(0,state_vector[right].set.L0);
//    vofa_send_data(1,leg_para[right].L0);
//    vofa_send_data(2,leg_para[left].L0);
//    vofa_send_data(4,leg_para[right].F);
//    vofa_send_data(5,leg_para[left].F);
//    vofa_send_data(0,power_model.power_predict_all);
//    vofa_send_data(3,power_model.real_power);
//    vofa_send_data(6,leg_para[right].Tp);
//    vofa_send_data(7,leg_para[left].Tp);
//    vofa_send_data(5,state_vector[right].feedback.theta);
//    vofa_send_data(6,state_vector[left].feedback.theta);
    vofa_send_data(7,(float)err_code);




//    vofa_send_data(4,leg_para[left].T1);
//    vofa_send_data(5,leg_para[left].T2);
//    vofa_send_data(6,rm_imu_data.world_acc_z);
//    vofa_send_data(7,leg_para[left].Zw_2dot);
//    vofa_send_data(8,leg_para[right].Zw_2dot);

//    vofa_send_data(10,(float)err_code);



    vofa_sendframetail();
}












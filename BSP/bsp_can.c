#include <string.h>
#include "bsp_can.h"
#include "Buzzer_task.h"
#include "All_struct.h"
#include "Chassis_Task.h"
#include "Detect_Task.h"
#include "bsp_motor.h"
#include "Detect_Task.h"
#include "arm_math.h"
#include "dm_motor_drv.h"
#include "bsp_super_cap.h"
#include "Controller.h"
rm_imu_data_t rm_imu_data;



/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/




void can_bsp_init(void)
{
    can_filter_init();
    HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x10;
    fdcan_filter.FilterID2 = 0x206;
    HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);


    FDCAN_FilterTypeDef fdcan2_filter;

    fdcan2_filter.IdType = FDCAN_STANDARD_ID;
    fdcan2_filter.FilterIndex = 0;
    fdcan2_filter.FilterType = FDCAN_FILTER_RANGE;
    fdcan2_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan2_filter.FilterID1 = 0x100;
    fdcan2_filter.FilterID2 = 0x500;
    HAL_FDCAN_ConfigFilter(&hfdcan2,&fdcan2_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);


    FDCAN_FilterTypeDef fdcan3_filter;

    fdcan3_filter.IdType = FDCAN_STANDARD_ID;
    fdcan3_filter.FilterIndex = 0;
    fdcan3_filter.FilterType = FDCAN_FILTER_RANGE;
    fdcan3_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan3_filter.FilterID1 = 0x100;
    fdcan3_filter.FilterID2 = 0x500;
    HAL_FDCAN_ConfigFilter(&hfdcan3,&fdcan3_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1);

}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;																// 标准ID
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// 数据帧
    TxHeader.DataLength = len;																		// 发送数据长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// 设置错误状态指示
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// 不开启可变波特率
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// 普通CAN格式
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// 用于发送事件FIFO控制, 不存储
    TxHeader.MessageMarker = 0x00; 			// 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0到0xFF

    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK)
        return 1;//发送
    return 0;
}

/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	供用户调用的接收弱函数
************************************************************************
**/
uint8_t rx_data1[8] = {0};
void fdcan1_rx_callback(void)
{
    fdcanx_receive(&hfdcan1, rx_data1);

}
uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{
    fdcanx_receive(&hfdcan2, rx_data2);
}
uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
    fdcanx_receive(&hfdcan3, rx_data3);
}


/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/

uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef fdcan_RxHeader;
    if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &fdcan_RxHeader, buf)!=HAL_OK)
        return 0;//接收数据

    if(hfdcan==&hfdcan1)
    {
        if(fdcan_RxHeader.DataLength==8)
        {
            switch (fdcan_RxHeader.Identifier)
            {
                case 0x11:
                {
                    dm_motor_fbdata(&dm_motor[Motor1],buf);
                    update_flag.joint_motor[Motor1]=1;

                }break;
                case 0x22:
                {
                    dm_motor_fbdata(&dm_motor[Motor2],buf);
                    update_flag.joint_motor[Motor2]=1;

                }break;
                case 0x33:
                {
                    dm_motor_fbdata(&dm_motor[Motor3],buf);
                    update_flag.joint_motor[Motor3]=1;

                }break;
                case 0x44:
                {
                    dm_motor_fbdata(&dm_motor[Motor4],buf);
                    update_flag.joint_motor[Motor4]=1;

                }break;
                case 0x205://yaw
                {
                    get_motor_measure(&motor_gimbal[0],buf)
                    Gimbal.yaw_motor.pos=NormalizeAngle(convert(motor_gimbal[0].ecd)-2.11f);
                    muti_turns_pos_calc(&Gimbal.yaw_motor);
                    Gimbal.yaw_motor.vel=(float)motor_gimbal[0].speed_rpm*0.10472f;
                    Gimbal.yaw_motor.torque=(float)motor_gimbal[0].given_current*TORQUE_CONSTANT/CUR_CONSTANT;
                    Gimbal.yaw_motor.last_pos=Gimbal.yaw_motor.pos;


                }break;
            }
        }
    }
    else if(hfdcan==&hfdcan2)
    {

        switch (fdcan_RxHeader.Identifier)
        {
            case 0x141:
            {
                can_LKmotor_data_decode(buf,&LK_motor[right]);
                update_flag.hub_right=1;
            }break;
            case 0x142:
            {
                can_LKmotor_data_decode(buf,&LK_motor[left]);
                update_flag.hub_left=1;
            }break;
            case RM_IMU_PARAM_ID:
            {
                rm_imu_data.accel_rangle = buf[0] &0x0F;
                rm_imu_data.gyro_rangle = (buf[0] &0xF0) >> 4;
                rm_imu_data.sensor_control_temperature = buf[2];
                rm_imu_data.imu_sensor_rotation = buf[3] & 0x1F;
                rm_imu_data.ahrs_rotation_sequence = (buf[3] & 0xE0) >> 5;
                rm_imu_data.quat_euler = buf[4] & 0x01;
                switch(rm_imu_data.gyro_rangle)
                {
                    case 0: rm_imu_data.gyro_sen = GYRO_2000_SEN; break;
                    case 1: rm_imu_data.gyro_sen = GYRO_1000_SEN; break;
                    case 2: rm_imu_data.gyro_sen = GYRO_500_SEN; break;
                    case 3: rm_imu_data.gyro_sen = GYRO_250_SEN; break;
                    case 4: rm_imu_data.gyro_sen = GYRO_125_SEN; break;
                }
                switch(rm_imu_data.accel_rangle)
                {
                    case 0: rm_imu_data.accel_sen = ACCEL_3G_SEN; break;
                    case 1: rm_imu_data.accel_sen = ACCEL_6G_SEN; break;
                    case 2: rm_imu_data.accel_sen = ACCEL_12G_SEN; break;
                    case 3: rm_imu_data.accel_sen = ACCEL_24G_SEN; break;
                }
                break;
            }
            case RM_IMU_QUAT_ID:
            {
                if(rm_imu_data.quat_euler && fdcan_RxHeader.DataLength == 6)
                {
                    memcpy(rm_imu_data.euler_angle, buf, fdcan_RxHeader.DataLength);
                    rm_imu_data.euler_angle_fp32[0] = rm_imu_data.euler_angle[0] * 0.0001f;
                    rm_imu_data.euler_angle_fp32[1] = rm_imu_data.euler_angle[1] * 0.0001f;
                    rm_imu_data.euler_angle_fp32[2] = rm_imu_data.euler_angle[2] * 0.0001f;
                    update_flag.euler=1;
                    rm_imu_data.yaw=rm_imu_data.euler_angle_fp32[0];
                    yaw_all_calc(&rm_imu_data);
                    rm_imu_data.last_yaw=rm_imu_data.yaw;

                }
                else if(rm_imu_data.quat_euler == 0 && fdcan_RxHeader.DataLength == 8)
                {
                    memcpy(rm_imu_data.quat, buf, fdcan_RxHeader.DataLength);
                    rm_imu_data.quat_fp32[0] = rm_imu_data.quat[0] * 0.0001f;
                    rm_imu_data.quat_fp32[1] = rm_imu_data.quat[1] * 0.0001f;
                    rm_imu_data.quat_fp32[2] = rm_imu_data.quat[2] * 0.0001f;
                    rm_imu_data.quat_fp32[3] = rm_imu_data.quat[3] * 0.0001f;
                }
                break;
            }
            case RM_IMU_GYRO_ID:
            {
                memcpy(rm_imu_data.gyro_int16, buf,6);
                rm_imu_data.gyro_fp32[0] = rm_imu_data.gyro_int16[0] * rm_imu_data.gyro_sen;
                rm_imu_data.gyro_fp32[1] = rm_imu_data.gyro_int16[1] * rm_imu_data.gyro_sen;
                rm_imu_data.gyro_fp32[2] = rm_imu_data.gyro_int16[2] * rm_imu_data.gyro_sen;
                //rm_imu_data.sensor_temperature = (int16_t)((buf[6] << 3) | (buf[7] >>5));
                rm_imu_data.sensor_temperature = (float)(( (buf[7] << 8) | buf[6] )/100.0f);
                update_flag.gyro=1;
                if (rm_imu_data.sensor_temperature > 1023)
                {
                    rm_imu_data.sensor_temperature -= 2048;
                }
                break;
            }
            case RM_IMU_ACCEL_ID:
            {
                memcpy(rm_imu_data.accel_int16, buf,6);
                rm_imu_data.accel_fp32[0] = rm_imu_data.accel_int16[0] * rm_imu_data.accel_sen;
                rm_imu_data.accel_fp32[1] = rm_imu_data.accel_int16[1] * rm_imu_data.accel_sen;
                rm_imu_data.accel_fp32[2] = rm_imu_data.accel_int16[2] * rm_imu_data.accel_sen;
                memcpy(&rm_imu_data.sensor_time, (buf + 6), 2);
                update_flag.acc=1;
                break;
            }
            case 0x443:
            {
                Cap_Data_Receive(buf,&receive);
                power_model.real_power=receive.P_in-receive.P_cap;
            }
        }
    }
    else if(hfdcan==&hfdcan3)
    {
        switch (fdcan_RxHeader.Identifier)
        {
            case 0x201://拨弹motor_shoot
            {
                get_motor_measure(&motor_shoot,buf)
                Shoot.shoot_motor.pos=convert(motor_shoot.ecd);
                muti_turns_pos_calc(&Shoot.shoot_motor);
                Shoot.shoot_motor.final_pos=Shoot.shoot_motor.muti_turns_pos/36.f;//输出轴端的位置
                Shoot.shoot_motor.vel=(float)motor_shoot.speed_rpm*0.10472f/36.f;//输出轴端的速度,rad/s
                Shoot.shoot_motor.torque=(float)motor_shoot.given_current*0.00018f;//*(10/10000)*0.18   NM
                Shoot.shoot_motor.last_pos=Shoot.shoot_motor.pos;
            }break;
            case 0x206://云台pitch
            {
                get_motor_measure(&motor_gimbal[1],buf)
                Gimbal.pitch_motor.pos=convert(motor_gimbal[1].ecd);
                muti_turns_pos_calc(&Gimbal.pitch_motor);
                Gimbal.pitch_motor.vel=(float)motor_gimbal[1].speed_rpm*0.10472f;
                Gimbal.pitch_motor.torque=(float)motor_gimbal[1].given_current*TORQUE_CONSTANT/CUR_CONSTANT;
                Gimbal.pitch_motor.last_pos=Gimbal.pitch_motor.pos;

            }break;
            case 0x211://摩擦轮左(后视)
            {
                dm_motor_fbdata(&rub_wheel[left],buf);

            }break;
            case 0x212://摩擦轮右(后视)
            {
                dm_motor_fbdata(&rub_wheel[right],buf);

            }break;
        }

    }

    return fdcan_RxHeader.DataLength;
}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan；FDCAN句柄
* @param:       RxFifo0ITs：中断标志位
* @retval:     	void
* @details:    	HAL库的FDCAN中断回调函数
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        if(hfdcan == &hfdcan1)
        {
            fdcan1_rx_callback();
        }
        if(hfdcan == &hfdcan2)
        {
            fdcan2_rx_callback();
        }
        if(hfdcan == &hfdcan3)
        {
            fdcan3_rx_callback();
        }
    }
}

void yaw_all_calc(rm_imu_data_t* data)
{
    float diff = data->yaw-data->last_yaw;
    if(diff < -PI)
    {
        data->turns++;
    }
    else if(diff>PI)
    {
        data->turns--;
    }
    data->yaw_all = data->yaw + data->turns*2*PI;
}





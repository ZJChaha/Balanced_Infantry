//
// Created by ZJC on 2025/2/8.
//

#include "bsp_super_cap.h"



cap_receive receive; //电容组&功控板接收结构体
cap_control control; //电容组&功控板控制结构体
/**
* @brief 电容组&功控板数据处理
* @param[in] CAN 原始数据 uint8_t 8 字节
* @param[in] 电容组&功控板接收结构体
* @retval 无
**/
void Cap_Data_Receive(uint8_t *data,cap_receive *p)
{
    uint16_t cap_v = data[1] << 8 | data[0];
    int16_t cap_p = data[3] << 8 | data[2];
    uint16_t bat_p = data[5] << 8 | data[4];
    p->V_cap = (float) (cap_v / 100.0f);
    p->P_cap = (float) (cap_p / 100.0f);
    p->P_in = (float) (bat_p / 100.0f);
    p->cap_mode = data[6];
    p->state = data[7];
}


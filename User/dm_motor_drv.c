#include <string.h>
#include "dm_motor_drv.h"
#include "fdcan.h"
#include "bsp_dwt.h"
#include "Chassis_Task.h"
#include "cmsis_os.h"

dm_motor_t dm_motor[num];
dm_motor_t rub_wheel[2];


void dm_motor_init(void)
{
    // Motor init
    memset(&dm_motor[Motor1], 0, sizeof(dm_motor[Motor1]));
    memset(&dm_motor[Motor2], 0, sizeof(dm_motor[Motor2]));
    memset(&dm_motor[Motor3], 0, sizeof(dm_motor[Motor3]));
    memset(&dm_motor[Motor4], 0, sizeof(dm_motor[Motor4]));

    memset(&rub_wheel[left], 0, sizeof(rub_wheel[left]));
    memset(&rub_wheel[right], 0, sizeof(rub_wheel[right]));

    dm_motor[Motor1].id = 0x01;
    dm_motor[Motor1].mst_id = 0x11;
    dm_motor[Motor1].tmp.read_flag = 1;
    dm_motor[Motor1].ctrl.mode 	= mit_mode;
    dm_motor[Motor1].ctrl.vel_set 	= 0;
    dm_motor[Motor1].ctrl.pos_set 	= 0;
    dm_motor[Motor1].ctrl.cur_set 	= 0;
    dm_motor[Motor1].ctrl.kd_set 	= 0;
    dm_motor[Motor1].tmp.PMAX		= 3.141593f;
    dm_motor[Motor1].tmp.VMAX		= 45.0f;
    dm_motor[Motor1].tmp.TMAX		= 40.0f;

    dm_motor[Motor2].id = 0x02;
    dm_motor[Motor2].mst_id = 0x22;
    dm_motor[Motor2].tmp.read_flag = 1;
    dm_motor[Motor2].ctrl.mode 	= mit_mode;
    dm_motor[Motor2].ctrl.vel_set 	= 0;
    dm_motor[Motor2].ctrl.pos_set 	= 0;
    dm_motor[Motor2].ctrl.cur_set 	= 0;
    dm_motor[Motor2].ctrl.kd_set 	= 0;
    dm_motor[Motor2].tmp.PMAX		= 3.141593f;
    dm_motor[Motor2].tmp.VMAX		= 45.0f;
    dm_motor[Motor2].tmp.TMAX		= 40.0f;

    dm_motor[Motor3].id = 0x03;
    dm_motor[Motor3].mst_id = 0x33;
    dm_motor[Motor3].tmp.read_flag = 1;
    dm_motor[Motor3].ctrl.mode 	= mit_mode;
    dm_motor[Motor3].ctrl.vel_set 	= 0;
    dm_motor[Motor3].ctrl.pos_set 	= 0;
    dm_motor[Motor3].ctrl.cur_set 	= 0;
    dm_motor[Motor3].ctrl.kd_set 	= 0;
    dm_motor[Motor3].tmp.PMAX		= 3.141593f;
    dm_motor[Motor3].tmp.VMAX		= 45.0f;
    dm_motor[Motor3].tmp.TMAX		= 40.0f;

    dm_motor[Motor4].id = 0x04;
    dm_motor[Motor4].mst_id = 0x44;
    dm_motor[Motor4].tmp.read_flag = 1;
    dm_motor[Motor4].ctrl.mode 	= mit_mode;
    dm_motor[Motor4].ctrl.vel_set 	= 0;
    dm_motor[Motor4].ctrl.pos_set 	= 0;
    dm_motor[Motor4].ctrl.cur_set 	= 0;
    dm_motor[Motor4].ctrl.kd_set 	= 0;
    dm_motor[Motor4].tmp.PMAX		= 3.141593f;
    dm_motor[Motor4].tmp.VMAX		= 45.0f;
    dm_motor[Motor4].tmp.TMAX		= 40.0f;

    rub_wheel[left].id = 0x01;
    rub_wheel[left].mst_id = 0x11;
    rub_wheel[left].tmp.read_flag = 1;
    rub_wheel[left].ctrl.mode 	= spd_mode;
    rub_wheel[left].ctrl.vel_set 	= 0;

    rub_wheel[right].id = 0x02;
    rub_wheel[right].mst_id = 0x22;
    rub_wheel[right].tmp.read_flag = 1;
    rub_wheel[right].ctrl.mode 	= spd_mode;
    rub_wheel[right].ctrl.vel_set 	= 0;

}


/**
************************************************************************
* @brief:      	dm_motor_clear_para
* @param[in]:   hcan:    fdcan struct
* @param[in]:   motor:   motor typedef struct
* @details:    	clear paraments
************************************************************************
**/
void dm_motor_clear_para(dm_motor_t *motor)
{
	motor->ctrl.kd_set 	= 0;
	motor->ctrl.kp_set	= 0;
	motor->ctrl.pos_set = 0;
	motor->ctrl.vel_set = 0;
	motor->ctrl.tor_set = 0;
	motor->ctrl.cur_set = 0;
}

/**
************************************************************************
* @brief:      	dm_motor_fbdata
* @param[in]:   motor:   motor typedef struct
* @param[in]:   rx_data:  receive data buffer
* @retval:     	void
* @details:    	analyze motor parameters
************************************************************************
**/
void dm_motor_fbdata(dm_motor_t *motor, const uint8_t *rx_data)
{
	motor->para.id = (rx_data[0])&0x0F;
	motor->para.state = (rx_data[0])>>4;
	motor->para.p_int=(rx_data[1]<<8)|rx_data[2];
	motor->para.v_int=(rx_data[3]<<4)|(rx_data[4]>>4);
	motor->para.t_int=((rx_data[4]&0xF)<<8)|rx_data[5];
	motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16); // (-3.141593,3.141593)
	motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12); // (-45.0,45.0)
	motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12); // (-40.0,40.0)
	motor->para.Tmos = (float)(rx_data[6]);
	motor->para.Tcoil = (float)(rx_data[7]);
}

/**
************************************************************************
* @brief:      	float_to_uint: Function to convert a float to an unsigned integer
* @param[in]:   x_float:	Float value to be converted
* @param[in]:   x_min:		Minimum range value
* @param[in]:   x_max:		Maximum range value
* @param[in]:   bits: 		Bit width of the target unsigned integer
* @retval:     	Unsigned integer result
* @details:    	Maps the given float x linearly within the specified range [x_min, x_max] to an unsigned integer of the specified bit width.
************************************************************************
**/

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: Function to convert an unsigned integer to a float
* @param[in]:   x_int: Unsigned integer to be converted
* @param[in]:   x_min: Minimum range value
* @param[in]:   x_max: Maximum range value
* @param[in]:   bits:  Bit width of the unsigned integer
* @retval:     	Float result
* @details:    	Maps the given unsigned integer x_int linearly within the specified range [x_min, x_max] to a float.
************************************************************************
**/

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
************************************************************************
* @brief:      	enable_motor_mode: Function to enable motor mode
* @param[in]:   hcan:     Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID specifying the target motor
* @param[in]:   mode_id:  Mode ID specifying the mode to enable
* @retval:     	void
* @details:     Sends a command via the CAN bus to enable a specific mode on the targeted motor
************************************************************************
**/
void enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFC;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void enable_all_motor_bare(void)
{
    for(uint8_t i=1;i<=4;i++)
    {
        clear_err(&hfdcan1,i,MIT_MODE);

        clear_err(&hfdcan3,0x11,SPD_MODE);
        HAL_Delay(1);
        clear_err(&hfdcan3,0x22,SPD_MODE);
        clear_err(&hfdcan1,i,MIT_MODE);
    }


    for(uint8_t i=1;i<=4;i++)
    {
        enable_motor_mode(&hfdcan1,i,MIT_MODE);
        enable_motor_mode(&hfdcan3,0x11,SPD_MODE);
        HAL_Delay(1);
        enable_motor_mode(&hfdcan3,0x22,SPD_MODE);
        enable_motor_mode(&hfdcan1,i,MIT_MODE);

    }
    hub_motor_send_data(&Chassis,3);
}

void enable_all_motor_os(void)
{
    for(uint8_t i=1;i<=4;i++)
    {
        clear_err(&hfdcan1,i,MIT_MODE);
        osDelay(1);
    }
    for(uint8_t i=1;i<=4;i++)
    {
        enable_motor_mode(&hfdcan1,i,MIT_MODE);
        osDelay(1);
    }
}


/**
************************************************************************
* @brief:      	disable_motor_mode: Function to disable motor mode
* @param[in]:   hcan:     Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID specifying the target motor
* @param[in]:   mode_id:  Mode ID specifying the mode to disable
* @retval:     	void
* @details:     Sends a command via the CAN bus to disable a specific mode on the targeted motor
************************************************************************
**/
void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFD;
	
	fdcanx_send_data(hcan, id, data, 8);
}

void disable_all_motor_os(void)
{
    for(uint8_t i=1;i<=4;i++)
    {
        disable_motor_mode(&hfdcan1,i,MIT_MODE);
        osDelay(1);
    }

}
/**
************************************************************************
* @brief:      	clear_err: Function to clear motor error
* @param[in]:   hcan:     Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id: Motor ID specifying the target motor
* @param[in]:   mode_id:  Mode ID specifying the mode to clear errors
* @retval:     	void
* @details:     Sends a command via the CAN bus to clear errors on a specific motor
************************************************************************
**/
void clear_err(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	data[0] = 0xFF;
	data[1] = 0xFF;
	data[2] = 0xFF;
	data[3] = 0xFF;
	data[4] = 0xFF;
	data[5] = 0xFF;
	data[6] = 0xFF;
	data[7] = 0xFB;
	
	fdcanx_send_data(hcan, id, data, 8);
}

/**
************************************************************************
* @brief:      	mit_ctrl: MIT mode motor control function
* @param[in]:   hcan:      Pointer to the CAN_HandleTypeDef structure
* @param[in]:   motor_id:  Motor ID specifying the target motor
* @param[in]:   pos:       Position setpoint
* @param[in]:   vel:       Velocity setpoint
* @param[in]:   kp:        Position proportional gain
* @param[in]:   kd:        Position derivative gain
* @param[in]:   tor:       Torque setpoint
* @retval:     	void
* @details:     Sends a control frame in MIT mode to the motor via the CAN bus
************************************************************************
**/
void mit_ctrl(hcan_t* hcan, dm_motor_t *motor, uint16_t motor_id, float pos, float vel,float kp, float kd, float tor)
{
	uint8_t data[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
	vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
	tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
	kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

	data[0] = (pos_tmp >> 8);
	data[1] = pos_tmp;
	data[2] = (vel_tmp >> 4);
	data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data[4] = kp_tmp;
	data[5] = (kd_tmp >> 4);
	data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data[7] = tor_tmp;
	
	fdcanx_send_data(hcan, id, data, 8);
}


void spd_ctrl(hcan_t* hcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t data[4];

    id = motor_id + SPD_MODE;
    vbuf=(uint8_t*)&vel;

    data[0] = *vbuf;
    data[1] = *(vbuf+1);
    data[2] = *(vbuf+2);
    data[3] = *(vbuf+3);

    fdcanx_send_data(hcan, id, data, 4);
}





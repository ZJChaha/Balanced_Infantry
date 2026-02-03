#ifndef BSP_USART_H__
#define BSP_USART_H__

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "pid.h"

#define UART_RX_DMA_SIZE (1024)

#define SBUS_HEAD 0X0F
#define SBUS_END 	0X00

#define DOUBLE_BUFFER_ENABLE 1






//迈克MC6C_mini遥控器数据长度为25字节
//大疆DT7遥控器数据长度为18字节
#define USART5_BUFLEN 18
#define USART5_MAX_LEN USART5_BUFLEN * 2


extern uint8_t usart5_buf[USART5_MAX_LEN];
extern uint8_t rx_buffer1[100];
extern uint8_t rx_buffer5[18];
extern uint8_t rx_buffer7[500];
extern uint8_t rx_buffer10[100];

extern uint8_t rx_len1;
extern uint16_t rx_len7;
extern uint8_t rx_len10;
extern uint8_t rx_len5;

typedef struct
{
    uint8_t state;
    uint8_t data[64];

} wifi_rx_t;
extern wifi_rx_t wifi_rx;


typedef struct
{
    int16_t ch[10];
} rc_sbus_t;
extern rc_sbus_t rc_sbus_receive;


typedef union _imu_data
{
    uint8_t data[24];
    float ActVal[6];
} imudata_t;


typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float set;
}__attribute__((packed)) debug_pid_t;

extern debug_pid_t debug_pid;

extern imudata_t imudata;

void uart_receive_handler(UART_HandleTypeDef *huart);
void uart_receive_init(UART_HandleTypeDef *huart);
void usart_init(void);





extern uint8_t rx_buffer[18];   //接收数据的数组
extern uint8_t rx_len; //接收数据的长度
extern uint8_t recv_end_flag; //接收结束标志位
extern uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream);
extern void usart5_callback_handler(uint8_t *buff);

void user_usart_init(void);
//void usart1_data_process(uint8_t* data,uint8_t len);
void uart7_data_process(uint8_t* data,uint16_t len);
void usart1_data_process(uint8_t* data,uint8_t len);
void pid_para_realtime_update(pid_type_def *pid,debug_pid_t* para);
void bluetooth_send_data(float *data);

#endif

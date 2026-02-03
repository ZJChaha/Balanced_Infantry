#ifndef IMU_TEMP_CTRL_H
#define IMU_TEMP_CTRL_H
void IMU_task(void * argument);
void INS_Init(void);
void INS_Task(void);

extern uint8_t IMU_init_finish;
#endif // IMU_TEMP_CTRL_H

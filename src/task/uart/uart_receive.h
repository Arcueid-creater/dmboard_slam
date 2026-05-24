

#ifndef CTRBOARD_H7_ALL_USART_RECIVE_TASK_H
#define CTRBOARD_H7_ALL_USART_RECIVE_TASK_H

#include "robot.h"

extern struct air_sensor_msg air_sensor_data;
extern volatile uint32_t uart7_error_code;
extern volatile uint32_t uart7_rx_callback_cnt;
extern volatile uint32_t uart7_proc_cnt;
extern volatile uint32_t uart7_checksum_fail_cnt;
extern volatile uint16_t uart7_last_size;

void USARTRecTask_Entry(void const * argument);

#endif //CTRBOARD_H7_ALL_USART_RECIVE_TASK_H

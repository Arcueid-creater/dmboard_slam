#ifndef ESP8266_H
#define ESP8266_H

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"

/* 返回值 */
#define ESP8266_OK      0
#define ESP8266_ERR     1
#define ESP8266_TIMEOUT 2

/* 接收缓冲区大小 */
#define ESP8266_RX_BUF_SIZE  512

/* 初始化 */
void esp8266_init(void);

/* DMA接收初始化（由uart_receive.c调用） */
void esp8266_dma_init(void);

/* ISR回调：接收完成通知（由HAL_UARTEx_RxEventCallback调用） */
void esp8266_rx_isr(uint16_t size);

/* ISR回调：DMA发送完成通知（由HAL_UART_TxCpltCallback调用） */
void esp8266_tx_done_isr(void);

/* 发送AT命令并等待期望响应，返回ESP8266_OK/ERR/TIMEOUT */
int esp8266_send_cmd(const char *cmd, const char *expected_response,
                      uint32_t timeout_ms);

/* 发送原始数据（AT+CIPSEND包装） */
bool esp8266_send_data(const uint8_t *data, uint16_t len);

/* 从接收缓冲提取+IPD数据 */
uint8_t *esp8266_get_ipd(uint16_t *out_len);

/* 处理接收到的数据（由process_uart10_data调用） */
void esp8266_process_rx(void);

/* 错误恢复：重置DMA并清空缓冲区 */
void esp8266_error_recover(void);

/* 检查接收缓冲中是否有CLOSED/WIFI DISCONNECT等断连标志 */
bool esp8266_is_connection_lost(void);

/* 清空接收缓冲 */
void esp8266_clear_rx(void);

/* 接收完成的二进制信号量（供uart接收任务通知cloud任务） */
extern SemaphoreHandle_t xSemaphoreESP8266Rx;

#endif

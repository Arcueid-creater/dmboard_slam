#include "esp8266.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* ---------- DMA双缓冲接收 ---------- */
static volatile uint8_t  rx_buf_idx;
static volatile uint16_t rx_buf_size;
static uint8_t rx_buffer[2][ESP8266_RX_BUF_SIZE];

/* ---------- 同步信号量 ---------- */
SemaphoreHandle_t xSemaphoreESP8266Rx = NULL;
static SemaphoreHandle_t tx_done_sem = NULL;

/* ---------- AT响应处理 ---------- */
static uint8_t  at_response_buf[ESP8266_RX_BUF_SIZE];
static uint16_t at_response_len;

/* ---------- 初始化 ---------- */
void esp8266_init(void)
{
    xSemaphoreESP8266Rx = xSemaphoreCreateBinary();
    tx_done_sem = xSemaphoreCreateBinary();
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(at_response_buf, 0, sizeof(at_response_buf));
    rx_buf_idx = 0;
    rx_buf_size = 0;
    at_response_len = 0;
}

void esp8266_dma_init(void)
{
    memset(rx_buffer, 0, sizeof(rx_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buffer[rx_buf_idx], ESP8266_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart10.hdmarx, DMA_IT_HT);
}

/* ---------- ISR回调（仅做buffer翻转和DMA重启，信号量由uart_receive.c处理） ---------- */
void esp8266_rx_isr(uint16_t size)
{
    if (size > ESP8266_RX_BUF_SIZE) return;

    rx_buf_size = size;
    rx_buf_idx ^= 1;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buffer[rx_buf_idx], ESP8266_RX_BUF_SIZE);
}

void esp8266_tx_done_isr(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(tx_done_sem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ---------- 数据处理（被uart接收任务调用） ---------- */
void esp8266_process_rx(void)
{
    uint8_t finished_buf = rx_buf_idx ^ 1;

    memcpy(at_response_buf, rx_buffer[finished_buf], rx_buf_size);
    at_response_len = rx_buf_size;

    memset(rx_buffer[finished_buf], 0, ESP8266_RX_BUF_SIZE);

    /* 通知等待AT响应的cloud任务 */
    if (xSemaphoreESP8266Rx) {
        xSemaphoreGive(xSemaphoreESP8266Rx);
    }
}

void esp8266_clear_rx(void)
{
    memset(at_response_buf, 0, sizeof(at_response_buf));
    at_response_len = 0;
}

/* ---------- DMA发送 ---------- */
static bool esp8266_dma_tx(const uint8_t *data, uint16_t len, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit_DMA(&huart10, (uint8_t *)data, len);
    if (status != HAL_OK) return false;

    if (xSemaphoreTake(tx_done_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return false;

    return true;
}

/* ---------- AT命令发送 ---------- */
int esp8266_send_cmd(const char *cmd, const char *expected_response,
                      uint32_t timeout_ms)
{
    esp8266_clear_rx();
    /* 排空残留信号量（上次SEND OK等未消费的信号），防止下次wait立即返回 */
    xSemaphoreTake(xSemaphoreESP8266Rx, 0);

    if (!esp8266_dma_tx((const uint8_t *)cmd, strlen(cmd), timeout_ms))
        return ESP8266_TIMEOUT;

    /* 循环等待响应，处理回显和应答分片到达的情况 */
    uint32_t elapsed = 0;
    const uint32_t poll_ms = 200;
    while (elapsed < timeout_ms) {
        if (xSemaphoreTake(xSemaphoreESP8266Rx, pdMS_TO_TICKS(poll_ms)) == pdTRUE) {
            if (strstr((const char *)at_response_buf, expected_response) != NULL)
                return ESP8266_OK;
            /* 检查是否收到错误/断连标志 */
            if (strstr((const char *)at_response_buf, "ERROR") != NULL ||
                strstr((const char *)at_response_buf, "CLOSED") != NULL)
                return ESP8266_ERR;
        }
        elapsed += poll_ms;
    }

    return ESP8266_TIMEOUT;
}

/* ---------- 发送数据（AT+CIPSEND包装） ---------- */
bool esp8266_send_data(const uint8_t *data, uint16_t len)
{
    char cmd_buf[32];

    esp8266_clear_rx();
    /* 排空残留信号量（上次SEND OK等未消费的信号），防止下次wait立即返回 */
    xSemaphoreTake(xSemaphoreESP8266Rx, 0);

    snprintf(cmd_buf, sizeof(cmd_buf), "AT+CIPSEND=%d\r\n", len);

    /* 发送AT+CIPSEND命令 */
    if (!esp8266_dma_tx((const uint8_t *)cmd_buf, strlen(cmd_buf), 3000))
        return false;

    /* 等待 ">" 提示符（循环处理回显分片） */
    {
        uint32_t elapsed = 0;
        const uint32_t poll_ms = 200;
        int found = 0;
        while (elapsed < 5000) {
            if (xSemaphoreTake(xSemaphoreESP8266Rx, pdMS_TO_TICKS(poll_ms)) == pdTRUE) {
                if (strstr((const char *)at_response_buf, ">") != NULL) {
                    found = 1;
                    break;
                }
                if (strstr((const char *)at_response_buf, "CLOSED") ||
                    strstr((const char *)at_response_buf, "ERROR"))
                    return false;
            }
            elapsed += poll_ms;
        }
        if (!found) return false;
    }

    /* 发送实际数据（参考代码不等待SEND OK，由上层轮询+IPD） */
    return esp8266_dma_tx(data, len, 5000);
}

/* ---------- 提取+IPD数据 ---------- */
uint8_t *esp8266_get_ipd(uint16_t *out_len)
{
    char *ptr;

    ptr = strstr((char *)at_response_buf, "+IPD,");
    if (ptr == NULL) return NULL;

    /* 解析长度字段 */
    int ipd_len = 0;
    if (sscanf(ptr, "+IPD,%d:", &ipd_len) != 1)
        return NULL;

    /* 找冒号 */
    ptr = strchr(ptr, ':');
    if (ptr == NULL) return NULL;
    ptr++;

    *out_len = (uint16_t)ipd_len;
    return (uint8_t *)ptr;
}

/* ---------- 检查连接是否断开 ---------- */
bool esp8266_is_connection_lost(void)
{
    if (strstr((const char *)at_response_buf, "CLOSED") != NULL)
        return true;
    if (strstr((const char *)at_response_buf, "WIFI DISCONNECT") != NULL)
        return true;
    return false;
}

/* ---------- 错误恢复 ---------- */
void esp8266_error_recover(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, rx_buffer[rx_buf_idx], ESP8266_RX_BUF_SIZE);
    memset(rx_buffer, 0, sizeof(rx_buffer));
    esp8266_clear_rx();
}

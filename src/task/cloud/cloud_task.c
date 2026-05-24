#include "cloud_task.h"
#include "esp8266.h"
#include "onenet.h"
#include "rm_config.h"
#include "robot.h"
#include "drv_dwt.h"
#include "bsp_log.h"

#include <string.h>
#include <stdio.h>

/* 外部引用：空气传感器数据 */
extern struct air_sensor_msg air_sensor_data;

/* 状态机变量 */
static cloud_state_t cloud_state = CLOUD_STATE_IDLE;
static uint8_t retry_count = 0;
static float last_publish_time = 0;

void cloud_task_init(void)
{
    esp8266_init();
    cloud_state = CLOUD_STATE_IDLE;
    retry_count = 0;
    last_publish_time = 0;
}

void cloud_control_task(void)
{
    char cmd_buf[128];

    switch (cloud_state)
    {
        /* ---------- 空闲，进入初始化 ---------- */
        case CLOUD_STATE_IDLE:
            cloud_state = CLOUD_STATE_ESP8266_INIT;
            break;

        /* ---------- ESP8266 AT通信测试 ---------- */
        case CLOUD_STATE_ESP8266_INIT:
            if (esp8266_send_cmd("AT\r\n", "OK", 3000) == ESP8266_OK) {
                /* 关闭回显，防止回显和应答分片导致send_cmd/send_data误判 */
                esp8266_send_cmd("ATE0\r\n", "OK", 3000);
                if (esp8266_send_cmd("AT+CWMODE=1\r\n", "OK", 3000) == ESP8266_OK) {
                    /* 启用DHCP（参考代码中有此步骤） */
                    if (esp8266_send_cmd("AT+CWDHCP=1,1\r\n", "OK", 3000) == ESP8266_OK) {
                        LOGINFO("[cloud] ESP8266 AT OK\r\n");
                        cloud_state = CLOUD_STATE_WIFI_CONNECT;
                        retry_count = 0;
                        break;
                    }
                }
            }
            if (++retry_count > 3) cloud_state = CLOUD_STATE_ERROR;
            break;

        /* ---------- 连接WiFi ---------- */
        case CLOUD_STATE_WIFI_CONNECT:
            snprintf(cmd_buf, sizeof(cmd_buf), "AT+CWJAP=\"%s\",\"%s\"\r\n",
                     ESP8266_WIFI_SSID, ESP8266_WIFI_PASSWORD);
            if (esp8266_send_cmd(cmd_buf, "GOT IP", 15000) == ESP8266_OK) {
                LOGINFO("[cloud] WiFi connected\r\n");
                cloud_state = CLOUD_STATE_TCP_CONNECT;
                retry_count = 0;
                break;
            }
            if (++retry_count > 3) cloud_state = CLOUD_STATE_ERROR;
            break;

        /* ---------- 连接MQTT服务器 ---------- */
        case CLOUD_STATE_TCP_CONNECT:
            snprintf(cmd_buf, sizeof(cmd_buf), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n",
                     ONENET_MQTT_SERVER, ONENET_MQTT_PORT);
            if (esp8266_send_cmd(cmd_buf, "CONNECT", 10000) == ESP8266_OK) {
                LOGINFO("[cloud] TCP connected\r\n");
                cloud_state = CLOUD_STATE_MQTT_CONNECT;
                retry_count = 0;
                break;
            }
            if (++retry_count > 3) cloud_state = CLOUD_STATE_ERROR;
            break;

        /* ---------- MQTT鉴权 ---------- */
        case CLOUD_STATE_MQTT_CONNECT:
            if (onenet_mqtt_connect()) {
                LOGINFO("[cloud] MQTT connected\r\n");
                cloud_state = CLOUD_STATE_MQTT_SUBSCRIBE;
                retry_count = 0;
                break;
            }
            LOGERROR("[cloud] MQTT connect failed\r\n");
            if (++retry_count > 3) cloud_state = CLOUD_STATE_ERROR;
            break;

        /* ---------- 订阅主题 ---------- */
        case CLOUD_STATE_MQTT_SUBSCRIBE:
            if (onenet_subscribe()) {
                LOGINFO("[cloud] Online, publishing enabled\r\n");
                cloud_state = CLOUD_STATE_ONLINE;
                retry_count = 0;
                last_publish_time = dwt_get_time_ms();
                break;
            }
            if (++retry_count > 3) cloud_state = CLOUD_STATE_ERROR;
            break;

        /* ---------- 在线：处理下行消息 + 定时发布 ---------- */
        case CLOUD_STATE_ONLINE:
        {
            /* 检查ESP8266是否断开 */
            if (esp8266_is_connection_lost()) {
                LOGERROR("[cloud] Connection lost, reconnecting...\r\n");
                cloud_state = CLOUD_STATE_IDLE;
                retry_count = 0;
                break;
            }

            /* 处理接收到的+IPD消息 */
            uint16_t ipd_len = 0;
            uint8_t *ipd = esp8266_get_ipd(&ipd_len);
            if (ipd && ipd_len > 0) {
                onenet_process_message(ipd, ipd_len);
            }

            /* 定时发布传感器数据 */
            float now = dwt_get_time_ms();
            if (now - last_publish_time >= CLOUD_PUBLISH_PERIOD_MS) {
                if (onenet_publish_sensor_data(
                    air_sensor_data.voc,
                    air_sensor_data.hcho,
                    air_sensor_data.eco2,
                    air_sensor_data.temp,
                    air_sensor_data.humidity
                )) {
                    last_publish_time = now;
                } else {
                    LOGERROR("[cloud] Publish failed, reconnecting...\r\n");
                    cloud_state = CLOUD_STATE_IDLE;
                    retry_count = 0;
                }
            }
            break;
        }

        /* ---------- 错误：等待后重试 ---------- */
        case CLOUD_STATE_ERROR:
            LOGERROR("[cloud] Error state, retrying in 5s...\r\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            retry_count = 0;
            cloud_state = CLOUD_STATE_IDLE;
            break;

        default:
            cloud_state = CLOUD_STATE_IDLE;
            break;
    }

    /* 清空接收缓冲，准备下一次响应 */
    esp8266_clear_rx();
}

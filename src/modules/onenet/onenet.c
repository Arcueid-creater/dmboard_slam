#include "onenet.h"
#include "esp8266.h"
#include "mqttkit.h"
#include "base64.h"
#include "hmac_sha1.h"
#include "rm_config.h"
#include "cJSON.h"

#include <string.h>
#include <stdio.h>

/* ---------- URL编码 ---------- */
static unsigned char OTA_UrlEncode(char *sign)
{
    char sign_t[80];
    unsigned char i = 0, j = 0;
    unsigned char sign_len = strlen(sign);

    if (sign == (void *)0 || sign_len < 28)
        return 1;

    for (; i < sign_len; i++) {
        sign_t[i] = sign[i];
        sign[i] = 0;
    }
    sign_t[i] = 0;

    for (i = 0, j = 0; i < sign_len; i++) {
        switch (sign_t[i]) {
            case '+': strcat(sign + j, "%2B"); j += 3; break;
            case ' ': strcat(sign + j, "%20"); j += 3; break;
            case '/': strcat(sign + j, "%2F"); j += 3; break;
            case '?': strcat(sign + j, "%3F"); j += 3; break;
            case '%': strcat(sign + j, "%25"); j += 3; break;
            case '#': strcat(sign + j, "%23"); j += 3; break;
            case '&': strcat(sign + j, "%26"); j += 3; break;
            case '=': strcat(sign + j, "%3D"); j += 3; break;
            default:  sign[j] = sign_t[i]; j++; break;
        }
    }
    sign[j] = 0;
    return 0;
}

/* ---------- OneNET鉴权Token生成 ---------- */
#define METHOD  "sha1"

static unsigned char OneNET_Authorization(char *ver, char *res, unsigned int et,
    char *access_key, char *dev_name, char *auth_buf, unsigned short auth_len, bool flag)
{
    size_t olen = 0;
    char sign_buf[64];
    char hmac_sha1_buf[64];
    char access_key_base64[64];
    char string_for_signature[128];

    if (ver == (void *)0 || res == (void *)0 || et < 1564562581
        || access_key == (void *)0 || auth_buf == (void *)0 || auth_len < 120)
        return 1;

    /* Base64解码access_key */
    memset(access_key_base64, 0, sizeof(access_key_base64));
    BASE64_Decode((unsigned char *)access_key_base64, sizeof(access_key_base64),
                  &olen, (unsigned char *)access_key, strlen(access_key));

    /* 构建签名字符串 */
    memset(string_for_signature, 0, sizeof(string_for_signature));
    if (flag)
        snprintf(string_for_signature, sizeof(string_for_signature),
                 "%d\n%s\nproducts/%s\n%s", et, METHOD, res, ver);
    else
        snprintf(string_for_signature, sizeof(string_for_signature),
                 "%d\n%s\nproducts/%s/devices/%s\n%s", et, METHOD, res, dev_name, ver);

    /* HMAC-SHA1签名 */
    memset(hmac_sha1_buf, 0, sizeof(hmac_sha1_buf));
    hmac_sha1((unsigned char *)access_key_base64, strlen(access_key_base64),
              (unsigned char *)string_for_signature, strlen(string_for_signature),
              (unsigned char *)hmac_sha1_buf);

    /* Base64编码签名 */
    olen = 0;
    memset(sign_buf, 0, sizeof(sign_buf));
    BASE64_Encode((unsigned char *)sign_buf, sizeof(sign_buf), &olen,
                  (unsigned char *)hmac_sha1_buf, strlen(hmac_sha1_buf));

    /* URL编码 */
    OTA_UrlEncode(sign_buf);

    /* 组装Token */
    if (flag)
        snprintf(auth_buf, auth_len,
                 "version=%s&res=products%%2F%s&et=%d&method=%s&sign=%s",
                 ver, res, et, METHOD, sign_buf);
    else
        snprintf(auth_buf, auth_len,
                 "version=%s&res=products%%2F%s%%2Fdevices%%2F%s&et=%d&method=%s&sign=%s",
                 ver, res, dev_name, et, METHOD, sign_buf);

    return 0;
}

/* ---------- MQTT连接 ---------- */
bool onenet_mqtt_connect(void)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};
    unsigned char *dataPtr;
    char auth_buf[160];
    bool status = false;
    int retries;

    OneNET_Authorization(ONENET_VERSION, ONENET_PROID, ONENET_TOKEN_ET,
                         ONENET_ACCESS_KEY, ONENET_DEVICE_NAME,
                         auth_buf, sizeof(auth_buf), 0);

    if (MQTT_PacketConnect(ONENET_PROID, auth_buf, ONENET_DEVICE_NAME,
                           256, 1, MQTT_QOS_LEVEL0,
                           NULL, NULL, 0, &mqttPacket) == 0)
    {
        esp8266_send_data(mqttPacket._data, mqttPacket._len);

        /* 等待服务器返回CONNACK（+IPD数据） */
        uint16_t ipd_len = 0;
        retries = 10; /* 10 × 500ms = 5s 超时 */
        while (retries--) {
            dataPtr = esp8266_get_ipd(&ipd_len);
            if (dataPtr != NULL) break;
            /* 等待下一次UART接收 */
            xSemaphoreTake(xSemaphoreESP8266Rx, pdMS_TO_TICKS(500));
        }

        if (dataPtr != NULL) {
            if (MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK) {
                if (MQTT_UnPacketConnectAck(dataPtr) == 0)
                    status = true;
            }
        }

        MQTT_DeleteBuffer(&mqttPacket);
    }

    return status;
}

/* ---------- 订阅 ---------- */
bool onenet_subscribe(void)
{
    MQTT_PACKET_STRUCTURE mqtt_packet = {NULL, 0, 0, 0};
    char topic_buf[56];
    const char *topic = topic_buf;
    int retries;
    unsigned char *dataPtr;

    snprintf(topic_buf, sizeof(topic_buf), "$sys/%s/%s/thing/property/set",
             ONENET_PROID, ONENET_DEVICE_NAME);

    if (MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, (const int8 **)&topic, 1, &mqtt_packet) == 0) {
        esp8266_send_data(mqtt_packet._data, mqtt_packet._len);

        /* 等待服务器返回SUBACK */
        uint16_t ipd_len = 0;
        retries = 10;
        while (retries--) {
            dataPtr = esp8266_get_ipd(&ipd_len);
            if (dataPtr != NULL) break;
            xSemaphoreTake(xSemaphoreESP8266Rx, pdMS_TO_TICKS(500));
        }

        if (dataPtr != NULL) {
            if (MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_SUBACK) {
                MQTT_UnPacketSubscribe(dataPtr);
            }
        }

        MQTT_DeleteBuffer(&mqtt_packet);
        return true;
    }

    return false;
}

/* ---------- 填充传感器JSON ---------- */
static unsigned char onenet_fill_buf(char *buf, uint16_t voc, uint16_t hcho,
                                      uint16_t eco2, float temp, float humidity)
{
    memset(buf, 0, 256);
    snprintf(buf, 256,
             "{\"id\":\"123\","
             "\"params\":{"
             "\"VOC\":{\"value\":%d},"
             "\"jiaquan\":{\"value\":%d},"
             "\"eCO2\":{\"value\":%d},"
             "\"temp\":{\"value\":%.1f},"
             "\"humi\":{\"value\":%.1f}"
             "}}",
             voc, hcho, eco2, temp, humidity);

    return strlen(buf);
}

/* ---------- 发布传感器数据 ---------- */
bool onenet_publish_sensor_data(uint16_t voc, uint16_t hcho, uint16_t eco2,
                                 float temp, float humidity)
{
    MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};
    char buf[256];
    uint16_t body_len;

    body_len = onenet_fill_buf(buf, voc, hcho, eco2, temp, humidity);

    if (body_len == 0) return false;

    if (MQTT_PacketSaveData(ONENET_PROID, ONENET_DEVICE_NAME, body_len, NULL, &mqttPacket) != 0)
        return false;

    /* 追加JSON payload到MQTT数据包尾部 */
    for (int i = 0; i < body_len; i++)
        mqttPacket._data[mqttPacket._len++] = buf[i];

    esp8266_send_data(mqttPacket._data, mqttPacket._len);
    MQTT_DeleteBuffer(&mqttPacket);

    return true;
}

/* ---------- 处理平台下发消息 ---------- */
void onenet_process_message(uint8_t *cmd, uint16_t len)
{
    (void)len;

    char *req_payload = NULL;
    char *cmdid_topic = NULL;
    uint16_t topic_len = 0;
    uint16_t req_len = 0;
    uint8_t qos = 0;
    static uint16_t pkt_id = 0;
    uint8_t type;

    type = MQTT_UnPacketRecv(cmd);
    switch (type)
    {
        case MQTT_PKT_PUBLISH:
            if (MQTT_UnPacketPublish(cmd, (int8 **)&cmdid_topic, &topic_len,
                                     (int8 **)&req_payload, &req_len, &qos, &pkt_id) == 0)
            {
                cJSON *raw_json = cJSON_Parse(req_payload);
                if (raw_json) {
                    cJSON *params = cJSON_GetObjectItem(raw_json, "params");
                    if (params) {
                        /* 处理下发的属性，后续可扩展 */
                    }
                    cJSON_Delete(raw_json);
                }
            }
            break;

        case MQTT_PKT_PUBACK:
            break;

        case MQTT_PKT_SUBACK:
            break;

        default:
            break;
    }

    if (type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH) {
        if (cmdid_topic) MQTT_FreeBuffer(cmdid_topic);
        if (req_payload) MQTT_FreeBuffer(req_payload);
    }
}

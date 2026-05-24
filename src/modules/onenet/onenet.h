#ifndef ONENET_H
#define ONENET_H

#include <stdbool.h>
#include <stdint.h>

/* MQTT连接并鉴权到OneNET平台 */
bool onenet_mqtt_connect(void);

/* 订阅属性设置topic */
bool onenet_subscribe(void);

/* 发布传感器数据 (JSON格式) */
bool onenet_publish_sensor_data(uint16_t voc, uint16_t hcho, uint16_t eco2,
                                 float temp, float humidity);

/* 处理平台下发的MQTT报文 */
void onenet_process_message(uint8_t *data, uint16_t len);

#endif

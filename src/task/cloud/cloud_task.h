#ifndef CLOUD_TASK_H
#define CLOUD_TASK_H

#include <stdint.h>

/* 云通信状态机 */
typedef enum {
    CLOUD_STATE_IDLE = 0,
    CLOUD_STATE_ESP8266_INIT,
    CLOUD_STATE_WIFI_CONNECT,
    CLOUD_STATE_TCP_CONNECT,
    CLOUD_STATE_MQTT_CONNECT,
    CLOUD_STATE_MQTT_SUBSCRIBE,
    CLOUD_STATE_ONLINE,
    CLOUD_STATE_ERROR,
} cloud_state_t;

void cloud_task_init(void);
void cloud_control_task(void);

#endif

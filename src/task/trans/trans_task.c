#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "rm_module.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include "trans_task.h"
#include "gimbal_task.h"

#define HEART_BEAT 500 // ms

/* ==================== USB队列传输 ==================== */
#define USB_RX_MSG_LEN          512
#define USB_RX_MSG_COUNT        8

#define USB_TX_MSG_COUNT        8
#define USB_TX_MSG_MAX          512

typedef struct {
    uint16_t len;
    uint8_t data[USB_RX_MSG_LEN];
} usb_rx_msg_t;

typedef struct {
    uint16_t len;
    uint8_t data[USB_TX_MSG_MAX];
} usb_tx_msg_t;

static QueueHandle_t usb_rx_queue = NULL;
static usb_rx_msg_t usb_rx_msg_pool[USB_RX_MSG_COUNT];
static uint8_t usb_rx_msg_idx = 0;

static QueueHandle_t usb_tx_queue = NULL;
static QueueHandle_t usb_tx_free_queue = NULL;
static usb_tx_msg_t usb_tx_msg_pool[USB_TX_MSG_COUNT];
static usb_tx_msg_t *usb_tx_inflight = NULL;

static void process_usb_bytes(uint8_t* Buf, uint16_t Len);
static void usb_tx_pump(void);

extern USBD_HandleTypeDef hUsbDeviceHS;

/* ==================== BCP接收状态机 ==================== */
#define BCP_FRAME_MAX  (6 + FRAME_MAX_LEN)  /* HEAD+D_ADDR+ID+LEN + DATA[max] + SC+AC = 42 */

static uint8_t frame_buffer[BCP_FRAME_MAX];
static uint32_t frame_index = 0;
static uint8_t bcp_data_len = 0;    /* 帧中 LEN 字段的值，即 DATA 区的字节数 */
static uint8_t bcp_total_len = 0;   /* 完整帧的预期总字节数 = 6 + LEN */

static enum {
    BCP_WAIT_FOR_HEADER,
    BCP_RECEIVING_HEADER,
    BCP_RECEIVING_DATA
} bcp_state = BCP_WAIT_FOR_HEADER;

/* ==================== 线程间通讯话题相关 ==================== */
// 发布
MCN_DECLARE(transmission_fdb_topic);
static struct trans_fdb_msg trans_fdb_data;

// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chass_cmd;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gimbal_cmd;
MCN_DECLARE(gimbal_fdb_topic);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gimbal_fdb;
MCN_DECLARE(gimbal_ins_topic);
static McnNode_t gimbal_ins_node;
static struct dm_imu_t gim_ins;

extern struct referee_fdb_msg referee_fdb;

static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);

/* ==================== 全局变量 ==================== */
float yaw_obs = 0;
static uint32_t heart_dt;
static float trans_dt;
static float trans_start;
TeamColor team_color = UNKNOWN;

extern auto_relative_angle_status_e auto_relative_angle_status;

/* ==================== 发送函数 ==================== */

/**
 * @brief 将已构建好的BCP帧加入发送队列
 */
static void send_packet(uint8_t *data, uint16_t length)
{
    if (usb_tx_queue == NULL || data == NULL)
        return;

    if (length > USB_TX_MSG_MAX)
        length = USB_TX_MSG_MAX;

    if (usb_tx_free_queue == NULL)
        return;

    usb_tx_msg_t *msg = NULL;
    if (xQueueReceive(usb_tx_free_queue, &msg, 0) != pdPASS || msg == NULL)
        return;

    memcpy(msg->data, data, length);
    msg->len = length;

    if (xQueueSend(usb_tx_queue, &msg, 0) != pdPASS) {
        (void)xQueueSend(usb_tx_free_queue, &msg, 0);
    }
}

/**
 * @brief 发送队列泵：当前帧发送完成后取下一帧通过CDC_Transmit_HS发送
 */
static void usb_tx_pump(void)
{
    if (usb_tx_queue == NULL)
        return;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceHS.pClassData;
    if (hcdc == NULL)
        return;

    if (hcdc->TxState != 0U)
        return;

    if (usb_tx_inflight != NULL) {
        (void)xQueueSend(usb_tx_free_queue, &usb_tx_inflight, 0);
        usb_tx_inflight = NULL;
    }

    if (usb_tx_inflight == NULL) {
        (void)xQueueReceive(usb_tx_queue, &usb_tx_inflight, 0);
    }

    if (usb_tx_inflight == NULL)
        return;

    (void)CDC_Transmit_HS(usb_tx_inflight->data, usb_tx_inflight->len);
}

/**
 * @brief 通用发送接口（外部可调用）：将完整BCP帧加入发送队列
 */
void send_custom_data(uint8_t *data, uint16_t length)
{
    send_packet(data, length);
}

/* ==================== BCP数据打包函数 ==================== */

void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch, float roll, int team_color)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0};
    int32_t rpy_data = 0;
    uint32_t *gimbal_rpy = (uint32_t *)&rpy_data;

    rpy_tx_buffer[0] = 0;
    rpy_data = yaw * 1000;
    rpy_tx_buffer[1] = *gimbal_rpy;
    rpy_tx_buffer[2] = *gimbal_rpy >> 8;
    rpy_tx_buffer[3] = *gimbal_rpy >> 16;
    rpy_tx_buffer[4] = *gimbal_rpy >> 24;
    rpy_data = pitch * 1000;
    rpy_tx_buffer[5] = *gimbal_rpy;
    rpy_tx_buffer[6] = *gimbal_rpy >> 8;
    rpy_tx_buffer[7] = *gimbal_rpy >> 16;
    rpy_tx_buffer[8] = *gimbal_rpy >> 24;
    rpy_data = roll * 1000;
    rpy_tx_buffer[9] = *gimbal_rpy;
    rpy_tx_buffer[10] = *gimbal_rpy >> 8;
    rpy_tx_buffer[11] = *gimbal_rpy >> 16;
    rpy_tx_buffer[12] = *gimbal_rpy >> 24;
    rpy_data = team_color * 1000;
    rpy_tx_buffer[13] = *gimbal_rpy;
    rpy_tx_buffer[14] = *gimbal_rpy >> 8;
    rpy_tx_buffer[15] = *gimbal_rpy >> 16;
    rpy_tx_buffer[16] = *gimbal_rpy >> 24;

    memcpy(&frame->DATA[0], rpy_tx_buffer, 17);
    frame->LEN = FRAME_RPY_LEN;
}

void Check_Rpy(RpyTypeDef *frame)
{
    uint8_t sum = 0;
    uint8_t add = 0;

    sum += frame->HEAD;
    sum += frame->D_ADDR;
    sum += frame->ID;
    sum += frame->LEN;
    add += sum;

    for (int i = 0; i < frame->LEN; i++) {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}

/* ==================== 接收函数 ==================== */

/**
 * @brief USB接收回调（在中断上下文中调用，将原始数据入队）
 */
void process_usb_data(uint8_t* Buf, uint32_t *Len)
{
    if (usb_rx_queue == NULL || Buf == NULL || Len == NULL || *Len == 0)
        return;

    uint32_t in_len = *Len;
    if (in_len > USB_RX_MSG_LEN)
        in_len = USB_RX_MSG_LEN;

    usb_rx_msg_t *msg = &usb_rx_msg_pool[usb_rx_msg_idx];
    msg->len = (uint16_t)in_len;
    memcpy(msg->data, Buf, in_len);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(usb_rx_queue, &msg, &xHigherPriorityTaskWoken) == pdPASS) {
        usb_rx_msg_idx = (usb_rx_msg_idx + 1) % USB_RX_MSG_COUNT;
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief 在任务上下文中解析接收到的BCP帧数据
 *
 * BCP 帧格式（每帧总长 = 6 + LEN，各帧 LEN 不同）：
 *   HEAD(1) | D_ADDR(1) | ID(1) | LEN(1) | DATA[LEN] | SC(1) | AC(1)
 *
 * 解析流程：
 *   1. WAIT_FOR_HEADER:    等待 0xFF 帧头
 *   2. BCP_RECEIVING_HEADER: 收齐 D_ADDR / ID / LEN，算出预期总长
 *   3. BCP_RECEIVING_DATA:   根据 LEN 收取剩余字节，收齐后按 ID 分发
 */
static void process_usb_bytes(uint8_t* Buf, uint16_t Len)
{
    for (uint16_t i = 0; i < Len; i++) {
        uint8_t current_byte = Buf[i];

        switch (bcp_state) {
            case BCP_WAIT_FOR_HEADER:
                if (current_byte == 0xFF) {
                    frame_index = 0;
                    frame_buffer[frame_index++] = current_byte;
                    bcp_state = BCP_RECEIVING_HEADER;
                }
                break;

            case BCP_RECEIVING_HEADER:
                frame_buffer[frame_index++] = current_byte;
                /* 收齐 4 字节头部：HEAD + D_ADDR + ID + LEN */
                if (frame_index >= 4) {
                    bcp_data_len = frame_buffer[3];          /* LEN 字段 */
                    bcp_total_len = 6 + bcp_data_len;        /* 完整帧总字节数 */

                    /* 防御：LEN 超过最大值则丢弃此帧 */
                    if (bcp_data_len > FRAME_MAX_LEN || bcp_total_len > BCP_FRAME_MAX) {
                        bcp_state = BCP_WAIT_FOR_HEADER;
                        break;
                    }
                    bcp_state = BCP_RECEIVING_DATA;
                }
                break;

            case BCP_RECEIVING_DATA:
                frame_buffer[frame_index++] = current_byte;

                if (frame_index >= bcp_total_len) {
                    /* ---- 帧收齐，按 ID 分发 ---- */
                    uint8_t id  = frame_buffer[2];
                    uint8_t len = frame_buffer[3];
                    uint8_t *data = &frame_buffer[4];        /* DATA 起始位置（偏移4） */

                    switch (id) {
                        case CHASSIS_CTRL: {
                            if (len >= 24) {
                                trans_fdb_data.linear_x = (*(int32_t *)&data[0] / 10000.0f);
                                trans_fdb_data.linear_y = (*(int32_t *)&data[4] / 10000.0f);
                                trans_fdb_data.linear_z = (*(int32_t *)&data[8] / 10000.0f);
                                trans_fdb_data.angular_x = (*(int32_t *)&data[12] / 10000.0f);
                                trans_fdb_data.angular_y = (*(int32_t *)&data[16] / 10000.0f);
                                trans_fdb_data.angular_z = (*(int32_t *)&data[20] / 10000.0f);
                            }
                        } break;

                        case GIMBAL: {
                            if (len >= 17) {
                                trans_fdb_data.yaw = -(*(int32_t *)&data[1] / 1000.0f);
                                trans_fdb_data.pitch = (*(int32_t *)&data[5] / 1000.0f);
                                trans_fdb_data.roll = (*(int32_t *)&data[9] / 1000.0f);
                                trans_fdb_data.mode = (*(int32_t *)&data[13] / 1000.0f);
                            }
                        } break;

                        case POSE_CTRL: {
                            if (len >= 1) {
                                trans_fdb_data.pose = data[0];
                                if (trans_fdb_data.pose == 3) {
                                    trans_fdb_data.chassis_power_limit = 150;
                                    trans_fdb_data.shooter_17mm_cooling_heat = 10.0f / 3.0f;
                                } else if (trans_fdb_data.pose == 2) {
                                    trans_fdb_data.chassis_power_limit = 50;
                                    trans_fdb_data.shooter_17mm_cooling_heat = 10.0f / 3.0f;
                                } else if (trans_fdb_data.pose == 1) {
                                    trans_fdb_data.chassis_power_limit = 50;
                                    trans_fdb_data.shooter_17mm_cooling_heat = 30;
                                }
                            }
                        } break;

                        case HEARTBEAT: {
                            if (len >= 1) {
                                trans_fdb_data.heartbeat = data[0];
                                heart_dt = dwt_get_time_ms();
                            }
                        } break;

                        default:
                            break;
                    }

                    /* 重置状态机，准备接收下一帧 */
                    memset(frame_buffer, 0, sizeof(frame_buffer));
                    bcp_state = BCP_WAIT_FOR_HEADER;
                }
                break;
        }
    }
}

/* ==================== 发布订阅 ==================== */

static void trans_pub_push(void)
{
    mcn_publish(MCN_HUB(transmission_fdb_topic), &trans_fdb_data);
}

static void trans_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb_topic), NULL, NULL);
    gimbal_ins_node = mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
}

static void trans_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);

    if (mcn_poll(chassis_cmd_node))
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chass_cmd);

    if (mcn_poll(gimbal_cmd_node))
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gimbal_cmd);

    if (mcn_poll(gimbal_fdb_node))
        mcn_copy(MCN_HUB(gimbal_fdb_topic), gimbal_fdb_node, &gimbal_fdb);

    if (mcn_poll(gimbal_ins_node))
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
}

/* ==================== 任务函数 ==================== */

struct trans_fdb_msg* get_trans_fdb(void)
{
    return &trans_fdb_data;
}

void trans_task_init(void)
{
    memset(&trans_fdb_data, 0, sizeof(trans_fdb_data));
    bcp_state = BCP_WAIT_FOR_HEADER;
    frame_index = 0;
    bcp_data_len = 0;
    bcp_total_len = 0;

    if (usb_rx_queue == NULL) {
        usb_rx_queue = xQueueCreate(USB_RX_MSG_COUNT, sizeof(usb_rx_msg_t *));
    }

    if (usb_tx_queue == NULL) {
        usb_tx_queue = xQueueCreate(USB_TX_MSG_COUNT, sizeof(usb_tx_msg_t *));
    }

    if (usb_tx_free_queue == NULL) {
        usb_tx_free_queue = xQueueCreate(USB_TX_MSG_COUNT, sizeof(usb_tx_msg_t *));
        if (usb_tx_free_queue != NULL) {
            for (uint32_t i = 0; i < USB_TX_MSG_COUNT; i++) {
                usb_tx_msg_t *p = &usb_tx_msg_pool[i];
                (void)xQueueSend(usb_tx_free_queue, &p, 0);
            }
        }
    }

    trans_sub_init();
    heart_dt = dwt_get_time_ms();
}

void trans_control_task(void)
{
    trans_start = dwt_get_time_ms();

    trans_sub_pull();

    // 处理接收队列中的所有消息
    usb_rx_msg_t *msg = NULL;
    while (xQueueReceive(usb_rx_queue, &msg, 0) == pdPASS) {
        if (msg != NULL && msg->len > 0) {
            process_usb_bytes(msg->data, msg->len);
        }
    }

    // 发送队列泵：在前一帧发送完成后驱动下一帧
    usb_tx_pump();

    // 心跳检测
    if ((dwt_get_time_ms() - heart_dt) >= HEART_BEAT) {
        heart_dt = dwt_get_time_ms();
    }

    // ==========================================
    // 1. 发送云台姿态 (原样保持，高频 1000Hz 发送)
    // ==========================================
    {
        RpyTypeDef rpy_tx;
        rpy_tx.HEAD = 0xFF;
        rpy_tx.D_ADDR = MAINFLOD;
        rpy_tx.ID = GIMBAL;

        pack_Rpy(&rpy_tx,
                 gimbal_fdb.yaw_offset_angle - gim_ins.yaw,
                 ins.pitch,
                 0.0f,
                 team_color);
        Check_Rpy(&rpy_tx);
        send_packet((uint8_t *)&rpy_tx, sizeof(rpy_tx));
    }

    // ==========================================
    // 2. 降频数据 (1000Hz / 100 = 10Hz)
    // ==========================================
    {
        static uint16_t send_cnt = 0;
        send_cnt++;
        if (send_cnt >= 100) {
            send_cnt = 0;

            // 2.1 哨兵姿态数据
            {
                uint8_t pose_buf[7] = {0};
                pose_buf[0] = 0xFF;
                pose_buf[1] = 0x01;
                pose_buf[2] = 0x06;
                pose_buf[3] = 0x01;

                uint8_t sentry_pose = (referee_fdb.sentry_info.sentry_info_2 >> 12) & 0x03;
                pose_buf[4] = sentry_pose;

                uint8_t sum_p = 0, add_p = 0;
                for (int i = 0; i < 5; i++) {
                    sum_p += pose_buf[i];
                    add_p += sum_p;
                }
                pose_buf[5] = sum_p;
                pose_buf[6] = add_p;

                send_packet(pose_buf, 7);
            }

            // 2.2 机器人血量数据
            {
                uint8_t hp_buf[38] = {0};
                hp_buf[0] = 0xFF;
                hp_buf[1] = 0x01;
                hp_buf[2] = ROBOT_HP;
                hp_buf[3] = 32;

                uint16_t red_hp = referee_fdb.game_robot_HP.red_7_robot_HP;
                uint16_t blue_hp = referee_fdb.game_robot_HP.blue_7_robot_HP;

                hp_buf[14] = red_hp & 0xFF;
                hp_buf[15] = (red_hp >> 8) & 0xFF;

                hp_buf[30] = blue_hp & 0xFF;
                hp_buf[31] = (blue_hp >> 8) & 0xFF;

                uint8_t sum_h = 0, add_h = 0;
                for (int i = 0; i < 36; i++) {
                    sum_h += hp_buf[i];
                    add_h += sum_h;
                }
                hp_buf[36] = sum_h;
                hp_buf[37] = add_h;

                send_packet(hp_buf, 38);
            }
        }
    }

    yaw_obs = gimbal_fdb.yaw_offset_angle - gim_ins.yaw;

    trans_pub_push();

    trans_dt = dwt_get_time_ms() - trans_start;
    if (trans_dt > 1)
        LOGINFO("Transmission Task is being DELAY! dt = [%f]\r\n", &trans_dt);
}

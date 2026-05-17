#include <stdio.h>
#include "cmsis_os.h"
#include "rm_module.h"
#include "usbd_cdc_if.h"
#include "trans_task.h"
#include "gimbal_task.h"


#define HEART_BEAT 500 //ms
#define USB_RX_RING_SIZE 512  // USB接收环形缓冲区大小

/*------------------------------USB接收环形缓冲区--------------------------------- */
typedef struct {
    uint8_t buffer[USB_RX_RING_SIZE];
    uint32_t write_idx;
    uint32_t read_idx;
} ring_buffer_t;

static ring_buffer_t usb_rx_ring;

static void ring_put_force(ring_buffer_t *rb, const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        rb->buffer[rb->write_idx % USB_RX_RING_SIZE] = data[i];
        rb->write_idx++;
        if (rb->write_idx - rb->read_idx > USB_RX_RING_SIZE) {
            rb->read_idx = rb->write_idx - USB_RX_RING_SIZE;
        }
    }
}

static uint32_t ring_available(ring_buffer_t *rb) {
    return rb->write_idx - rb->read_idx;
}

static uint32_t ring_get(ring_buffer_t *rb, uint8_t *data, uint32_t len) {
    uint32_t avail = ring_available(rb);
    if (len > avail) return 0;
    for (uint32_t i = 0; i < len; i++) {
        data[i] = rb->buffer[rb->read_idx % USB_RX_RING_SIZE];
        rb->read_idx++;
    }
    return len;
}

/*------------------------------传输数据相关 --------------------------------- */

extern struct referee_fdb_msg referee_fdb;

RpyTypeDef rpy_tx_data={
        .HEAD = 0XFF,
        .D_ADDR = MAINFLOD,
        .ID = GIMBAL,
        .LEN = FRAME_RPY_LEN,
        .DATA={0},
        .SC = 0,
        .AC = 0,
};
RpyTypeDef rpy_rx_data; //接收解析结构体
static uint32_t heart_dt;
/* ---------------------------------usb虚拟串口数据相关 --------------------------------- */

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
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
static void trans_pub_push(void);
static void trans_sub_init(void);
static void trans_sub_pull(void);
float yaw_obs=0;

/* --------------------------------- 通讯线程入口 --------------------------------- */
static float trans_dt;
static float trans_start;
static float yaw_filtered=0;
static float pitch_filtered=0;

void trans_task_init(){
    trans_sub_init();
}

void trans_control(){

    trans_start = dwt_get_time_ms();
/*--------------------------------------------------具体需要发送的数据--------------------------------- */
    if((dwt_get_time_ms()-heart_dt)>=HEART_BEAT)
    {
        heart_dt=dwt_get_time_ms();
    }
    Send_to_pc(rpy_tx_data);
    yaw_obs=gimbal_fdb.yaw_offset_angle - gim_ins.yaw;

/*--------------------------------------------------具体需要发送的数据---------------------------------*/
    /* 用于调试监测线程调度使用 */
    trans_dt = dwt_get_time_ms() - trans_start;
    if (trans_dt > 1)
        LOGINFO("Transmission Task is being DELAY! dt = [%f]\r\n", &trans_dt);

}

void trans_control_task(){
    /*订阅数据更新*/
    trans_sub_pull();
    trans_control();
    /* 发布数据更新 */
    trans_pub_push();
}

void Send_to_pc(RpyTypeDef data_r)
{
    // ==========================================
    // 1. 发送云台姿态 (1000Hz)
    // ==========================================
    pack_Rpy(&data_r, (gimbal_fdb.yaw_offset_angle - ins.yaw), ins.pitch, ins.roll);
    Check_Rpy(&data_r);
    CDC_Transmit_HS((uint8_t*)&data_r, sizeof(data_r));

    // ==========================================
    // 引入静态分频计数器，降低低频数据的发送速率
    // 1000Hz / 100 = 10Hz
    // ==========================================
    static uint16_t send_cnt = 0;
    send_cnt++;
    if (send_cnt >= 100)
    {
        send_cnt = 0;

        // ==========================================
        // 2. 发送哨兵姿态数据 (降频至 10Hz)
        // ==========================================
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

        CDC_Transmit_HS(pose_buf, 7);

        // ==========================================
        // 3. 发送机器人血量数据 (降频至 10Hz)
        // ==========================================
        uint8_t hp_buf[38] = {0};
        hp_buf[0] = 0xFF;
        hp_buf[1] = 0x01;
        hp_buf[2] = 0x31;
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

        CDC_Transmit_HS(hp_buf, 38);
    }
}

void pack_Rpy(RpyTypeDef *frame, float yaw, float pitch, float roll)
{
    int8_t rpy_tx_buffer[FRAME_RPY_LEN] = {0} ;
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

    memcpy(&frame->DATA[0], rpy_tx_buffer, 13);

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

    for (int i = 0; i < frame->LEN; i++)
    {
        sum += frame->DATA[i];
        add += sum;
    }

    frame->SC = sum & 0xFF;
    frame->AC = add & 0xFF;
}



void process_usb_data(uint8_t* Buf, uint32_t *Len)
{
    // 将收到的数据放入环形缓冲区
    ring_put_force(&usb_rx_ring, Buf, *Len);

    uint8_t frame[sizeof(RpyTypeDef)];

    // 循环提取所有完整帧
    while (ring_available(&usb_rx_ring) >= sizeof(RpyTypeDef)) {
        // 查找帧头 0xFF，跳过无效字节
        uint8_t byte;
        while (ring_available(&usb_rx_ring) > 0) {
            byte = usb_rx_ring.buffer[usb_rx_ring.read_idx % USB_RX_RING_SIZE];
            if (byte == 0xFF) break;
            ring_get(&usb_rx_ring, &byte, 1);
        }

        if (ring_available(&usb_rx_ring) < sizeof(RpyTypeDef)) break;

        // 读取完整帧
        ring_get(&usb_rx_ring, frame, sizeof(RpyTypeDef));
        memcpy(&rpy_rx_data, frame, sizeof(rpy_rx_data));

        switch (rpy_rx_data.ID) {
            case CHASSIS_CTRL: {
                trans_fdb_data.linear_x = (*(int32_t *)&rpy_rx_data.DATA[0] / 10000.0);
                trans_fdb_data.linear_y = (*(int32_t *)&rpy_rx_data.DATA[4] / 10000.0);
                trans_fdb_data.linear_z = (*(int32_t *)&rpy_rx_data.DATA[8] / 10000.0);
                trans_fdb_data.angular_x = (*(int32_t *)&rpy_rx_data.DATA[12] / 10000.0);
                trans_fdb_data.angular_y = (*(int32_t *)&rpy_rx_data.DATA[16] / 10000.0);
                trans_fdb_data.angular_z = (*(int32_t *)&rpy_rx_data.DATA[20] / 10000.0);
            } break;

            case GIMBAL: {
                trans_fdb_data.yaw = -(*(int32_t *)&rpy_rx_data.DATA[1] / 1000.0);
                trans_fdb_data.pitch = (*(int32_t *)&rpy_rx_data.DATA[5] / 1000.0);
                trans_fdb_data.roll = (*(int32_t *)&rpy_rx_data.DATA[9] / 1000.0);
                trans_fdb_data.mode = (*(int32_t *)&rpy_rx_data.DATA[13] / 1000.0);
                yaw_filtered = 0.1f * trans_fdb_data.yaw + 0.9f * yaw_filtered;
                pitch_filtered = 0.1f * trans_fdb_data.pitch + 0.9f * pitch_filtered;
                trans_fdb_data.yaw_filtered = yaw_filtered;
                trans_fdb_data.pitch_filtered = pitch_filtered;
            } break;

            case POSE_CTRL: {
                trans_fdb_data.pose = (*(uint8_t *)&rpy_rx_data.DATA[0]);
                if (trans_fdb_data.pose == 3) {
                    trans_fdb_data.chassis_power_limit = 150;
                    trans_fdb_data.shooter_17mm_cooling_heat = 10 / 3;
                } else if (trans_fdb_data.pose == 2) {
                    trans_fdb_data.chassis_power_limit = 50;
                    trans_fdb_data.shooter_17mm_cooling_heat = 10 / 3;
                } else if (trans_fdb_data.pose == 1) {
                    trans_fdb_data.chassis_power_limit = 50;
                    trans_fdb_data.shooter_17mm_cooling_heat = 30;
                }
            } break;

            case HEARTBEAT: {
                trans_fdb_data.heartbeat = (*(uint8_t *)&rpy_rx_data.DATA[0]);
                heart_dt = dwt_get_time_ms();
            } break;
        }

        memset(&rpy_rx_data, 0, sizeof(rpy_rx_data));
    }
}

// 提供获取 trans_fdb_data 数据的函数
struct trans_fdb_msg* get_trans_fdb(void)
{
    return &trans_fdb_data;
}

/******************************************************消息订阅*************************************************************************/
void trans_pub_push(){
    // data_content my_data = ;
    mcn_publish(MCN_HUB(transmission_fdb_topic), &trans_fdb_data);
}

void trans_sub_init(){
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb_topic), NULL, NULL);
    gimbal_ins_node =mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
}

void trans_sub_pull(){

    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chass_cmd);
    }
    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gimbal_cmd);
    }
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb_topic), gimbal_fdb_node, &gimbal_fdb);
    }
    if (mcn_poll(gimbal_ins_node))
    {
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
    }
}
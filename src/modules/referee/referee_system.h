//
// Created by ���ο� on 25-2-25.
//

#ifndef CTRBOARD_H7_ALL_REFEREE_SYSTEM_H
#define CTRBOARD_H7_ALL_REFEREE_SYSTEM_H

#include "main.h"

#define HEADER_SOF 0xA5
#define REFEREE_RX_BUF_SIZE (127+9+68+5)        // 209

#define CUSTOMER_CONTROLLER_BUF_SIZE (30+9+20+4)   // 63


/**
*   ����Э�����ݵ�֡ͷ���ݽṹ��
*/
typedef struct __attribute__((__packed__))
{
    uint8_t sof;                           /*! ����֡��ʼ�ֽڣ��̶�ֵΪ 0xA5 */
    uint16_t data_length;                  /*! ����֡�� data �ĳ��� */
    uint8_t seq;                           /*! ����� */
    uint8_t crc8;                          /*! ֡ͷ CRC8У�� */
} referee_data_header_t;

#define REF_PROTOCOL_HEADER_SIZE            sizeof(referee_data_header_t)  // Э��֡ͷ��С 5�ֽ�
#define REF_PROTOCOL_CMD_SIZE               2  // Э�������ֽڴ�С
#define REF_PROTOCOL_CRC16_SIZE             2  // Э�� CRC16 У���С
#define REF_HEADER_CRC_CMD_SIZE            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + REF_PROTOCOL_CMD_SIZE)  // ֡ͷ��������� CRC ����
#define REF_PROTOCOL_DATA_MAX_SIZE         127  //TODO���������ݶδ�С�������壬��127����//(127 - REF_HEADER_CRC_CMD_SIZE)   // ���ɴ������ݴ�С��������0x301�����˽���������󳤶�Ϊ127�ֽ�(�Ѿ�����֡ͷ֡β��������������6�ֽڣ������ݰ���С����Ϊ112�ֽ�
#define REF_PROTOCOL_FRAME_MAX_SIZE         (REF_HEADER_CRC_CMD_SIZE+REF_PROTOCOL_DATA_MAX_SIZE)  // Э��֡����С

/**
*   ���յ�Э������
*/
typedef struct __attribute__((__packed__))
{
    referee_data_header_t* frame_header;        /*! ֡ͷ���ݽṹ�� */
    uint16_t cmd_id;                          /*! ������ ID */
    uint8_t* data;                            /*! ���յ�����ָ�� */
    uint16_t frame_tail;                      /*! frame_tail */
} referee_data_t;

typedef enum
{
    STEP_HEADER_SOF  = 0,
    STEP_DATA_SIZE_LOW  = 1,
    STEP_DATA_SIZE_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

/**
*   �������
*/
typedef struct __attribute__((__packed__))
{
    referee_data_header_t *p_header;
    unpack_step_e  unpack_step;
    uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
    uint16_t       index;
} unpack_data_t;

typedef enum
{
    GAME_STATUS_CMD_ID                = 0x0001,  // ����״̬���� ���� 11
    GAME_RESULT_CMD_ID                = 0x0002,  // ����������� ���� 1
    GAME_ROBOT_HP_CMD_ID              = 0x0003,  // ������Ѫ������ ���� 32
    FIELD_EVENTS_CMD_ID               = 0x0101,  // �����¼����� ���� 4
    REFEREE_WARNING_CMD_ID            = 0x0104,  // ���о������� ���� 3
    DART_FIRE_CMD_ID                  = 0x0105,  // ���ڷ������� ���� 3
    ROBOT_STATUS_CMD_ID               = 0x0201,  // ������״̬���������� ���� 13
    POWER_HEAT_DATA_CMD_ID            = 0x0202,  // ʵʱ���̻�������������������� ���� 16
    ROBOT_POS_CMD_ID                  = 0x0203,  // ������λ������ ���� 16
    BUFF_MUSK_CMD_ID                  = 0x0204,  // ����������͵����������� ���� 7
    ROBOT_HURT_CMD_ID                 = 0x0206,  // �˺�״̬���� ���� 1
    SHOOT_DATA_CMD_ID                 = 0x0207,  // ʵʱ������� ���� 7
    BULLET_REMAINING_CMD_ID           = 0x0208,  // ʣ���ӵ����� ���� 6
    ROBOT_RFID_CMD_ID                 = 0x0209,  // ������RFIDģ��״̬���� ���� 4
    DART_DIRECTIONS_CMD_ID            = 0x020A,  // ����ѡ�ֶ�ָ������ ���� 6
    ROBOT_LOCATION_CMD_ID             = 0x020B,  // ���������λ������ ���� 40
    RADAR_PROGRESS_CMD_ID             = 0x020C,  // �״��ǽ������� ���� 1
    SENTRY_AUTONOMY__CMD_ID           = 0x020D,  // �ڱ�����������Ϣͬ������ ���� 6
    RADAR_AUTONOMY_CMD_ID             = 0x020E,  // �״�����������Ϣͬ������ ���� 1
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301,  // �����˽������ݣ����ͷ��������� ���� 127
    ARM_DATA_FROM_CONTROLLER_CMD_ID_2  = 0x0302,  // �Զ��������������˽������ݣ����ͷ��������� ���� 30
    PLAYER_MINIMAP_CMD_ID             = 0x0303,  // ѡ�ֶ�С��ͼ�������� ���� 15
    KEYBOARD_MOUSE_CMD_ID             = 0x0304,  // ����ң������ ���� 12
    RADAR_MINIMAP_CMD_ID              = 0x0305,  // ѡ�ֶ�С��ͼ�����״����� ���� 24
    CUSTOMER_CONTROLLER_PLAYER_CMD_ID = 0x0306,  // �Զ����������ѡ�ֶ˽������� ���� 8
    PLAYER_MINIMAP_SENTRY_CMD_ID      = 0x0307,  // ѡ�ֶ�С��ͼ�����ڱ����� ���� 103
    PLAYER_MINIMAP_ROBOT_CMD_ID       = 0x0308,  // ѡ�ֶ�С��ͼ���ջ��������� ���� 34
    ARM_DATA_FROM_CONTROLLER_CMD_ID_9   = 0x0309,  // �Զ�����������ջ��������� ���� 30
} referee_cmd_id_t;

/**
*   ����״̬����         ��Ӧ��������IDΪ��0x0001
*/
typedef struct __attribute__((__packed__))
{
    uint8_t game_type : 4;                      /*! ���������� */
    uint8_t game_progress : 4;                  /*! ��ǰ�����׶� */
    uint16_t stage_remain_time;                 /*! ��ǰ�׶�ʣ��ʱ�� */
    uint64_t SyncTimeStamp;                     /*! �����˽��յ���ָ��ľ�ȷ Unix ʱ�� */
} game_status_t;

/**
*   �����������         ��Ӧ��������IDΪ��0x0002
*/
typedef struct __attribute__((__packed__))
{
    uint8_t winner;                             /*! 0 ƽ�� 1 �췽ʤ�� 2 ����ʤ�� */
} game_result_t;

/**
*   ������Ѫ������         ��Ӧ��������IDΪ��0x0003
*/
typedef struct __attribute__((__packed__))
{
    uint16_t red_1_robot_HP;            /*! �� 1 Ӣ�ۻ�����Ѫ�������û�����δ�ϳ����߱����£���Ѫ��Ϊ 0 */
    uint16_t red_2_robot_HP;            /*! �� 2 ���̻�����Ѫ�� */
    uint16_t red_3_robot_HP;            /*! �� 3 ����������Ѫ�� */
    uint16_t red_4_robot_HP;            /*! �� 4 ����������Ѫ�� */
    uint16_t reserved_1;                  /*! ����λ 1 */
    uint16_t red_7_robot_HP;            /*! �� 7 �ڱ�������Ѫ�� */
    uint16_t red_outpost_HP;            /*! �췽ǰ��վѪ�� */
    uint16_t red_base_HP;               /*! �췽����Ѫ�� */
    uint16_t blue_1_robot_HP;           /*! �� 1 Ӣ�ۻ�����Ѫ�� */
    uint16_t blue_2_robot_HP;           /*! �� 2 ���̻�����Ѫ�� */
    uint16_t blue_3_robot_HP;           /*! �� 3 ����������Ѫ�� */
    uint16_t blue_4_robot_HP;           /*! �� 4 ����������Ѫ�� */
    uint16_t reserved_2;                  /*! ����λ 2 */
    uint16_t blue_7_robot_HP;           /*! �� 7 �ڱ�������Ѫ�� */
    uint16_t blue_outpost_HP;           /*! ����ǰ��վѪ�� */
    uint16_t blue_base_HP;              /*! ��������Ѫ�� */
} game_robot_HP_t;

/**
    0��δռ��/δ����         ��Ӧ��������IDΪ��0x0101
    1����ռ��/�Ѽ���
    bit 0-2��
    ? bit 0��������һ������ص��Ĳ�����ռ��״̬��1 Ϊ��ռ��
    ? bit 1��������һ����ص��Ĳ�����ռ��״̬��1 Ϊ��ռ��
    ? bit 2��������������ռ��״̬��1 Ϊ��ռ�죨�� RMUL ���ã�
    bit 3-5��������������״̬
    ? bit 3������С�������صļ���״̬��1 Ϊ�Ѽ���
    ? bit 4���������������صļ���״̬��1 Ϊ�Ѽ���
    ? bit 5-6����������ߵص�ռ��״̬��1 Ϊ������ռ�죬2 Ϊ���Է�ռ��
    ? bit 7-8���������θߵص�ռ��״̬��1 Ϊ��ռ��
    ? bit 9-17���Է��������һ�λ��м���ǰ��վ����ص�ʱ�䣨0-420����
    ��Ĭ��Ϊ 0��
    ? bit 18-20���Է��������һ�λ��м���ǰ��վ����صľ���Ŀ�꣬����
    Ĭ��Ϊ 0��1 Ϊ����ǰ��վ��2 Ϊ���л��ع̶�Ŀ�꣬3 Ϊ���л������
    �̶�Ŀ�꣬4 Ϊ���л�������ƶ�Ŀ��
    ? bit 21-22������������ռ��״̬��0 Ϊδ��ռ�죬1 Ϊ������ռ�죬2
    Ϊ���Է�ռ�죬3 Ϊ��˫��ռ�졣���� RMUL ���ã�
    ? bit 23-31������λ
 */
typedef struct __attribute__((__packed__))
{
    uint32_t event_data;
} event_data_t;

/**
 *   ���о�����Ϣ         ��Ӧ��������IDΪ��0x0104
 *   ����Ƶ�ʣ��������淢������
 *   �������һ���ܵ��з��ĵȼ���
 *          1��˫������
 *          2������
 *          3������
 *          4���и�
 *    ��������� ID��
 *          �������һ���ܵ��з���Υ������� ID������� 1 ������ ID Ϊ 1����1 ������ ID Ϊ 101��
 *          �и���˫������ʱ����ֵΪ 0
 *    Υ�������
 *          �������һ���ܵ��з���Υ������˶�Ӧ�з��ȼ���Υ�������������Ĭ��Ϊ 0����
*/
typedef struct __attribute__((__packed__))
{
    uint8_t level;                              /*! ����ȼ� */
    uint8_t offending_robot_id;                      /*! ��������� ID */
    uint8_t count;                                /*! Υ����� */
} referee_warning_t;

/**
 *   ���ڷ���ڵ���ʱ         ��Ӧ��������IDΪ��0x0105
 *   ����Ƶ�ʣ�1Hz ���ڷ��ͣ����ͷ�Χ������������
 *   ʣ��ʱ�䣺
 *         �������ڷ���ʣ��ʱ�䣬��λ����
 *   ����Ŀ�꣺
 *   bit 0-2��
 *      ���һ�μ������ڻ��е�Ŀ�꣬����Ĭ��Ϊ 0��1 Ϊ����ǰ��վ��2 Ϊ����
 *      ���ع̶�Ŀ�꣬3 Ϊ���л�������̶�Ŀ�꣬4 Ϊ���л�������ƶ�Ŀ��
 *   bit 3-5��
 *      �Է���������е�Ŀ���ۼƱ����мƴ���������Ĭ��Ϊ 0������Ϊ 4
 *   bit 6-7��
 *      ���ڴ�ʱѡ���Ļ���Ŀ�꣬����Ĭ�ϻ�δѡ��/ѡ��ǰ��վʱΪ 0��ѡ�л�
 *      �ع̶�Ŀ��Ϊ 1��ѡ�л�������̶�Ŀ��Ϊ 2��ѡ�л�������ƶ�Ŀ��Ϊ 3
 *   bit 8-15������
*/
typedef struct __attribute__((__packed__))
{
    uint8_t dart_remaining_time;            /*! 15s ����ʱ */
    uint16_t dart_info;
} dart_info_t;


/**
 *   ����������״̬         ��Ӧ��������IDΪ��0x0201
 *   ����Ƶ�ʣ�10Hz
 *    �������� ID��
 *            1���췽Ӣ�ۻ����ˣ�
 *            2���췽���̻����ˣ�
 *            3/4/5���췽���������ˣ�
 *            6���췽���л����ˣ�
 *            7���췽�ڱ������ˣ�
 *            8���췽���ڻ����ˣ�
 *            9���췽�״�վ��
 *            101������Ӣ�ۻ����ˣ�
 *            102���������̻����ˣ�
 *            103/104/105���������������ˣ�
 *            106���������л����ˣ�
 *            107�������ڱ������ˣ�
 *            108���������ڻ����ˣ�
 *            109�������״�վ��
 *   ���ص�Դ��������
 *            gimbal ������� 1 Ϊ�� 24V �����0 Ϊ�� 24v �����
 *            chassis �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
 *            shooter �������1 Ϊ�� 24V �����0 Ϊ�� 24v �����
*/
typedef struct __attribute__((__packed__))
{
    uint8_t robot_id;                               /*! �������� ID */
    uint8_t robot_level;                            /*! �����˵ȼ� */
    uint16_t current_HP;                             /*! ������ʣ��Ѫ�� */
    uint16_t maximum_HP;                                /*! ����������Ѫ�� */
    uint16_t shooter_barrel_cooling_value;             /*!������ǹ������ÿ����ȴֵ*/
    uint16_t shooter_barrel_heat_limit;                /*!������ǹ����������*/
    uint16_t chassis_power_limit;                   /*! �����˵��̹����������� */
    uint8_t power_management_gimbal_output : 1;          /*! ���ص�Դ��������gimbal ����� */
    uint8_t power_management_chassis_output : 1;         /*! ���ص�Դ��������chassis ����� */
    uint8_t power_management_shooter_output : 1;        /*! ���ص�Դ��������shooter ����� */
} robot_status_t;

/**
 *   ʵʱ������������   ��Ӧ������IDΪ��0x0202
 *   ����Ƶ�ʣ�50Hz
 *   ����λ
 *   ����λ
 *   ����λ
 *   ���̹��ʻ��� ��λ J ���� ��ע�����¸��ݹ��������� 250J
 *   1 �� 17mm ǹ������
 *   2 �� 17mm ǹ������
 *   42mm ǹ������
 */
typedef struct __attribute__((__packed__))
{
    uint16_t reserved_1;                          /*!���������ѹ*/
    uint16_t reserved_2;                          /*!�����������*/
    float reserved_3;;                               /*!�����������*/
    uint16_t buffer_energy;                            /*!���̹��ʻ���*/
    uint16_t shooter_17mm_1_barrel_heat;               /*!������1 �� 17mm ǹ������*/
    uint16_t shooter_17mm_2_barrel_heat;               /*!������2 �� 17mm ǹ������*/
    uint16_t shooter_42mm_barrel_heat;                 /*!������42mm ǹ������*/
} power_heat_data_t;

/**
 * ������λ��    ��Ӧ������IDΪ��0x0203
 * ����Ƶ�ʣ�10Hz
 * λ�� x ���꣬��λ m
 * λ�� y ���꣬��λ m
 * λ��ǹ�ڣ���λ��
 */
typedef  struct __attribute__((__packed__))
{
    float x;                               /*!λ�� x ����*/
    float y;                               /*!λ�� y ����*/
    float angle;                           /*!�������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ 0 ��*/
} robot_pos_t;

/**
* ����������  ��Ӧ������IDΪ��0x0204
* ����Ƶ�ʣ�1Hz
* ������Ѫ����Ѫ״̬
* ǹ��������ȴ����
* �����˷����ӳ�
* �����˹����ӳ�
* ���� bit ����
*/
typedef  struct __attribute__((__packed__))
{
    uint8_t recovery_buff;               /*!�����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��ʾÿ��ָ�Ѫ�����޵� 10%��*/
    uint8_t cooling_buff;                /*!������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ʾ 5 ����ȴ��*/
    uint8_t defence_buff;                /*!�����˷������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩*/
    uint8_t vulnerability_buff;          /*!�����˸��������棨�ٷֱȣ�ֵΪ 30 ��ʾ-30%�������棩*/
    uint16_t attack_buff;                /*!�����˹������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩*/
    uint8_t remaining_energy;            /*!������ʣ���������ٷֱȣ�ֵΪ 50 ��ʾʣ������Ϊ 50%��*/
/*
* bit 0-4��������ʣ������ֵ�������� 16 ���Ʊ�ʶ������ʣ������ֵ���������ڻ�����ʣ������С�� 50%ʱ����������Ĭ�Ϸ��� 0x32��bit 0����ʣ��������50%ʱΪ 1���������Ϊ 0
* ? bit 1����ʣ��������30%ʱΪ 1���������Ϊ 0
* ? bit 2����ʣ��������15%ʱΪ 1���������Ϊ 0
* ? bit 3����ʣ��������5%ʱΪ 1���������Ϊ 0
* ? bit 4����ʣ��������1%ʱΪ 1���������Ϊ 0
*/
} buff_t;

/**
*  �˺�״̬  ��Ӧ������IDΪ��0x0206
*  ����Ƶ�ʣ��˺���������
*  bit 0-3������Ѫԭ��Ϊװ��ģ�鱻���蹥������ײ�������߻����ģ������ʱ��
*  �� 4 bit ��ɵ���ֵΪװ��ģ������ģ��� ID ��ţ�
*  ������ԭ���¿�Ѫʱ������ֵΪ 0
*  bit 4-7��Ѫ���仯����
*  0x0 װ��ģ�鱻���蹥�����¿�Ѫ
*  0x1 ����ϵͳ��Ҫģ�����ߵ��¿�Ѫ
*  0x2 ������ٶȳ��޵��¿�Ѫ
*  0x3 ����������޵��¿�Ѫ
*  0x4 ���̹��ʳ��޵��¿�Ѫ
*  0x5 װ��ģ���ܵ�ײ�����¿�Ѫ
*/
typedef struct __attribute__((__packed__))
{
    uint8_t armor_id : 4;                    /*!װ��ID */
    uint8_t HP_deduction_reason : 4;         /*!Ѫ���仯ԭ��*/
} hurt_data_t;

/**
*  ʵʱ�����Ϣ   ��Ӧ������IDΪ��0x0207
*  ����Ƶ�ʣ��������
*  �ӵ�����: 1��17mm ���� 2��42mm ����
*  ������� ID��
*  1��1 �� 17mm �������
*  2��2 �� 17mm �������
*  3��42mm �������
* �ӵ���Ƶ ��λ Hz
* �ӵ����� ��λ m/s
*/
typedef struct __attribute__((__packed__))
{
    uint8_t bullet_type;                 /*!�ӵ�����*/
    uint8_t shooter_number;              /*!����ǹ��ID*/
    uint8_t launching_frequency;         /*!�ӵ���Ƶ*/
    float initial_speed;                 /*!�ӵ�����*/
} shoot_data_t;

/**
*  �ӵ�ʣ�෢����    ��Ӧ������IDΪ��0x0208
*  ����Ƶ�ʣ�10Hz ���ڷ��ͣ����л����˷���
*   17mm �ӵ�ʣ�෢����������˵��             ������                              �Կ���
*    ����������                   ȫ�Ӳ�����Ӣ��ʣ��ɷ��� 17mm ��������          ȫ�� 17mm ����ʣ��ɶһ�����
*    Ӣ�ۻ�����                   ȫ�Ӳ�����Ӣ��ʣ��ɷ��� 17mm ��������          ȫ�� 17mm ����ʣ��ɶһ�����
*    ���л����ˡ��ڱ�������         �û�����ʣ��ɷ��� 17mm ��������               �û�����ʣ��ɷ��� 17mm ��������
*
*   17mm �ӵ�ʣ�෢����Ŀ
*   42mm �ӵ�ʣ�෢����Ŀ
*   ʣ��������
*/
typedef struct __attribute__((__packed__))
{
    uint16_t projectile_allowance_17mm;                 /*!17mm��ͷʣ������*/
    uint16_t projectile_allowance_42mm;                 /*!42mm��ͷʣ������*/
    uint16_t remaining_gold_coin;                       /*!���ʣ������*/
} projectile_allowance_t;

/**
*  ������ RFID ״̬  ��Ӧ������IDΪ��0x0209
*  ����Ƶ�ʣ�1Hz    ���ͷ�Χ������װ�� RFIDģ��Ļ�����
*  bit λֵΪ 1/0 �ĺ��壺�Ƿ��Ѽ�⵽������� RFID ��
? bit 0���������������
? bit 1����������ߵ������
? bit 2���Է�����ߵ������
? bit 3���������θߵ������
? bit 4���Է����θߵ������
? bit 5���������ο�Խ����㣨���£�����������һ�����ǰ��
? bit 6���������ο�Խ����㣨���£�����������һ����º�
? bit 7���Է����ο�Խ����㣨���£��������Է�һ�����ǰ��
? bit 8���Է����ο�Խ����㣨���£��������Է�һ����º�
? bit 9���������ο�Խ����㣨����ߵ��·���
? bit 10���������ο�Խ����㣨����ߵ��Ϸ���
? bit 11���Է����ο�Խ����㣨����ߵ��·���
? bit 12���Է����ο�Խ����㣨����ߵ��Ϸ���
? bit 13���������ο�Խ����㣨��·�·���
? bit 14���������ο�Խ����㣨��·�Ϸ���
? bit 15���Է����ο�Խ����㣨��·�·���
? bit 16���Է����ο�Խ����㣨��·�Ϸ���
? bit 17���������������
? bit 18������ǰ��վ�����
? bit 19��������һ������ص��Ĳ�����/RMUL ������
? bit 20��������һ����ص��Ĳ�����
? bit 21����������Դ�������
? bit 22���Է�����Դ�������
? bit 23����������㣨�� RMUL ���ã�
? bit 24-31������
ע������ RFID ������������Ч�������⣬��ʹ��⵽��Ӧ�� RFID ������
ӦֵҲΪ 0��
* ע����������㡢�ߵ�����㡢��������㡢ǰ��վ����㡢��Դ������㡢
* ��Ѫ�㡢�һ�������������㣨�������� RMUL�����ڱ�Ѳ������ RFID ��
* ����������Ч�������⣬��ʹ��⵽��Ӧ�� RFID ������ӦֵҲΪ 0��
*/
typedef struct __attribute__((__packed__))
{
    uint32_t rfid_status;                        /*!������RFID״̬ */
} rfid_status_t;

/**
* ���ڻ����˿ͻ���ָ������   ��Ӧ������IDΪ��0x020A
* ����Ƶ�ʣ�10Hz ���ͷ�Χ���������ڻ�����
*  ��ǰ���ڷ���վ��״̬
*  1���رգ�
*  2�����ڿ������߹ر���
*  0���Ѿ�����
*  ����
*  �л�����Ŀ��ʱ�ı���ʣ��ʱ�䣬��λ���룬��/δ�л�������Ĭ��Ϊ 0��
*  ���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ�䣬��λ��, ��ʼֵΪ 0��
*/
typedef struct __attribute__((__packed__))
{
    uint8_t dart_launch_opening_status;             /*!���ڷ���վ״̬*/
    uint8_t reserved;                               /*!����*/
    uint16_t target_change_time;                    /*!�л����Ŀ��ʱ�ı���ʣ��ʱ��*/
    uint16_t latest_launch_cmd_time;               /*!���һ�β�����ȷ������ָ��ʱ�ı���ʣ��ʱ��*/
} dart_client_cmd_t;

/**
*  ���������λ������   ��Ӧ������IDΪ��0x020B
*  ����Ƶ�ʣ�1Hz ���ͷ�Χ�������ڱ�������
*  ��λ��m
*
*/
typedef struct __attribute__((__packed__))
{
    float hero_x;       /*!����Ӣ�ۻ�����λ�� x ������*/
    float hero_y;       /*!����Ӣ�ۻ�����λ�� y ������*/
    float engineer_x;   /*!�������̻�����λ�� x ������*/
    float engineer_y;   /*!�������̻�����λ�� y ������*/
    float standard_3_x; /*!���� 3 �Ų���������λ�� x ������*/
    float standard_3_y; /*!���� 3 �Ų���������λ�� y ������*/
    float standard_4_x; /*!���� 4 �Ų���������λ�� x ������*/
    float standard_4_y; /*!���� 4 �Ų���������λ�� y ������*/
    float reserved_1; /*!���� 5 �Ų���������λ�� x ������*/
    float reserved_2; /*!���� 5 �Ų���������λ�� y ������*/
} ground_robot_position_t;

/**
*  �״��ǽ�������  ��Ӧ������IDΪ��0x020C
*  ����Ƶ�ʣ�1Hz ���ͷ�Χ�������״������
*  �ڶ�Ӧ�����˱���ǽ��ȡ�100 ʱ���� 1������ǽ���<100 ʱ���� 0��
? bit 0���Է� 1 ��Ӣ�ۻ������������
? bit 1���Է� 2 �Ź��̻������������
? bit 2���Է� 3 �Ų����������������
? bit 3���Է� 4 �Ų����������������
? bit 4���Է��ڱ��������������
*/

typedef struct __attribute__((__packed__))
{
    uint8_t mark_progress;
} radar_mark_data_t;

/**
*  �ڱ�����������Ϣͬ��  ��Ӧ������IDΪ��0x020D
*  ����Ƶ�ʣ�1Hz ���ͷ�Χ�������ڱ�������
* bit 0-10��
*     ��Զ�̶һ��⣬�ڱ��ɹ��һ��ķ�����������Ϊ 0�����ڱ��ɹ���
*     ��һ���������󣬸�ֵ����Ϊ�ڱ��ɹ��һ��ķ�����ֵ��
* bit 11-14��
*     �ڱ��ɹ�Զ�̶һ��������Ĵ���������Ϊ 0�����ڱ��ɹ�Զ�̶�
*     ���������󣬸�ֵ����Ϊ�ڱ��ɹ�Զ�̶һ��������Ĵ�����
* bit 15-18��
*     �ڱ��ɹ�Զ�̶һ�Ѫ���Ĵ���������Ϊ 0�����ڱ��ɹ�Զ�̶һ�
*     Ѫ���󣬸�ֵ����Ϊ�ڱ��ɹ�Զ�̶һ�Ѫ���Ĵ�����
* bit 19���ڱ������˵�ǰ�Ƿ����ȷ����Ѹ������ȷ����Ѹ���ʱֵΪ1������Ϊ 0��
* bit 20���ڱ������˵�ǰ�Ƿ���Զһ�����������Զһ���������ʱֵΪ1������Ϊ 0��
* bit 21-30���ڱ������˵�ǰ���һ�����������Ҫ���ѵĽ������
* bit 31��������

bit 0���ڱ���ǰ�Ƿ�����ս״̬��������ս״̬ʱΪ 1������Ϊ 0��
bit 1-11������ 17mm ������������ʣ��ɶһ�����
bit 12-15��������
*/

typedef struct __attribute__((__packed__))
{
    uint32_t sentry_info;
    uint16_t sentry_info_2;
} sentry_info_t;

/**
*  �״�����������Ϣͬ��  ��Ӧ������IDΪ��0x020E
*  ����Ƶ�ʣ�1Hz ���ͷ�Χ�������״������
* bit 0-1��
*     �״��Ƿ�ӵ�д���˫�����˵Ļ��ᣬ����Ϊ 0��
*     ��ֵΪ�״�ӵ�д���˫�����˵Ļ��ᣬ����Ϊ 2
* bit 2��
*     �Է��Ƿ����ڱ�����˫������
*      0���Է�δ������˫������
*      1���Է����ڱ�����˫������
* bit 3-7������
*/
typedef struct __attribute__((__packed__))
{
    uint8_t radar_info;
} radar_info_t;

/**
 *  �������ݽ�����Ϣ   ��Ӧ������IDΪ��0x0301
 * ������ ID��
 *      ��Ϊ���ŵ������� ID
 * �����ߵ� ID:
 *      ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ������ 1 ���͸��� 5��������ҪУ��� 1
 * �����ߵ� ID:
 *      ���޼���ͨ��
 *      ��Ϊ���������Ķ��ͨѶ������
 *      ��������Ϊѡ�ֶˣ�����ɷ����������߶�Ӧ��ѡ�ֶ�
 *      ID ��������¼
 * �������ݶ�:
 *  ���Ϊ 113
*/
/*
�����˽�������ͨ��������·���ͣ������ݶΰ���һ��ͳһ�����ݶ�ͷ�ṹ�����ݶ�ͷ�ṹ�������� ID��
�����ߺͽ����ߵ� ID���������ݶΡ������˽������ݰ����ܳ������� 127 ���ֽڣ���ȥ frame_header��
cmd_id �� frame_tail �� 9 ���ֽ��Լ����ݶ�ͷ�ṹ�� 6 ���ֽڣ��ʻ����˽������ݵ��������ݶ����
Ϊ 112 ���ֽڡ�
ÿ 1000 ���룬Ӣ�ۡ����̡����������л����ˡ������ܹ��������ݵ�����Ϊ 3720 �ֽڣ��״���ڱ�����
���ܹ��������ݵ�����Ϊ 5120 �ֽڡ�
���ڴ��ڶ������ ID�������� cmd_id ����Ƶ�����Ϊ 30Hz����������Ŵ�����
*/
typedef struct __attribute__((__packed__))
{
    uint16_t data_cmd_id;                           /*! ������ ID*/
    uint16_t sender_id;                             /*! ������ ID */
    uint16_t receiver_id;                           /*! ������ ID */
    uint8_t user_data[112];                              /*! �������ݶ� �������Ϊ112�ֽ�*/
} robot_interaction_data_t;
//TODO:�����˽������������ݵ�����
//TODO:δ������������ָ��
/** �������ݰ���ͼ������ ����ID */
typedef enum
{
    //0x200-0x02ff 	�����Զ������� ��ʽ  INTERACT_ID_XXXX
    DELETE_GRAPHIC_ID = 0x0100,  //�ͻ���ɾ��ͼ��
    DRAW_ONE_GRAPHIC_ID = 0x0101,  //�ͻ��˻���һ��ͼ��
    DRAW_TWO_GRAPHIC_ID = 0x0102,  //�ͻ��˻��ƶ���ͼ��
    DRAW_FIVE_GRAPHIC_ID = 0x0103,  //�ͻ��˻������ͼ��
    DRAW_SEVEN_GRAPHIC_ID = 0x0104,  //�ͻ��˻����߸�ͼ��
    DRAW_CHAR_GRAPHIC_ID = 0x0110,  //�ͻ��˻����ַ�ͼ��

    ID_delete_graphic 			= 0x0100,  //�ͻ���ɾ��ͼ��
    ID_draw_one_graphic 		= 0x0101,  //�ͻ��˻���һ��ͼ��
    ID_draw_two_graphic 		= 0x0102,  //�ͻ��˻��ƶ���ͼ��
    ID_draw_five_graphic 	  = 0x0103,  //�ͻ��˻������ͼ��
    ID_draw_seven_graphic 	= 0x0104,  //�ͻ��˻����߸�ͼ��
    ID_draw_char_graphic 	  = 0x0110,  //�ͻ��˻����ַ�ͼ��
} ui_cmd_id_e;

/* ���ݶγ��� */
enum
{
    DELETE_GRAPHIC_LEN = 8,
    DRAW_ONE_GRAPHIC_LEN = 21,
    DRAW_TWO_GRAPHIC_LEN = 36,
    DRAW_FIVE_GRAPHIC_LEN = 81,
    DRAW_SEVEN_GRAPHIC_LEN = 111,
    DRAW_CHAR_GRAPHIC_LEN = 51,

    LEN_ID_delete_graphic     = 8,  //6+2
    LEN_ID_draw_one_graphic   = 21, //6+15
    LEN_ID_draw_two_graphic   = 36, //6+15*2
    LEN_ID_draw_five_graphic  = 81, //6+15*5
    LEN_ID_draw_seven_graphic = 111,//6+15*7
    LEN_ID_draw_char_graphic  = 51, //6+15+30���ַ������ݣ�
};

/* �������� */
typedef enum
{
    NONE   = 0,/*�ղ���*/
    ADD    = 1,/*����ͼ��*/
    MODIFY = 2,/*�޸�ͼ��*/
    DELETE = 3,/*ɾif��ͼ��*/
} operate_tpye_e;

/* ��ɫ */
typedef enum
{
    RED_BLUE  = 0,  //������ɫ
    YELLOW    = 1,  //��
    GREEN     = 2,  //��
    ORANGE    = 3,  //��
    FUCHSIA   = 4,	//�Ϻ�ɫ
    PINK      = 5,   //��
    CYAN_BLUE = 6,	//��ɫ
    BLACK     = 7,
    WHITE     = 8
}graphic_color_e;

typedef struct __attribute__((__packed__))
{
    uint8_t operate_tpye;
    uint8_t layer;
} ext_client_custom_graphic_delete_t;

typedef struct __attribute__((__packed__))
{
    uint8_t delete_type;
    uint8_t layer;
} interaction_layer_delete_t;

/* ͼ������ */
typedef struct __attribute__((__packed__))
{
    uint8_t graphic_name[3];
    uint32_t operate_tpye:3;       /* 0:�ղ���;1:����;2:�޸�;3:ɾ��	*/
    uint32_t graphic_tpye:3;        /*	0:ֱ��;1:����;2:��Բ;3:��Բ;4:Բ��;5:������;6:����;7:�ַ� */
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t start_angle:9;
    uint32_t end_angle:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t radius:10;
    uint32_t end_x:11;
    uint32_t end_y:11;
} graphic_data_struct_t;

typedef struct __attribute__((__packed__))
{
    uint8_t figure_name[3];
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} interaction_figure_t;

/* �ͻ��˻���һ��ͼ�����ݶ� */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

/* �ͻ��˻��ƶ���ͼ�����ݶ� */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef struct __attribute__((__packed__))
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

/* �ͻ��˻������ͼ�����ݶ� */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

typedef struct __attribute__((__packed__))
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

/* �ͻ��˻����߸�ͼ�����ݶ� */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef struct __attribute__((__packed__))
{
    interaction_figure_t interaction_figure[7];
}interaction_figure_4_t;

/* �ͻ��˻����ַ����ݶ� */
typedef struct __attribute__((__packed__))
{
    graphic_data_struct_t grapic_data_struct;
    char data[30];
} ext_client_custom_character_t;

//typedef struct __attribute__((__packed__))
//{
//    graphic_data_struct_t grapic_data_struct;
//    uint8_t data[30];
//} ext_client_custom_character_t;

///* �����˼佻������ר��֡�ṹ */
//typedef struct __attribute__((__packed__))
//{
//    referee_data_header_t frame_header;  //֡ͷ
//    uint16_t cmd_id;  //������ ID
//    ext_student_interactive_header_data_t data_header;  //���ݶ�ͷ�ṹ
//    uint16_t frame_tail;  //֡β
//} frame_t;
//
//typedef struct __attribute__((__packed__))
//{
//    referee_data_header_t frame_header;  //֡ͷ
//    uint16_t cmd_id;  //������ ID
//    ext_student_interactive_header_data_t data_header;  //���ݶ�ͷ�ṹ
//    uint16_t frame_tail;  //֡β
//} interaction_data_frame_t;

/* �ͻ�����Ϣ */
typedef struct __attribute__((__packed__))
{
    uint8_t robot_id;
    uint16_t client_id;
} client_info_t;




/**
*  ��̨�ֿ�ͨ��ѡ�ֶ˴��ͼ������˷��͹̶����ݡ���Ӧ������Ϊ 0x0303
*  ����ʱ���ͣ����η��ͼ�����õ��� 0.5 �롣
* Byte 0-3��
*     Ŀ��λ�� x �����꣬��λ m��������Ŀ������� ID ʱ����ֵΪ 0
* Byte 4-7��
*     Ŀ��λ�� y �����꣬��λ m��������Ŀ������� ID ʱ����ֵΪ 0
* Byte 8��
*     ��̨�ְ��µļ��̰���ͨ�ü�ֵ���ް������£���Ϊ 0
* Byte 9��
*     �Է������� ID����������������ʱ����ֵΪ 0
* Byte 10-11��
*     ��Ϣ��Դ ID����Ϣ��Դ�� ID��ID ��Ӧ��ϵ�����¼
*/
typedef struct __attribute__((__packed__))
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
} map_command_t;

/**
*  ѡ�ֶ�С��ͼ���յ��״﷢�͵ĶԷ������˵��������ݡ� ��Ӧ������IDΪ��0x0305
 *  �� x��y �����߽�ʱ��ʾ�ڶ�Ӧ��Ե������ x��y ��Ϊ 0 ʱ����Ϊδ���ʹ˻��������ꡣ
 *  Ӣ�ۻ����� x λ�����꣬��λ��cm
 *  Ӣ�ۻ����� y λ�����꣬��λ��cm
 *  ���̻����� x λ�����꣬��λ��cm
 *  ���̻����� y λ�����꣬��λ��cm
 *  ���� 3 �Ż����� x λ�����꣬��λ��cm
 *  ���� 3 �Ż����� y λ�����꣬��λ��cm
 *  ���� 4 �Ż����� x λ�����꣬��λ��cm
 *  ���� 4 �Ż����� y λ�����꣬��λ��cm
 *  ���� 5 �Ż����� x λ�����꣬��λ��cm
 *  ���� 5 �Ż����� y λ�����꣬��λ��cm
 *  �ڱ������� x λ�����꣬��λ��cm
 *  �ڱ������� y λ�����꣬��λ��cm
*/
typedef struct __attribute__((__packed__))
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;

/**
*  ѡ�ֶ�С��ͼ���յ��ڱ������˻�ѡ����Զ����Ʒ�ʽ�Ļ�����ͨ��������·���͵ĶԷ������˵��������ݡ� ��Ӧ������IDΪ��0x0307
*     С��ͼ���½�Ϊ����ԭ�㣬ˮƽ����Ϊ X ����
*     ������ֱ����Ϊ Y ����������ʾλ�ý���
*     �ճ��سߴ���С��ͼ�ߴ�ȱ����ţ������߽�
*     ��λ�ý��ڱ߽紦��ʾ
* Byte 0��
*     1����Ŀ��㹥��
*     2����Ŀ������
*     3���ƶ���Ŀ���
* Byte 1-2��
*     ·����� x �����꣬��λ��dm
* Byte 3-4��
*     ·����� y �����꣬��λ��dm
* Byte 5-53��
*     ·���� x ���������飬��λ��dm
* Byte 54-102��
*     ·���� y ���������飬��λ��dm
* Byte 103-104��
*     ������ ID
*/
typedef struct __attribute__((__packed__))
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} map_data_t;

/**
*  ���������˿�ͨ��������·�򼺷�����ѡ�ֶ˷����Զ������Ϣ������Ϣ���ڼ���ѡ�ֶ��ض�λ����ʾ�� ��Ӧ������IDΪ��0x0308
* Byte 0-1��
*     ������ ID
* Byte 2-3��
*     ������ ID
* Byte 4-33��
*     �ַ�
*/
typedef struct __attribute__((__packed__))
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} custom_info_t;

/**
*  �����ֿ�ʹ���Զ��������ͨ��ͼ����·���Ӧ�Ļ����˷�������  ��Ӧ������IDΪ��0x0302
*/
typedef struct __attribute__((__packed__))
{
    uint8_t data[30];
} custom_robot_data_t;

/**
*  �����ֿ�ʹ���Զ��������ͨ��ͼ����·���Ӧ�Ļ����˷�������  ��Ӧ������IDΪ��0x0309
*/
typedef struct __attribute__((__packed__))
{
    uint8_t data[30];
} robot_custom_data_t;

/**
*  ͨ��ң�������͵ļ���ң�����ݽ�ͬ��ͨ��ͼ����·���͸���Ӧ�����ˡ�  ��Ӧ������IDΪ��0x0304
* Byte 0-1��
*     ��� x ���ƶ��ٶȣ���ֵ��ʶ�����ƶ�
* Byte 2-3��
*     ��� y ���ƶ��ٶȣ���ֵ��ʶ�����ƶ�
* Byte 4-5��
*     �������ƶ��ٶȣ���ֵ��ʶ������
* Byte 6��
*     �������Ƿ��£�0 Ϊδ���£�1 Ϊ����
* Byte 7��
*     ����Ҽ��Ƿ��£�0 Ϊδ���£�1 Ϊ����
* Byte 8-9��
*     ���̰�����Ϣ��ÿ�� bit ��Ӧһ��������0 Ϊδ���£�1 Ϊ���£�
*     ? bit 0��W ��
*     ? bit 1��S ��
*     ? bit 2��A ��
*     ? bit 3��D ��
*     ? bit 4��Shift ��
*     ? bit 5��Ctrl ��
*     ? bit 6��Q ��
*     ? bit 7��E ��
*     ? bit 8��R ��
*     ? bit 9��F ��
*     ? bit 10��G ��
*     ? bit 11��Z ��
*     ? bit 12��X ��
*     ? bit 13��C ��
*     ? bit 14��V ��
*     ? bit 15��B ��
* Byte 10-11��
*     ����λ
*/
typedef struct __attribute__((__packed__))
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} remote_control_t;

/**
*  ����·���ݣ������ֿ�ʹ���Զ��������ģ��������ѡ�ֶˡ� ��Ӧ������IDΪ��0x0306
* Byte 0-1��
*     ���̼�ֵ��
*     ? bit 0-7������ 1 ��ֵ
*     ? bit 8-15������ 2 ��ֵ
* Byte 2-3��
*     ? bit 0-11����� X ������λ��
*     ? bit 12-15��������״̬
* Byte 4-5��
*     ? bit 0-11����� Y ������λ��
*     ? bit 12-15������Ҽ�״̬
* Byte 6-7��
*     ����λ
*/
typedef struct __attribute__((__packed__))
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
} custom_client_data_t;

/**
 * @brief ����ϵͳ���ճ�ʼ��
 */
void referee_system_init();
/**
 * @brief ����ϵͳ��������֡���
 */
void referee_data_unpack(uint8_t *data, uint16_t len);
/**
 * @brief ����ϵͳ���ݸ��²�����
 */
void referee_data_save(uint8_t* referee_data_frame);

/* ------------------------------ referee����״̬���� ------------------------------ */
/**
 * @brief ��λ������״̬����,��referee����
 */
struct referee_fdb_msg
{
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    remote_control_t remote_control;
    custom_robot_data_t custom_robot_data;
    game_robot_HP_t game_robot_HP;
    sentry_info_t sentry_info;
};

#endif //CTRBOARD_H7_ALL_REFEREE_SYSTEM_H

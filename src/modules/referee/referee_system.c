//
// Created by ���ο� on 25-2-25.
//

#include "referee_system.h"
#include "crc8_crc16.h"
#include "string.h"


/* --------------------------------����ϵͳ���ھ�� ------------------------------- */
static referee_data_header_t referee_data_header;   //��������֡ͷ�ṹ��
static referee_data_t referee_data;   //��������֡ͷ�ṹ��
static unpack_data_t referee_unpack_obj;
static float float_values[7] = {0}; // �洢ת�����7��float���ĳɶ��д���


struct referee_fdb_msg referee_fdb;


/*!�ṹ��ʵ����*/
static game_status_t                           game_status;
static game_result_t                           game_result;
static game_robot_HP_t                         game_robot_HP;
static event_data_t                            event_data;
static referee_warning_t                       referee_warning;
static dart_info_t                             dart_info;
static robot_status_t                          robot_status;
static power_heat_data_t                       power_heat_data;
static robot_pos_t                             robot_pos;
static buff_t                                  buff;
static hurt_data_t                             hurt_data;
static shoot_data_t                            shoot_data;
static projectile_allowance_t                  projectile_allowance;
static rfid_status_t                           rfid_status;
static dart_client_cmd_t                       dart_client_cmd;
static ground_robot_position_t                 ground_robot_position;
static radar_mark_data_t                       radar_mark_data;
static sentry_info_t                           sentry_info;
static radar_info_t                            radar_info;
static robot_interaction_data_t                robot_interaction_data;
static map_command_t                           map_command;
static map_robot_data_t                        map_robot_data;
static map_data_t                              map_data;
static custom_info_t                           custom_info;
static custom_robot_data_t                     custom_robot_data;
static robot_custom_data_t                     robot_custom_data;
static remote_control_t                        remote_control;
static custom_client_data_t                    custom_client_data;

void referee_system_init()
{
    memset(&referee_data_header, 0, sizeof(referee_data_header_t));
    memset(&referee_data, 0, sizeof(referee_data_t));
    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));
    memset(&event_data, 0, sizeof(event_data_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));
    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));
    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&map_robot_data, 0, sizeof(map_robot_data_t ));
    memset(&map_data, 0, sizeof(map_data_t));
    memset(&custom_info, 0, sizeof(custom_info_t));
    memset(&custom_robot_data, 0, sizeof(custom_robot_data_t));
    memset(&robot_custom_data, 0, sizeof(robot_custom_data_t));
    memset(&remote_control, 0, sizeof(remote_control_t));
    memset(&custom_client_data, 0, sizeof(custom_client_data_t));

    memset(&referee_fdb, 0, sizeof(struct referee_fdb_msg));
}


/**
 * @brief ����ϵͳ���ݽ������
 */
void referee_data_unpack(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        switch (referee_unpack_obj.unpack_step)  //״̬ת����
        {
            case STEP_HEADER_SOF:      //����Ƕ�ȡ֡ͷSOF��״̬
            {
                if (byte == HEADER_SOF)       //�ж��Ƿ�ΪSOF
                {
                    referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;  //��������ã������������ȼ�1
                    referee_unpack_obj.unpack_step = STEP_DATA_SIZE_LOW;       //�ı�״̬���´��ó�����byte��ȥ��ͼ��Ӧ���ݳ��ȵĵͰ�λ
                } else {
                    referee_unpack_obj.index = 0;   //������ǣ����ٴ�fifo���ó���һ��byte����������ֱ��������һ��sof
                }
            }
                break;

            case STEP_DATA_SIZE_LOW:       //���Ŀǰ��״̬�Ƕ������ݳ��ȵĵͰ�λ
            {
                referee_unpack_obj.p_header->data_length = byte;           //�Ͱ�λֱ�ӷ���
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;   //�������
                referee_unpack_obj.unpack_step = STEP_DATA_SIZE_HIGH;          //ת��״̬
            }
                break;

            case STEP_DATA_SIZE_HIGH:  //���Ŀǰ��״̬ʱ�����ݳ��ȵĸ߰�λ
            {
                referee_unpack_obj.p_header->data_length |= (byte << 8);     //����data_len�ĸ߰�λ
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;  //�������
                if (referee_unpack_obj.p_header->data_length < REF_PROTOCOL_DATA_MAX_SIZE) {
                    referee_unpack_obj.unpack_step = STEP_FRAME_SEQ;        //ת��״̬����һ���ö������
                } else {
                    //������ݳ��Ȳ��Ϸ�������ͷ��ʼ��ȡ������֮ǰ��õ���������
                    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                    referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                    referee_unpack_obj.index = 0;
                }

            }
                break;

            case STEP_FRAME_SEQ: {
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;  //�������
                referee_unpack_obj.unpack_step = STEP_HEADER_CRC8;          //ת��״̬����һ��byte������CRC8
            }
                break;

            case STEP_HEADER_CRC8: {
                //�Ƚ���һbyte���ݷ��룬ʹ֡ͷ�ṹ�������Ա������Խ���CRCУ��
                referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
                //�����һbyte����֮�����ݳ�����һ��֡ͷ�ĳ��ȣ���ô�ͽ���CRCУ��
                if (referee_unpack_obj.index == REF_PROTOCOL_HEADER_SIZE) {
                    if (verify_CRC8_check_sum(referee_unpack_obj.protocol_packet, REF_PROTOCOL_HEADER_SIZE)) {
                        referee_unpack_obj.unpack_step = STEP_DATA_CRC16;   //���У��ͨ������״̬ת����ȥ��ȡ֡β
                    } else {
                        //���У�鲻ͨ�������ͷ��ʼ��֮ǰ��õ���������
                        memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                        referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                        referee_unpack_obj.index = 0;
                    }
                }
            }
                break;

            case STEP_DATA_CRC16: {
                //��֡ͷ��֡β�Ĺ����е�����һ�����
                if (referee_unpack_obj.index < (REF_HEADER_CRC_CMD_SIZE + referee_unpack_obj.p_header->data_length)) {
                    referee_unpack_obj.protocol_packet[referee_unpack_obj.index++] = byte;
                }
                //������ݶ�ȡ��dataĩβ����ת��״̬��׼����ʼ��һ֡�Ķ�ȡ
                if (referee_unpack_obj.index >= (REF_HEADER_CRC_CMD_SIZE + referee_unpack_obj.p_header->data_length)) {
                    //��������У��
                    if (verify_CRC16_check_sum(referee_unpack_obj.protocol_packet,
                                               REF_HEADER_CRC_CMD_SIZE + referee_unpack_obj.p_header->data_length)) {
                        referee_data_save(referee_unpack_obj.protocol_packet);
                    }
                    memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                    referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                    referee_unpack_obj.index = 0;
                }
            }
                break;

            default:
                memset(&referee_unpack_obj, 0, sizeof(unpack_data_t));
                referee_unpack_obj.unpack_step = STEP_HEADER_SOF;
                referee_unpack_obj.index = 0;
                break;
        }
    }
}

/**
 * @brief ����ϵͳ�������ݽ������
 * @param referee_data_frame: ���յ�����֡����
 */
void referee_data_save(uint8_t* frame)
{
    uint16_t cmd_id = 0;
    uint8_t index = 0;

    memcpy(&referee_data_header, frame, sizeof(referee_data_header_t));
    index += sizeof(referee_data_header_t);
    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATUS_CMD_ID:
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            break;
        case GAME_RESULT_CMD_ID:
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            break;
        case GAME_ROBOT_HP_CMD_ID:
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            memcpy(&(referee_fdb.game_robot_HP), frame + index, sizeof(game_robot_HP_t));
            break;
        case FIELD_EVENTS_CMD_ID:
            memcpy(&event_data, frame + index, sizeof(event_data_t));
            break;
        case REFEREE_WARNING_CMD_ID:
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            break;
        case DART_FIRE_CMD_ID :
            memcpy(&dart_info, frame + index, sizeof(dart_info_t));
            break;
        case ROBOT_STATUS_CMD_ID:
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            memcpy(&(referee_fdb.robot_status),&robot_status, sizeof(robot_status_t));
            break;
        case POWER_HEAT_DATA_CMD_ID:
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            break;
        case ROBOT_POS_CMD_ID:
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
            break;
        case BUFF_MUSK_CMD_ID:
            memcpy(&buff, frame + index, sizeof(buff_t));
            break;
        case ROBOT_HURT_CMD_ID:
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
            break;
        case SHOOT_DATA_CMD_ID:
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            break;
        case BULLET_REMAINING_CMD_ID:
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            break;
        case ROBOT_RFID_CMD_ID :
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
            break;
        case DART_DIRECTIONS_CMD_ID  :
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
            break;
        case ROBOT_LOCATION_CMD_ID :
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
            break;
        case RADAR_PROGRESS_CMD_ID :
            memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
            break;
        case SENTRY_AUTONOMY__CMD_ID :
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
            memcpy(&(referee_fdb.sentry_info), frame + index, sizeof(sentry_info_t));
            break;
        case RADAR_AUTONOMY_CMD_ID :
            memcpy(&radar_info, frame + index, sizeof(radar_info_t));
            break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
            break;
        case ARM_DATA_FROM_CONTROLLER_CMD_ID_2 :
            memcpy(&custom_robot_data, frame + index, sizeof(custom_robot_data_t));
            // ֱ��ת��ң������
            for (int i = 0; i < 7; i++) {
                uint8_t *byte_ptr = &custom_robot_data.data[i * 4];
                memcpy(&float_values[i], byte_ptr, sizeof(float));
            }
            break;
        case PLAYER_MINIMAP_CMD_ID :
            memcpy(&map_command, frame + index, sizeof(map_command_t));
            break;
        case KEYBOARD_MOUSE_CMD_ID :
            memcpy(&remote_control, frame + index, sizeof(remote_control_t));
            memcpy(&(referee_fdb.remote_control), &remote_control, sizeof(remote_control_t));
            break;
        case RADAR_MINIMAP_CMD_ID :
            memcpy(&map_robot_data, frame + index, sizeof(map_robot_data_t));
            break;
        case CUSTOMER_CONTROLLER_PLAYER_CMD_ID :
            memcpy(&custom_client_data, frame + index, sizeof(custom_client_data_t));
            break;
        case PLAYER_MINIMAP_SENTRY_CMD_ID :
            memcpy(&map_data, frame + index, sizeof(map_data_t));
            break;
        case PLAYER_MINIMAP_ROBOT_CMD_ID :
            memcpy(&custom_info, frame + index, sizeof(custom_info_t));
            break;
        case ARM_DATA_FROM_CONTROLLER_CMD_ID_9 :
            memcpy(&robot_custom_data, frame + index, sizeof(robot_custom_data_t));
            break;
        default:
            break;
    }
}

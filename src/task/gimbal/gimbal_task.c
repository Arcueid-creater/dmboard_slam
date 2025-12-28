//
// Created by Gleam on 25-8-22.
//

#include <stdlib.h>
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#define GIM_PITCH_MOTOR_NUM 1
#define GIM_YAW_MOTOR_NUM 3
#define GIM_MOTOR_NUM 3
/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */

// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chass_cmd;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gim_cmd;
MCN_DECLARE(gimbal_ins_topic);
static McnNode_t gimbal_ins_node;
static struct dm_imu_t gim_ins;

// 发布
MCN_DECLARE(gimbal_fdb);
struct gimbal_fdb_msg gimbal_fdb_data;

static void gimbal_pub_push(void);
static void gimbal_sub_init(void);
static void gimbal_sub_pull(void);


/* ------------------------------------------------- 电机控制相关 ------------------------------------------------------ */

static struct gimbal_controller_t{
    /* 基于imu数据闭环，主要用于手动模式 */
    pid_obj_t *pid_speed_imu;
    pid_obj_t *pid_angle_imu;
    /* 基于imu数据闭环，主要用于自动模式 */
    pid_obj_t *pid_speed_auto;
    pid_obj_t *pid_angle_auto;
}gim_controller[GIM_MOTOR_NUM];

motor_config_t gimbal_motor_config[GIM_MOTOR_NUM] = {
        {
                .motor_type = DM4310,
                .can_id = CAN_ID_GIMBAL_MOTOR,
                .rx_id = 0x10,
                .tx_id = 0x01,
                // .controller = &gim_controller[YAW],
        },
        {
                .motor_type = DM4310,   //英雄pitch轴改用丝杆结构，换用3508电机
                .can_id = CAN_ID_GIMBAL_MOTOR,
                .rx_id = 0x10,   //电机ID待定
                // .controller = &gim_controller[PITCH],
                .tx_id = 0x02,

        },
        {
            .motor_type = DM4310,
            .can_id = CAN_ID_GIMBAL_MOTOR,
            .rx_id = 0x10,
            .tx_id = 0x03,
            // .controller = &gim_controller[YAW],
        }
};


/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
/* gyro三轴：[0]为X，[1]为Y，[2]为Z */
#define X 0
#define Y 1
#define Z 2
static int16_t yaw_motor_relive, pitch_motor_relive;  // 电机相对于归中值的角度

static ramp_obj_t *yaw_ramp;//yaw 轴云台控制斜坡
static ramp_obj_t *pit_ramp;//pitch 轴云台控制斜坡

static dji_motor_object_t *gim_motor_pitch[GIM_PITCH_MOTOR_NUM];  // 底盘电机实例
static dm_motor_object_t *gim_motor_yaw[GIM_YAW_MOTOR_NUM];  // 底盘电机实例
static float gim_motor_ref[GIM_MOTOR_NUM]; // 电机控制期望值

static void gimbal_motor_init();
static int16_t motor_control_yaw(dji_motor_measure_t measure);
static int16_t motor_control_pitch(dji_motor_measure_t measure);
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);
static int auto_staus=1;
/*自瞄相对角传参反馈*/
auto_relative_angle_status_e auto_relative_angle_status=RELATIVE_ANGLE_TRANS;

/* ------------------------------------------------ 云台线程入口 ----------------------------------------------------- */
static float gim_dt;

void gimbal_task_init(void){
    gimbal_sub_init();
    gimbal_motor_init();
    // yaw_ramp = ramp_register(0, BACK_CENTER_TIME/GIMBAL_PERIOD);
    // pit_ramp = ramp_register(0, BACK_CENTER_TIME/GIMBAL_PERIOD);

}


static float gim_start;
static uint32_t init_start_time; // 云台初始化归中开始时间，避免长时间因为静态误差，卡在归中模式
static uint32_t init_dt; // 云台初始化归中进行时长
/* USER CODE END Header_ChassisTask_Entry */
void gimbal_control()
{

    gim_start = dwt_get_time_ms();
    // 云台本身相对于归中值的角度，取负
    yaw_motor_relive = -(int16_t)get_relative_pos(gim_motor_yaw[0]->measure.total_angle, CENTER_ECD_YAW) / 22.75f;
//     pitch_motor_relive = -(int16_t )get_relative_pos(gim_motor_pitch->measure.total_angle, CENTER_ECD_PITCH) / 22.75f;
    pitch_motor_relive = gim_ins.pitch;   //pitch轴改用丝杆结构，直接使用ins_data.pitch作为相对角度值

//        if((gim_cmd.ctrl_mode==GIMBAL_GYRO||GIMBAL_AUTO)&&gimbal_fdb_data.back_mode==BACK_IS_OK)
//        {
//        yaw_motor_relive = (rt_int16_t)(gim_cmd.yaw+gim_ins.yaw_total_angle);
//        }

    for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
    {
        dji_motor_enable(gim_motor_pitch[i]);
    }
    for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
    {
        dm_motor_enable(gim_motor_yaw[i]);
    }

    switch (gim_cmd.ctrl_mode)
    {
        case GIMBAL_RELAX:
            for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor_pitch[i]);
            }
            for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
            {
                dm_motor_relax(gim_motor_yaw[i]);
            }
            gimbal_fdb_data.back_mode = BACK_STEP;
            yaw_ramp->reset(yaw_ramp, 0, BACK_CENTER_TIME/GIMBAL_PERIOD);
            pit_ramp->reset(pit_ramp, 0, BACK_CENTER_TIME/GIMBAL_PERIOD);

            break;
        case GIMBAL_INIT:
            // TODO：加入斜坡算法，可以控制归中时间
            // TODO: 将编码器值转化为角度值
            // TODO: 优化归中逻辑，yaw轴选取最近的方向
            if(gim_cmd.last_mode != GIMBAL_INIT)
                init_start_time = dwt_get_time_ms();
            else
                init_dt = dwt_get_time_ms() - init_start_time;

            gim_motor_ref[YAW] = yaw_motor_relive * ( 1 - yaw_ramp->calc(yaw_ramp));
            gim_motor_ref[PITCH] = pitch_motor_relive* ( 1 - pit_ramp->calc(pit_ramp));

            //pitch轴改用丝杆结构，只能根据imu数据控制归中
            if(abs(gim_ins.pitch) <= (20 / 22.75f)
               && (abs(gim_motor_yaw[0]->measure.total_angle - CENTER_ECD_YAW) <= 20)
               // 若长时间陷于归中模式，可以适当放宽归中条件
               || ((abs(gim_ins.pitch) <= (200 / 22.75f))
                   && (abs(gim_motor_yaw[0]->measure.total_angle - CENTER_ECD_YAW) <= 200)
                   && (init_dt > INIT_TIMEOUT)))
            {
                gimbal_fdb_data.back_mode = BACK_IS_OK;
                gimbal_fdb_data.yaw_offset_angle_total = gim_ins.yaw_total_angle;/*云台抽风的原因，期望应该为总角度。抽风原因：不应该用ins_data.yaw*/
                gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
                gimbal_fdb_data.pit_offset_angle = gim_ins.pitch;
                auto_staus=1;
            }
            else
            {
                gimbal_fdb_data.back_mode = BACK_IS_OK;
            }
            break;
        case GIMBAL_GYRO:

            gim_motor_ref[YAW] = gim_cmd.yaw;
            gim_motor_ref[PITCH] = gim_cmd.pitch;
            // 底盘相对于云台归中值的角度，取负
            gimbal_fdb_data.yaw_relative_angle = -yaw_motor_relive;
//            gimbal_fdb_data.yaw_relative_angle = -(/*gim_ins.yaw_total_angle - */gimbal_fdb_data.yaw_offset_angle_total);
            gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
            gimbal_fdb_data.pit_offset_angle=gim_ins.pitch;
            if (auto_staus==0)
            {
                auto_staus=1;
                auto_relative_angle_status=RELATIVE_ANGLE_TRANS;
            }
            break;

            // TODO: add auto mode
        case GIMBAL_AUTO:
            /*gim_motor_ref[YAW] = gim_cmd.yaw_auto;*/
            if(auto_staus==1)
            {
                //gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
                auto_staus=0;
                auto_relative_angle_status=RELATIVE_ANGLE_TRANS;
            }
            gim_motor_ref[YAW] =gim_cmd.yaw;
            gim_motor_ref[PITCH] =gim_cmd.pitch;
            // 底盘相对于云台归中值的角度，取负
            gimbal_fdb_data.yaw_relative_angle = -yaw_motor_relive;
            break;

        default:
            for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor_pitch[i]);
            }
            for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
            {
                dm_motor_relax(gim_motor_yaw[i]);
            }
            break;
    }
    /* 用于调试监测线程调度使用 */
    gim_dt = dwt_get_time_ms() - gim_start;
    // vTaskDelay(1);
}
uint8_t data[8];
void gimbal_control_task(){
    /* 更新该线程所有的订阅者 */
    gimbal_sub_pull();
    // gimbal_control();
    /* 更新发布该线程的msg */
    data[0] = 110;
    data[1] = 55;
    CAN_send(&hfdcan2,0x110,data);
    gimbal_pub_push();

}
static float control_dt[4];
static float control_start[4];
static float dm_send_t[4];
float dm_obs[4];
#define DM_RATIO 1.0f
#define DM_OUTPUT_LIMIT  10.0f
static void motor_enable()
{
    dm_motor_enable_all();  // 所有电机进入 motor 模式




}
static float control_dt[4];
static float control_start[4];
static float dm_send_t[4]={0};
float dm_obs[4];
#define DM_RATIO 1.0f
#define DM_OUTPUT_LIMIT  10.0f

/*目前是以护栏较窄的一侧为正方向,从正方向向后看去,rigdm_front:id 2 ; rigdm_back:id 3;left_front:id 1;left_back:id 4*/
/* 1 号电机 */
static dm_motor_para_t dm_control_1(dm_motor_measure_t measure)
{
    control_dt[0] = dwt_get_time_us() - control_start[0];
    control_start[0] = dwt_get_time_us();
    static dm_motor_para_t set;

//    dm_send_t[0] = WBR_T_L[1] * DM_RATIO;
    // dm_send_t[0] = 0;
    LIMIT_MIN_MAX(dm_send_t[0], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[0] = dm_send_t[0] ;

#ifdef DM8009P_SET_ZERO
        dm_send_t[0] = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[0], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = dm_send_t[0]; // 正负没问题
    }
    return set;
}
/* 2 号电机 */
static dm_motor_para_t dm_control_2(dm_motor_measure_t measure)
{
    control_dt[1] = dwt_get_time_us() - control_start[1];
    control_start[1] = dwt_get_time_us();
    static dm_motor_para_t set;


//    dm_send_t[1] = -WBR_T_R[1] * DM_RATIO;
    // dm_send_t[1] = 0;
    LIMIT_MIN_MAX(dm_send_t[1], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[1] = dm_send_t[1] ;



#ifdef DM8009P_SET_ZERO
        dm_send_t[1] = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[1], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = dm_send_t[1]; // 正负没问题
    }
    return set;
}
/* 3 号电机 */
static dm_motor_para_t dm_control_3(dm_motor_measure_t measure)
{
    control_dt[2] = dwt_get_time_us() - control_start[2];
    control_start[2] = dwt_get_time_us();
    static dm_motor_para_t set;

//    dm_send_t[2] = -WBR_T_R[0] * DM_RATIO ;
    // dm_send_t[2] = 0;
    LIMIT_MIN_MAX(dm_send_t[2], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[2] = dm_send_t[2] ;


#ifdef DM8009P_SET_ZERO
        dm_send_t[2] = 0;
#endif

    LIMIT_MIN_MAX(dm_send_t[2], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = dm_send_t[2]; // 正负没问题
    }
    return set;
}
/* 4 号电机 */
static dm_motor_para_t dm_control_4(dm_motor_measure_t measure)
{
    control_dt[3] = dwt_get_time_us() - control_start[3];
    control_start[3] = dwt_get_time_us();
    static dm_motor_para_t set;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位

//    dm_send_t[3] = WBR_T_L[0] * DM_RATIO;
    // dm_send_t[3] = 0;
    LIMIT_MIN_MAX(dm_send_t[3], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    dm_obs[3] = dm_send_t[3];


#ifdef DM8009P_SET_ZERO
    dm_send_t[3] = 0;

#endif

    LIMIT_MIN_MAX(dm_send_t[3], -DM_OUTPUT_LIMIT, DM_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = dm_send_t[3]; // 正负没问题
    }
    return set;
}
/* 底盘每个电机对应的控制函数 */
static void *dm_control[4] =
        {
                dm_control_1,
                dm_control_2,
                dm_control_3,
                dm_control_4,
        };
/* ------------------------------------------------ 云台控制相关 ----------------------------------------------------- */
/**
 * @brief 注册云台电机及其控制器初始化
 */
static void gimbal_motor_init()
{
/* ----------------------------------- yaw ---------------------------------- */
    for (int i=0; i<3; i++)
    {
        gim_motor_yaw[i]=dm_motor_register(&gimbal_motor_config[i],dm_control[i]);
    }
    // dm_motor_enable_all();  // 所有电机进入 motor 模式
    motor_enable();
}
int16_t test_speed_yaw, test_speed_pitch=0;
static int16_t motor_control_yaw(dji_motor_measure_t measure){
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static int16_t send_data;        // 最终发送给电调的数据

    switch (gim_cmd.ctrl_mode)
    {
        // TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
        case GIMBAL_INIT:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_speed = gim_ins.gyro[Z];
            get_angle = -yaw_motor_relive;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            // // 将imu零飘清0，无奈之举，期待imu零飘问题的解决
            // if(get_speed < 0.5 && get_speed > -0.5)
            // {
            //     get_speed = 0;
            // }
            // if(get_angle < 0.5 && get_angle > -0.5)
            // {
            //     get_angle = 0;
            // }
            get_speed = gim_ins.gyro[Z];
            get_angle = gim_ins.yaw_total_angle - gimbal_fdb_data.yaw_offset_angle_total;

            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[YAW].pid_speed_auto;
            pid_angle = gim_controller[YAW].pid_angle_auto;
            get_speed = gim_ins.gyro[Z];
            get_angle = gim_ins.yaw_total_angle - gimbal_fdb_data.yaw_offset_angle_total;
            break;
        default:
            break;
    }
    /* 切换模式需要清空控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
        pid_clear(gim_controller[YAW].pid_angle_imu);
        pid_clear(gim_controller[YAW].pid_speed_imu);
        pid_clear(gim_controller[YAW].pid_angle_auto);
        pid_clear(gim_controller[YAW].pid_speed_auto);
    }


    if(gim_cmd.ctrl_mode == GIMBAL_INIT)  // 编码器闭环
    {
        /* 注意负号 */
        pid_angle->ITerm =0;
        pid_angle->Iout =0;
        pid_speed->ITerm =0;
        pid_speed->Iout =0;
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);  // 编码器增长方向与imu相反
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);     // 电机转动正方向与imu相反
    }
    else if(gim_cmd.ctrl_mode != GIMBAL_RELAX && gim_cmd.ctrl_mode != GIMBAL_INIT) /* imu闭环 */
    {
        /* 注意负号 */
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);
        // float feedforward = 500 * pid_out_angle; //+ 50 * filtered_accel  ; //简单估计前馈量
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);// + feedforward;      // 电机转动正方向与imu相反
    }

    return send_data;
}

static int16_t motor_control_pitch(dji_motor_measure_t measure){
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static int16_t send_data;        // 最终发送给电调的数据
    static uint32_t FEEDBACK_DWT_CNT;   //记录时间戳
    static float filtered_accel;

    switch (gim_cmd.ctrl_mode)
    {
        /* 根据云台模式，切换对应的控制器及观测量 */
        case GIMBAL_INIT:// TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            get_speed = gim_ins.gyro[Y];
            get_angle = gim_ins.pitch;  //pitch轴改用丝杆结构，直接使用ins_data.pitch作为相对角度值
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            get_speed = gim_ins.gyro[Y];
            get_angle = gim_ins.pitch;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[PITCH].pid_speed_auto;
            pid_angle = gim_controller[PITCH].pid_angle_auto;
            get_speed = gim_ins.gyro[Y];
            get_angle = gim_ins.pitch;
            break;
        default:
            break;
    }
    /* 切换模式需要清空  控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
        pid_clear(gim_controller[PITCH].pid_angle_imu);
        pid_clear(gim_controller[PITCH].pid_speed_imu);
        pid_clear(gim_controller[PITCH].pid_angle_auto);
        pid_clear(gim_controller[PITCH].pid_speed_auto);
    }

    // 对于英雄的pitch轴，由于采用丝杆结构，编码器数值无法作为归中位置的参考，故均采用imu闭环
    if(gim_cmd.ctrl_mode == GIMBAL_AUTO)  // 编码器闭环
    {
        /*串级pid的使用，角度环套在速度环上面*/
        /* 注意负号 */
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);

        float feedforward = 25 * pid_out_angle; //+ 50 * filtered_accel  ; //简单估计前馈量
        send_data = -pid_calculate(pid_speed, get_speed, pid_out_angle) - feedforward;     // 电机转动正方向与imu相反
    }
    else /* imu闭环 */
    {
        /* 限制云台俯仰角度 */
        VAL_LIMIT(gim_motor_ref[PITCH], PIT_ANGLE_MIN, PIT_ANGLE_MAX);

        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);

        float feedforward = 50 * pid_out_angle; //+ 50 * filtered_accel  ; //简单估计前馈量
        send_data = -pid_calculate(pid_speed, get_speed, pid_out_angle) - feedforward ;      // 电机转动正方向与imu相反
    }

    return send_data;
}

/**
 * @brief Get the relative pos object
 *
 * @param raw_ecd  实际的编码器值
 * @param center_offset 相对的参考编码器值
 * @return int16_t
 */
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4095){
        if (raw_ecd > center_offset - 4095)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8191 - center_offset;
    }
    else{
        if (raw_ecd > center_offset + 4095)
            tmp = raw_ecd - 8191 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}


/******************************************************消息订阅*************************************************************************/
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void gimbal_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(gimbal_fdb), &gimbal_fdb_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void gimbal_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_ins_node = mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void gimbal_sub_pull(void)
{
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
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gim_cmd);
    }

    if (mcn_poll(gimbal_ins_node))
    {
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
    }
}




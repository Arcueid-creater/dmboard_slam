
#include "cmd_task.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "robot.h"

/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(chassis_fdb);
static McnNode_t chassis_fdb_node;
static struct chassis_fdb_msg chassis_fdb;
// 发布
MCN_DECLARE(chassis_cmd);
static struct chassis_cmd_msg chassis_cmd_data;
MCN_DECLARE(gimbal_cmd);
static struct gimbal_cmd_msg gimbal_cmd_data;
MCN_DECLARE(shoot_cmd);
static struct shoot_cmd_msg shoot_cmd_data;
MCN_DECLARE(lifter_cmd_topic);
MCN_DECLARE(lifter_fdb_topic);
MCN_DECLARE(gimbal_fdb);
static struct gimbal_fdb_msg gimbal_fdb_data;
static McnNode_t gimbal_fdb_node;
static struct lifter_cmd_msg lifter_cmd;
static struct lifter_fdb_msg lifter_fdb;
static McnNode_t lifter_fdb_node;
static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);
static void LifterState_Ctrl();
ramp_obj_t *lifter_period = NULL;
/* --------------------------------------------------- 遥控器相关 ---------------------------------------------------- */
#ifdef BSP_USING_RC_DBUS
static rc_dbus_obj_t *rc_now, *rc_last;

#else
extern sbus_data_t sbus_data_fdb;
#endif
height_ref_t height_ref;
/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd(void);
static void ChassisState_Ctrl();
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式

/* -------------------------------- cmd 线程主体 -------------------------------- */
void cmd_task_init(void)
{
    lifter_period=ramp_register(0,LIFTER_PERIOD);
    cmd_sub_init();
    rc_now = dbus_rc_init();//rc_now接收的是rc_dbus_obj[2]数组的首地址，rc_last接收的是last的地址
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
}

void cmd_control_task(void)
{
    cmd_sub_pull();

    remote_to_cmd();

    cmd_pub_push();
}
static float jump_flag=0;
static float flag=0;
int spin_cnt=0;
static float jump_count=0;
/**
 * @brief 将遥控器数据转换为控制指令
 */

static void remote_to_cmd(void)
{
    /* 保存上一次数据 */
    // gimbal_cmd_data.last_mode = gimbal_cmd_data.ctrl_mode;
    // chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
    // shoot_cmd_data.last_mode=shoot_cmd_data.ctrl_mode;
    // lifter_cmd.last_mode=lifter_cmd.ctrl_mode;
    /* 保存上一次数据 */
    // gim_cmd.last_mode = gim_cmd.ctrl_mode;
    LifterState_Ctrl();
    ChassisState_Ctrl();
}
static void GimbalState_Ctrl()
{
    if (rc_now->sw2==RC_UP)
    {
        gimbal_cmd_data.ctrl_mode=GIMBAL_RELAX;
    }
    if (rc_now->sw2!=RC_UP)//所有状态机需要在使能模式下才能转变
    {
        if (rc_now->sw2==RC_MI||rc_now->sw2==RC_DN)//因为直接一下拨打RC_DN，是自瞄模式，直接开启自瞄模式，默认需要进行归中操作
        {
            if (rc_last->sw2==RC_UP||gimbal_cmd_data.last_mode==GIMBAL_RELAX)//如果没有完成归中操作，需要先进行归中
            {
                gimbal_cmd_data.ctrl_mode=GIMBAL_INIT;
            }//转换成初始化的控制模式之后，完成归中会自动转换为KEEP模式
            //所以不需要写上一个模式时是初始化，该做什么处理
        }
        if (rc_now->sw1==RC_DN&&gimbal_cmd_data.ctrl_mode==GIMBAL_GYRO)//处于LIFTER_HEIGHT_KEEP模式，说明之前归中任务完成，可以直接转换成小陀螺模式
        {
            gimbal_cmd_data.ctrl_mode=GIMBAL_AUTO;
        }

    }
    switch (gimbal_cmd_data.ctrl_mode)
    {
        case GIMBAL_RELAX:



            break;

        case GIMBAL_INIT:
            gimbal_cmd_data.gimbal_height=GIMBAL_MID_HEIGHT;

            break;

        case GIMBAL_GYRO:


            break;
        case GIMBAL_AUTO:

        case GIMBAL_NO_FOLLOW:

            break;

    }
}
static void ChassisState_Ctrl()
{
    chassis_cmd_data.last_mode=chassis_cmd_data.ctrl_mode;

    if (rc_now->sw2==RC_UP||rc_now->sw2==0)
    {
        chassis_cmd_data.ctrl_mode=CHASSIS_RELAX;
    }
    if (rc_now->sw2!=RC_UP&&rc_now->sw2!=0)//所有状态机需要在使能模式下才能转变
    {
        if (rc_now->sw2==RC_MI)
        {
            chassis_cmd_data.ctrl_mode=CHASSIS_NO_GIMBAL;//目前没有云台，
        }
        if (rc_now->sw1==RC_DN&&chassis_cmd_data.ctrl_mode==CHASSIS_NO_GIMBAL)//处于LIFTER_HEIGHT_KEEP模式，说明之前归中任务完成，可以直接转换成小陀螺模式
        {
            chassis_cmd_data.ctrl_mode=CHASSIS_SPIN;
        }
        if ((chassis_cmd_data.last_mode==CHASSIS_SPIN||chassis_cmd_data.ctrl_mode==CHASSIS_SPIN)&&rc_now->sw1!=RC_DN)//必须满足上一次时旋转模式，并且上一次的拨杆时在下方，
            //拨杆拨动，退出小陀螺模式，才能确保转换状态正常完成，并且需要进行一次归中
        {
            chassis_cmd_data.ctrl_mode=CHASSIS_RETURN;
        }
        if ( chassis_cmd_data.ctrl_mode==CHASSIS_RETURN)
        {

        }
    }
    switch (chassis_cmd_data.ctrl_mode)
    {
        case CHASSIS_RELAX:

            break;

        case CHASSIS_NO_GIMBAL:
            chassis_cmd_data.vx =  (float)rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
            chassis_cmd_data.vy =  (float)rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
            chassis_cmd_data.vw =  (float)rc_now->ch3 * CHASSIS_RC_MOVE_RATIO_R / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VR_SPEED + (float)rc_now->mouse.x * CHASSIS_PC_MOVE_RATIO_R;

            break;
        case CHASSIS_SPIN:
            chassis_cmd_data.vw=2;// * msg_cmd->robot_status.chassis_power_limit/55;/*!小陀螺转速，随着功率限制提升加快转速*/
            chassis_cmd_data.vx =  (float)rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VX_SPEED + km.vx * CHASSIS_PC_MOVE_RATIO_X;
            chassis_cmd_data.vy =  (float)rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_DBUS_MAX_VALUE * MAX_CHASSIS_VY_SPEED + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
                if(chassis_fdb.vw_ch < chassis_cmd_data.vw*0.85f) //当小陀螺被堵住时，自动退出小陀螺模式
                {
                    spin_cnt++;
                    if(spin_cnt>2000)
                    {
                        chassis_cmd_data.ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
                        spin_cnt=0;
                    }
                }
                else
                {
                    spin_cnt =0;
                }
            break;
        case CHASSIS_RETURN:
            if (chassis_fdb.spin_flag==0)
            {
                chassis_cmd_data.ctrl_mode=CHASSIS_NO_GIMBAL;
            }
            break;
    }

}
static void LifterState_Ctrl()
{
    lifter_cmd.last_mode=lifter_cmd.ctrl_mode;
        ////TODO 现在是开小陀螺的时候打开升降底盘，按操作手需求，是否开启自瞄时自动小陀螺
        if (rc_now->sw2==RC_UP||rc_now->sw2==0)
        {
            lifter_cmd.ctrl_mode=LIFTER_RELAX;
        }
        if (rc_now->sw2!=RC_UP&&rc_now->sw2!=0)//所有状态机需要在使能模式下才能转变
        {
            if (rc_now->sw2==RC_MI||rc_now->sw2==RC_DN)//因为直接一下拨打RC_DN，是自瞄模式，直接开启自瞄模式，默认需要进行归中操作
            {
                if (rc_last->sw2==RC_UP||lifter_cmd.last_mode==LIFTER_RELAX)//如果没有完成归中操作，需要先进行归中
                {
                    lifter_cmd.ctrl_mode=LIFTER_HEIGHT_INIT;
                }//转换成初始化的控制模式之后，完成归中会自动转换为KEEP模式
                //所以不需要写上一个模式时是初始化，该做什么处理
            }
            if (rc_now->sw1==RC_DN&&lifter_cmd.ctrl_mode==LIFTER_HEIGHT_KEEP)//处于LIFTER_HEIGHT_KEEP模式，说明之前归中任务完成，可以直接转换成小陀螺模式
            {
                lifter_cmd.ctrl_mode=LIFTER_SPIN;
            }
            if ((lifter_cmd.last_mode==LIFTER_SPIN||lifter_cmd.ctrl_mode==LIFTER_SPIN)&&rc_now->sw1!=RC_DN)//必须满足上一次时旋转模式，并且上一次的拨杆时在下方，
                //拨杆拨动，退出小陀螺模式，才能确保转换状态正常完成，并且需要进行一次归中
            {
                lifter_cmd.ctrl_mode=LIFTER_HEIGHT_INIT;
            }
            //如果按下拨杆时的模式压根不是小陀螺模式，则不需要处理
            //极限情况，从失能模式进入归中模式，同时开启小陀螺，因为不是KEEP模式，底盘不会进入小陀螺模式，只有等升降底盘完成归中才会开启小陀螺
        }

        switch (lifter_cmd.ctrl_mode)
        {
            case LIFTER_RELAX:
                //LIFTER_RELAX失能状态已经在任务那边处理
                // lifter_cmd.target_angle=-10.0f;
                // lifter_cmd.dTarget_angle=0.0f;
                lifter_cmd.enable=0;
                lifter_cmd.Kd=0.03f;
                break;
                ////TODO查看资料如何获取得到此时在空中，关节电机为落地做好准备
                ////TODO，麦轮位于坡上时，如何避免PITCH角度，使LQR输出变化。
                ////TODO 位于坡上时，四个腿的前馈支持力发生变化，如何更改，保持稳定
                ////TODO 飞坡模式按键控制，并且需要合适的状态机处理
            case LIFTER_FLY:

                break;
                ////TODO 按键改变高度，虽然我觉得没什么意义
            case LIFTER_HEIGHT_CHANGE:


                break;
            case LIFTER_SPIN:
            {
                float temp = lifter_period->calc(lifter_period);
                lifter_cmd.target_angle = 12.500f + 22.500f * arm_sin_f32(2.000000f * PI * temp-0.46055399f);
                // height_ref.dheight = LIFTER_AMPLITUDE * 0.50000f * PI * arm_cos_f32(2.000000f * PI * temp); //上一步已经递增过了
                //角度和速度周期性变化
                //关于速度的公式，求导得到，高度的形式是为了使用ms为单位控制2*PI*t/4000 t转换为s，则2*PI*t/4， w=2*PI/4
                if (lifter_period->count == lifter_period->scale)
                {
                    lifter_period->reset(lifter_period, 0,LIFTER_PERIOD);
                }
                break;
            }
            case LIFTER_HEIGHT_KEEP://这个状态机是控制的是一般情况下的底盘高度，如果进入修改高度的模式，KEEP模式下的高度会继承
                //KEEP模式不需要做特别处理，这个函数结束时，会给lifter_cmd.height赋值
                lifter_cmd.Kd=0.01f;
                lifter_cmd.target_angle+=((float)rc_now->ch4)*0.0001f;
                VAL_LIMIT(lifter_cmd.target_angle,-10.0f,35.0f);
                break;
            case LIFTER_HEIGHT_INIT:
                lifter_cmd.enable=1;
                lifter_cmd.target_angle=0.0f;
                lifter_cmd.dTarget_angle=0.0f;
                lifter_cmd.Kd=0.03f;
                // height_ref.dheight=0.0f;
                if (lifter_fdb.back_mode==LIFTER_BACK_IS_OK)
                {
                 lifter_cmd.ctrl_mode=LIFTER_HEIGHT_KEEP;
                }
                break;
        }


        // lifter_cmd.height=height_ref.height;
        // lifter_cmd.d_height=height_ref.dheight;
}
/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_cmd_data);
    mcn_publish(MCN_HUB(shoot_cmd), &shoot_cmd_data);
    mcn_publish(MCN_HUB(lifter_cmd_topic),&lifter_cmd);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);
    lifter_fdb_node=mcn_subscribe(MCN_HUB(lifter_fdb_topic), NULL, NULL);
    gimbal_fdb_node=mcn_subscribe(MCN_HUB(gimbal_fdb), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    if (mcn_poll(chassis_fdb_node))
    {
        mcn_copy(MCN_HUB(chassis_fdb), chassis_fdb_node, &chassis_fdb);
    }
    if (mcn_poll(lifter_fdb_node))
    {
        mcn_copy(MCN_HUB(lifter_fdb_topic),lifter_fdb_node,&lifter_fdb);
    }
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb),gimbal_fdb_node,&gimbal_fdb_data);
    }
}

int chassis_board_rx_callback(uint32_t id, uint8_t *data)
{
    switch (id)
    {
    case CAN_RPY_TX:
        chassis_cmd_data.offset_angle = *(float*)&data[0];
        return 0;
        break;
    
    default:
        return -1;
        break;
    }
}

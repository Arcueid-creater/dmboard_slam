//
// Created by w on 25-12-12.
//
#include "rm_task.h"
#include "rm_module.h"
#include "uart_receive.h"
osThreadId insTaskHandle;
osThreadId chassisTaskHandle;
osThreadId cmdTaskHandle;
osThreadId motorTaskHandle;
osThreadId transTaskHandle;
osThreadId refereeTaskHandle;
osThreadId shootTaskHandle;
osThreadId lifterTaskHandle ;
osThreadId gimbalTaskHandle;
osThreadId uartTaskHandle;
osThreadId cloudTaskHandle;
static float motor_dt;
static float chassis_dt;
static float cmd_dt;
static float trans_dt;
static float shoot_dt;
static float ins_dt;
static float gimbal_dt;
static float lifter_dt;
static char cmd_dt_str[16], motor_dt_str[16], chassis_dt_str[16],
            trans_dt_str[16], shoot_dt_str[16], gimbal_start_str[16], ins_dt_str[16];


/**
 * @brief ��ʼ������������,���г������е������������ʼ��
 *
 */
 void OS_task_init()
{
    osThreadDef(instask, ins_task_entry, osPriorityNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // Ϊ��̬�������ýϸ����ȼ�,ȷ����1khz��Ƶ��ִ��

    osThreadDef(motortask, motor_task_entry,osPriorityAboveNormal , 0, 2048);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(chassistask, chassis_task_entry, osPriorityNormal, 0, 2048);
    chassisTaskHandle = osThreadCreate(osThread(chassistask), NULL);

    osThreadDef(cmdtask, cmd_task_entry, osPriorityNormal, 0, 1024);
    cmdTaskHandle = osThreadCreate(osThread(cmdtask), NULL);

    osThreadDef(gimbaltask, gimbal_task_entry, osPriorityNormal, 0, 1024);
    gimbalTaskHandle = osThreadCreate(osThread(gimbaltask), NULL);
    // //
    osThreadDef(transtask, trans_task_entry, osPriorityNormal, 0, 4.96);
    transTaskHandle = osThreadCreate(osThread(transtask), NULL);

    osThreadDef(shoottask, shoot_task_entry, osPriorityNormal, 0, 1024);
    shootTaskHandle = osThreadCreate(osThread(shoottask), NULL);

     // osThreadDef(lifter_task, LifterTask_entry, osPriorityAboveNormal, 0, 2048);
     // lifterTaskHandle = osThreadCreate(osThread(lifter_task), NULL);

     osThreadDef(uart_task,USARTRecTask_Entry,osPriorityAboveNormal,0,2048);
     uartTaskHandle=osThreadCreate(osThread(uart_task), NULL);

     osThreadDef(cloudtask, cloud_task_entry, osPriorityNormal, 0, 2048);
     cloudTaskHandle = osThreadCreate(osThread(cloudtask), NULL);

//    osThreadDef(refereetask, referee_task_entry, osPriorityNormal, 0, 1024);
//    refereeTaskHandle = osThreadCreate(osThread(refereetask), NULL);

}

int lifter_tim=0;
__attribute__((noreturn))  void LifterTask_entry (void const *argument)
 {
     float start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] Chassis Task Start\r\n");

     uint32_t lifter_waste_time = osKernelSysTick();
     for (;;)
     {
         /* ------------------------------ ���Լ���̵߳��� ------------------------------ */
         lifter_dt = dwt_get_time_ms() - start;
         start = dwt_get_time_ms();
         lifter_tim=lifter_dt;
         if (chassis_dt > 5.5) {
             Float2Str(chassis_dt_str,chassis_dt);
             LOGERROR("[freeRTOS] Chassis Task is being DELAY! dt = %s\r\n", &chassis_dt_str);
         }

         /* ------------------------------ ���Լ���̵߳��� ------------------------------ */

         lifter_control_task();

         vTaskDelayUntil(&lifter_waste_time, 1);
     }
 }
int motor_tim=0;
__attribute__((noreturn))  void motor_task_entry(void const *argument)
{
    float motor_start = dwt_get_time_ms();
    LOGINFO("[freeRTOS] Motor Task Start\r\n");

    uint32_t motor_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        motor_dt = dwt_get_time_ms() - motor_start;
        motor_start = dwt_get_time_ms();
        motor_tim=motor_dt;
        if (motor_dt > 1.5) {
            Float2Str(motor_dt_str,motor_dt);
            LOGERROR("[freeRTOS] Motor Task is being DELAY! dt = %s\r\n", &motor_dt_str);
        }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

        motor_control_task();
        //
        // dji_motor_control();
        vTaskDelayUntil(&motor_wake_time, 1);
    }
}
int chassis_tim=0;
__attribute__((noreturn))  void chassis_task_entry(void const *argument)
{
    float chassis_start = dwt_get_time_ms();
    LOGINFO("[freeRTOS] Chassis Task Start\r\n");

    uint32_t chassis_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        chassis_dt = dwt_get_time_ms() - chassis_start;
        chassis_start = dwt_get_time_ms();
        chassis_tim=chassis_dt;
        if (chassis_dt > 5.5) {
            Float2Str(chassis_dt_str,chassis_dt);
            LOGERROR("[freeRTOS] Chassis Task is being DELAY! dt = %s\r\n", &chassis_dt_str);
        }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

        chassis_control_task();
        // dji_motor_control();
        vTaskDelayUntil(&chassis_wake_time, 1);  // ƽ�ⲽ����Ҫ1khz
    }
}
int cmd_tim=0;
 __attribute__((noreturn))  void cmd_task_entry(void const *argument)
 {
     float cmd_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] Cmd Task Start\r\n");
     uint32_t robot_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
         cmd_dt = dwt_get_time_ms() - cmd_start;
         cmd_start = dwt_get_time_ms();
         cmd_tim=cmd_dt;
         if (cmd_dt > 1.5) {
             Float2Str(cmd_dt_str,cmd_dt);
             LOGERROR("[freeRTOS] Cmd Task is being DELAY! dt = %s\r\n", &cmd_dt_str);
         }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

         cmd_control_task();

         vTaskDelayUntil(&robot_wake_time, 1);
     }
 }
int trans_tim=0;
 __attribute__((noreturn))  void trans_task_entry(void const *argument)
{
    float trans_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] Trans Task Start\r\n");
    uint32_t trans_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
        trans_dt = dwt_get_time_ms() - trans_start;
        trans_start = dwt_get_time_ms();
        trans_tim=trans_dt;
        if (trans_dt > 1.5) {
            Float2Str(trans_dt_str,trans_dt);
            LOGERROR("[freeRTOS] Trans Task is being DELAY! dt = %s\r\n", &trans_dt_str);
        }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

        trans_control_task();

        vTaskDelayUntil(&trans_wake_time, 1);
    }
}
int gimbal_tim=0;
 __attribute__((noreturn))  void gimbal_task_entry(void const *argument)
 {
     float gimbal_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] gimbal Task Start\r\n");
     uint32_t gimbal_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
         gimbal_dt = dwt_get_time_ms() - gimbal_start;
         gimbal_start = dwt_get_time_ms();
         gimbal_tim=gimbal_dt;
         if (gimbal_dt > 1.5) {
             Float2Str(gimbal_start_str,gimbal_dt);
             LOGERROR("[freeRTOS] Gimbal Task is being DELAY! dt = %s\r\n", &gimbal_start_str);
         }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

         gimbal_control_task();

         vTaskDelayUntil(&gimbal_wake_time, 1);
     }
 }
int shoot_tim=0;
 __attribute__((noreturn))  void shoot_task_entry(void const *argument)
 {
     float shoot_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] shoot Task Start\r\n");
     uint32_t shoot_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
         shoot_dt = dwt_get_time_ms() - shoot_start;
         shoot_start = dwt_get_time_ms();
         shoot_tim=shoot_dt;
         if (shoot_dt > 1.5) {
             Float2Str(shoot_dt_str,shoot_dt);
             LOGERROR("[freeRTOS] shoot Task is being DELAY! dt = %s\r\n", &shoot_dt_str);
         }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */

         shoot_control_task();

         vTaskDelayUntil(&shoot_wake_time, 1);
     }
 }
int ins_tim=0;
 __attribute__((noreturn))  void ins_task_entry(void const *argument)
 {
     float ins_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] ins Task Start\r\n");
     uint32_t ins_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
         ins_dt = dwt_get_time_ms() - ins_start;
         ins_start = dwt_get_time_ms();
         ins_tim=ins_dt;
         if (ins_dt > 1.5) {
             Float2Str(ins_dt_str,ins_dt);
             LOGERROR("[freeRTOS] ins Task is being DELAY! dt = %s\r\n", &ins_dt_str);
         }

/* ------------------------------ ���Լ���̵߳��� ------------------------------ */
         ins_control_task();

         vTaskDelayUntil(&ins_wake_time, 1);
     }
 }

//__attribute__((noreturn)) void referee_task_entry(void const *argument)
//{
//    /* USER CODE BEGIN RefereeTask */
//    static float referee_start;
//    static uint32_t referee_dwt = 0;
//    static float dt = 0;
//    static uint32_t count = 0;
//
//    // referee_UI_task_init();
//
//    dt = dwt_get_delta(&referee_dwt);
//    referee_start = dwt_get_time_ms();
//
//    uint32_t referee_wake_time = osKernelSysTick();
//    PrintLog("[freeRTOS] Ins Task Start\n");
//    /* Infinite loop */
//    for(;;)
//    {
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//        referee_dt = dwt_get_time_ms() - referee_start;
//        referee_start = dwt_get_time_ms();
///* ------------------------------ ���Լ���̵߳��� ------------------------------ */
//
//        dt = dwt_get_delta(&referee_dwt);
//
//
//        vTaskDelayUntil(&referee_wake_time, 1);
//    }
//}

__attribute__((noreturn)) void cloud_task_entry(void const *argument)
{
    float cloud_start = dwt_get_time_ms();
    LOGINFO("[freeRTOS] Cloud Task Start\r\n");

    uint32_t cloud_wake_time = osKernelSysTick();
    for (;;)
    {
        cloud_control_task();
        vTaskDelayUntil(&cloud_wake_time, 100);
    }
}

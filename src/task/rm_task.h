
 /**
 * @file rm_task.h
 * @brief  注意该文件应只用于任务初始化,只能被robot.c包含
 * @date 2023-12-28
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-28      ChuShicheng     first version
 */
#ifndef _RM_TASK_H
#define _RM_TASK_H

#include "ins_task.h"
#include "motor_task.h"
#include "cmd_task.h"
#include "chassis_task.h"
#include "trans_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "bsp_log.h"
#include "drv_dwt.h"
#include "lifter_task.h"
/* ---------------------------------- 线程相关 ---------------------------------- */

void ins_task_entry(void const *argument);
void motor_task_entry(void const *argument);
void chassis_task_entry(void const *argument);
void cmd_task_entry(void const *argument);
void trans_task_entry(void const *argument);
void gimbal_task_entry(void const *argument);
void shoot_task_entry(void const *argument);
//void referee_task_entry(void const *argument);
void LifterTask_entry(void const * argument);
void OS_task_init();
#endif /* _RM_TASK_H */

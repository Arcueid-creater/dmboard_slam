//
// Created by w on 25-12-1.
//

#ifndef LQR_CONTROL_H
#define LQR_CONTROL_H
#define LQR_NUM_MAX 5
#define LQR_STATE_DIM_MAX 10
#define LQR_CONTROL_DIM_MAX 8
#include <stdint.h>
#include <string.h>
#include "arm_math.h"
#include "cmsis_os.h"
typedef enum
{
    LQR_K_FIXED = 0,
    LQR_K_POLYNOMIAL,//多项式拟合得到
}lqr_k_type_e;

typedef struct
{
    uint8_t state_dim;//状态向量的维度
    uint8_t control_dim;//输入向量的维度
    lqr_k_type_e k_type;//
    float *K_data;


    float *state;
    float *state_ref;
    float *state_err;

    arm_matrix_instance_f32 K_matrix;
    float *lqr_output;          // 控制输出 u (m维)
    float *feedforward;     // 前馈控制 u_ff (m维)

    float *output_max;     // 控制上限 (m维)
    float *output_min;     // 控制下限 (m维)
    float* (*control)();  //返回的数组必须是静态或全局数组，不能是局部数组
    uint8_t enable;
    float *temp_buffer;
}lqr_object_t;

typedef struct
{
    uint8_t state_dim;
    uint8_t control_dim;
    lqr_k_type_e k_type;
    float *K_data;
    float *output_max;
    float *output_min;
    float *feedforward;
}lqr_config_t;

/**
 *
 * @param config 初始化设置
 * @param update_control 更新K矩阵的函数，计算LQR时，会自动调用一次
 * @return 返回lqr实例指针
 */
lqr_object_t *lqr_register(lqr_config_t *config,void *update_control );

/**
 *
 * @param object lqr实例指针
 * @param state 当前的状态向量
 * @param state_ref 期望的状态向量
 * @return 返回LQR输出
 */
float* lqr_update(lqr_object_t *object,float*state,float *state_ref );
void lqr_Enable(lqr_object_t *object);
void lqr_SetLimits(lqr_object_t *object,float *max_control,float*min_control);
void lqr_SetFeedforward(lqr_object_t *object, const float *feedforward);
#endif //LQR_CONTROL_H

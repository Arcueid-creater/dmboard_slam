//
// Created by w on 25-11-6.
//

#ifndef LIFTER_TASK_H
#define LIFTER_TASK_H

#include "stdint.h"
#include "cmsis_os.h"

// ========== 多项式拟合参数 ==========
#define POLY_ORDER 4  // 4阶多项式，求导后为3阶

// 足端位置多项式系数 (从MATLAB拟合获得，需替换)
// x_foot = px[0] + px[1]*theta + px[2]*theta^2 + px[3]*theta^3 + px[4]*theta^4
// y_foot = py[0] + py[1]*theta + py[2]*theta^2 + py[3]*theta^3 + py[4]*theta^4
typedef struct {
    float px[POLY_ORDER + 1];   // X坐标多项式系数
    float py[POLY_ORDER + 1];   // Y坐标多项式系数
    float dpx[POLY_ORDER];      // X坐标导数系数 (阶数-1)
    float dpy[POLY_ORDER];      // Y坐标导数系数 (阶数-1)
} PolyCoeff_t;
//=========================================cmd状态机设置期望高度和速度

// ========== 腿部机构参数 ==========
typedef struct {
    // 连杆长度 (mm)
    float L1;  // 主动连杆长度
    float L2;
    float L3;
    float L4;
    float L_small;    // 小连杆长度
    float L3_offset;  // 小连杆在L3上的偏移
    
    // 多项式系数
    PolyCoeff_t poly;
} LegMechanism_t;
// void lifter_motor_init(void);
void lifter_motor_init(void);
// ========== 底盘状态 ==========
typedef struct {
    // 位置状态
    float h;      // 车身高度 (m)
    float roll;    // Roll角 (rad)
    float pitch ;  // Pitch角 (rad)
    
    // 速度状态
    float dh;     // 竖直速度 (m/s)
    float droll ;   // Roll角速度 (rad/s)
    float dpitch ; // Pitch角速度 (rad/s)
    
    // 期望状态
    float h_ref;
    float roll_ref;
    float pitch_ref ;
    float dh_ref;
    float droll_ref ;
    float dpitch_ref ;
    float x_ref;
} LIFTERState_t;

// ========== 四条腿控制数据 ==========
typedef struct {
    // 关节电机角度 (deg)
    float joint_angle[4];  // [前左, 前右, 后左, 后右]
    float ref_joint_angle[4];
    // 足端位置 (mm)
    float foot_pos_x[4];
    float foot_pos_y[4];
    
    // 虚拟足端力 (N)
    float foot_force_x[4];
    float foot_force_y[4];
    
    // 关节电机力矩 (N·m)
    float joint_torque[4];
} LegControl_t;

// ========== 升降底盘控制器 ==========
// 状态向量: x = [h, phi, theta, dh, dphi, dtheta]^T (6维)
// 控制输入: u = [F1, F2, F3, F4]^T (4维，四条腿的虚拟足端力)
#define LIFTER_STATE_DIM 6
#define LIFTER_CONTROL_DIM 4

typedef struct {
    LIFTERState_t state;     // 底盘状态
    LegControl_t leg;         // 腿部控制
    LegMechanism_t mechanism; // 机构参数
    
    // LQR控制器（算法层）
    void *lqr_controller;     // lqr_controller_t指针
    
    // 卡尔曼滤波器（高度-速度估计）
    void *height_kf;          // KalmanFilter_t指针
    
    // 系统参数
    float mass;        // 车身质量 (kg)
    float I_xx;        // Roll转动惯量 (kg·m²)
    float I_yy;        // Pitch转动惯量 (kg·m²)
    float gravity;     // 重力加速度 (m/s²)
    float L_x;         // 前后腿距离 (m)
    float L_y;         // 左右腿距离 (m)
    
    // 控制限制
    float max_force;   // 最大足端力 (N)
    float min_force;   // 最小足端力 (N)
    float max_torque;  // 最大关节力矩 (N·m)
    
    uint8_t enable;    // 使能标志
} LifterController_t;

// ========== 函数声明 ==========

// 任务入口


// 初始化函数
static void Lifter_Init(LifterController_t *lifter);

// 多项式相关函数
float Poly_Eval(const float *coeff, int order, float x);
void Poly_Derivative(const float *coeff, int order, float *deriv_coeff);
float Poly_Eval_Derivative(const float *coeff, int order, float x);

// 正逆运动学
void FK_FootPosition(const LegMechanism_t *mech, float theta_deg, float *x, float *y);
void Jacobian_Compute(const LegMechanism_t *mech, float theta_deg, float *J11, float *J21);
void LifterInit(void);
// 力矩转换
void Force_To_Torque(const LegMechanism_t *mech, int leg_idx, 
                     float Fx, float Fy, float *torque);
void Torque_To_Force(const LegMechanism_t *mech, int leg_idx,
                     float torque, float *Fx, float *Fy);
void lifter_control_task(void);
// LQR控制器
void LQR_Control(LifterController_t *lifter);

// 状态更新
void Lifter_UpdateState(LifterController_t *lifter);

// 使能控制
void Lifter_Enable(void);
void Lifter_Disable(void);

// 参考值设置
void Lifter_SetHeight(float height_m);
void Lifter_SetVelocity(float velocity_m_s);

// IMU重力补偿和状态估计
float IMU_RemoveGravity_Z(float accel_imu, float roll, float pitch, float gravity);
float Estimate_Height_From_Legs(LifterController_t *lifter);

// 卡尔曼滤波器（高度-速度估计）
void Height_KF_Init(LifterController_t *lifter);
void Height_KF_Update(LifterController_t *lifter, float h_measured);

#endif //LIFTER_TASK_H

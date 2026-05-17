# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此仓库中工作提供指引。

## 项目概述

Robomaster 串联腿步兵（平衡步兵）嵌入式控制固件，运行于 **STM32H723VGTX**（Cortex-M7，单精度 FPU）。使用 **FreeRTOS**（通过 CMSIS-OS 封装）。

## 构建

项目由 **STM32CubeMX** 生成（`CtrBoard-H7_ALL.ioc`）。构建使用 **CMake + ARM GCC** 交叉编译工具链，构建目录为 `cmake-build-debug/`。

```bash
# 在项目根目录下执行，使用 ninja（推荐）：
cd cmake-build-debug && ninja

# 或使用 make：
cd cmake-build-debug && make -j$(nproc)
```

工具链路径（来自 CMakeCache.txt）：`D:/qq/wenjian/ARM_GCC/ARM_GCC/ARM_GCC/arm/bin/`。构建产物：`.elf`、`.hex`、`.bin`、`.map`，均在 `cmake-build-debug/` 下。

项目根目录的 CMakeLists.txt 是自动生成的，不要手动修改。

## 架构

### 分层结构

```
src/
├── algorithm/     # 数学算法库：PID、卡尔曼、MahonyAHRS、四元数EKF、LQR、Ramp、滤波器
├── modules/       # 硬件驱动（CAN、PWM、传感器、电机、遥控器、裁判系统、IPC、LED）
├── task/          # 应用层 RTOS 任务
├── rm_config.h    # 项目全部配置宏（电机选型、PID参数、底盘尺寸等）
├── robot.h        # 全部 IPC 消息结构体定义 + 共享枚举
├── robot.c        # 机器人初始化：MCN 话题注册 → 各任务初始化 → RTOS 任务创建
└── rm_task.h/c    # RTOS 任务入口函数和任务创建（OS_task_init）
```

### RTOS 任务（来自 `src/task/rm_task.c`）

每个任务以 1 kHz 频率运行（`vTaskDelayUntil(&wake_time, 1)`）。优先级从高到低：

| 任务 | 优先级 | 所在目录 |
|------|--------|----------|
| Motor | AboveNormal | `src/task/motor/` |
| Lifter | AboveNormal | `src/task/Lifter/` |
| UART | AboveNormal | `src/task/uart/` |
| INS | Normal（较大栈空间） | `src/task/ins/` |
| Chassis | Normal | `src/task/chassis/` |
| Cmd | Normal | `src/task/cmd/` |
| Gimbal | Normal | `src/task/gimbal/` |
| Trans | Normal | `src/task/trans/` |
| Shoot | Normal | `src/task/shoot/` |

### 线程间通信：uMCN

发布-订阅 IPC 模型（`src/modules/ipc/`）。话题在 `robot.c` 中通过 `MCN_DEFINE()` 定义，在 `mcn_topic_init()` 中注册。每个任务订阅输入话题，发布输出话题。

关键话题（在 `robot.c` 和 `robot.h` 中定义）：
- `chassis_cmd` / `chassis_fdb` — 底盘速度指令和反馈
- `gimbal_cmd` / `gimbal_fdb_topic` — 云台角度指令和反馈
- `shoot_cmd` / `shoot_fdb` — 射击/摩擦轮/拨弹指令和反馈
- `ins_topic` — IMU 姿态数据（陀螺仪、加速度计、roll/pitch/yaw）
- `trans_fdb` / `transmission_fdb_topic` — 上位机自瞄数据
- `lifter_cmd_topic` / `lifter_fdb_topic` — 升降腿高度指令和反馈
- `gimbal_ins_topic` — 云台端 DM IMU 数据

### 任务编写模板

每个任务遵循以下模式（参考 `src/task/README.md`）：

```c
// 在 .c 文件中：
MCN_DECLARE(input_topic);
static McnNode_t input_node;
static struct input_msg input_data;    // 订阅者数据不加 _data 后缀

static void xxx_sub_init(void) {
    input_node = mcn_subscribe(MCN_HUB(input_topic), NULL, NULL);
}
static void xxx_sub_pull(void) {
    if (mcn_poll(input_node))
        mcn_copy(MCN_HUB(input_topic), input_node, &input_data);
}
static void xxx_pub_push(void) {
    mcn_publish(MCN_HUB(output_topic), &output_data);  // 发布者数据加 _data 后缀
}

void xxx_task_init(void) { xxx_sub_init(); }
void xxx_control_task(void) {
    xxx_sub_pull();
    // ... 处理逻辑 ...
    xxx_pub_push();
}
```

### 电机子系统

通过 `rm_config.h` 中的宏选择电机类型：
- `BSP_USING_DJI_MOTOR` — DJI 3508/6020/2006 电机，CAN 总线
- `BSP_USING_DM_MOTOR` — DM J8009P 电机，CAN 总线（用作关节电机）
- LK 电机、HT 电机、UNITREE 电机（485 总线）

`motor_task`（`src/task/motor/`）以固定 1 kHz 频率运行全部电机控制。电机驱动代码在 `src/modules/motor/` 下。

### CAN 总线布局（`rm_config.h`）
- `CAN_CHASSIS_MOTOR` (hfdcan1) — 底盘关节电机
- `CAN_GIMBAL` / `CAN_WHEEL_MOTOR` (hfdcan2) — 云台电机和轮毂电机
- 与上位机（视觉/自瞄）的板间 CAN 通信，扩展 ID：`0x340`、`0x345` 等

### 底盘

配置为麦轮底盘（`BSP_CHASSIS_MECANUM_MODE`）+ 平衡腿模式（`WHEEL_LEG_INFANTRY`、`BSP_CHASSIS_LEG_MODE`）。使用 LQR 控制器保持平衡（`src/algorithm/LQR/`），腿长跟踪采用 VMC/WBR 算法（`src/modules/leg_vmc/`、`src/modules/leg_wbr/`）。

### 传感器
- **BMI088**（加速度计 + 陀螺仪）：SPI 连接，用作底盘 IMU（`src/modules/BMI088/`）
- **DM IMU**：云台端辅助 IMU，用于云台姿态解算（`src/modules/dm_imu/`）
- 校准：`BSP_BMI088_CALI` 宏使能校准流程

### 遥控器
- `BSP_USING_DBUS` — DJI DBUS 协议（默认启用）
- `BSP_USING_SBUS` — 备选方案，已注释
- `BSP_USING_RC_DBUS` — 使能遥控器输入解析
- 同时也支持 USB 键鼠输入（`src/modules/rc/keyboard/`）

### 核心配置文件：`src/rm_config.h`

包含全部项目级常量：各控制回路的 PID 参数（底盘速度环、云台 yaw/pitch 的 IMU/自瞄回路、摩擦轮、拨弹电机）、底盘尺寸（轮距、轮半径）、最大速度限制、升降机构限位、CAN ID 分配、功能宏开关。

## 编码约定

- `robot_init()` 在 RTOS 调度器启动前调用一次（在 `main.c` 中）
- `robot_init()` 执行期间禁止使用中断和阻塞延时，如必须延时请用 `dwt`
- 应用层代码包含 `rm_module.h` 和 `rm_task.h` 即可（已聚合全部模块和任务头文件）
- 话题名称全局不可重复
- 命名约定：订阅者数据变量不加后缀，发布者数据变量加 `_data` 后缀

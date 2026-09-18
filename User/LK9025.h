#ifndef LK9025_H
#define LK9025_H

#include "main.h"
#include "canio.h"

#include <stdint.h>
#include <stdbool.h>


/* ============================================================
 * 1. MF9025v2 CAN 基础参数
 * ============================================================ */

/* 单电机 CAN ID = 0x140 + Motor ID */
#define LK9025_CAN_BASE_ID                       0x140U

/* 电机 ID 范围 */
#define LK9025_MIN_MOTOR_ID                      1U
#define LK9025_MAX_MOTOR_ID                      32U


/* ============================================================
 * 2. MF9025v2 CAN 命令
 * ============================================================ */

/* 关闭电机 */
#define LK9025_CMD_OFF                           0x80U

/* 停止电机 */
#define LK9025_CMD_STOP                          0x81U

/* 运行电机 */
#define LK9025_CMD_RUN                           0x88U

/* 读取电机状态 2 */
#define LK9025_CMD_READ_STATUS2                  0x9CU

/* 转矩闭环 */
#define LK9025_CMD_TORQUE                        0xA1U

/* 速度闭环 */
#define LK9025_CMD_SPEED                         0xA2U

/* 多圈位置闭环 */
#define LK9025_CMD_POSITION                      0xA3U

/* 多圈位置闭环 + 最大速度限制 */
#define LK9025_CMD_POSITION_SPEED                0xA4U

/* 单圈位置闭环 */
#define LK9025_CMD_SINGLE_POSITION               0xA5U

/* 单圈位置闭环 + 最大速度限制 */
#define LK9025_CMD_SINGLE_POSITION_SPEED         0xA6U

/* 增量位置闭环 */
#define LK9025_CMD_INCREMENT_POSITION            0xA7U


/* ============================================================
 * 3. 本项目的软件限制
 *
 * 这些不是电机硬件极限。
 * 是我们程序主动限制的目标范围。
 *
 * 后续根据靶车机械结构再修改。
 * ============================================================ */

/* 最大目标转速，单位 rpm */
#define LK9025_SPEED_COMMAND_LIMIT_RPM           100.0f

/* 位置模式默认最大速度，单位 degree/s */
#define LK9025_POSITION_MAX_SPEED_DPS            360U

/* 超过该时间没有收到反馈，认为设备离线 */
#define LK9025_FEEDBACK_TIMEOUT_MS               100U


/* ============================================================
 * 4. 编码器参数
 *
 * MF9025v2 内部是 18 bit 编码器。
 *
 * 普通 CAN 状态反馈只返回高 16 bit，
 * 所以软件收到的数据范围按 0~65535 处理。
 * ============================================================ */

#define LK9025_ENCODER_RESOLUTION                65536L
#define LK9025_ENCODER_HALF_RANGE                32768L


/* ============================================================
 * 5. 控制模式
 * ============================================================ */

typedef enum
{
    /* 保护模式，不允许运动 */
    LK9025_MODE_PROTECT = 0,

    /* 速度闭环 */
    LK9025_MODE_SPEED,

    /* 位置闭环 */
    LK9025_MODE_POSITION

} LK9025_ControlMode_t;


/* ============================================================
 * 6. MF9025v2 电机对象
 *
 * 每一台 MF9025v2 都创建一个 LK9025_t 对象。
 *
 * 例如：
 *
 * LK9025_t motors[2];
 *
 * motors[0] -> ID 1 -> CAN 0x141
 * motors[1] -> ID 2 -> CAN 0x142
 * ============================================================ */

typedef struct
{
    /* --------------------------------------------------------
     * CAN 通信
     * -------------------------------------------------------- */

    /* 当前电机挂在哪条 CAN Bus */
    CANIO_Bus_t *bus;

    /* 电机 ID，范围 1~32 */
    uint8_t motor_id;

    /* 实际 CAN ID = 0x140 + motor_id */
    uint32_t can_id;


    /* --------------------------------------------------------
     * 控制模式
     * -------------------------------------------------------- */

    /* 上层希望进入的模式 */
    LK9025_ControlMode_t command_mode;

    /* 当前软件实际采用的模式 */
    LK9025_ControlMode_t active_mode;


    /* --------------------------------------------------------
     * 控制命令
     * -------------------------------------------------------- */

    /* 目标转速，单位 rpm */
    float command_speed_rpm;

    /* 目标多圈角度，单位 degree */
    float command_angle_deg;

    /* 位置模式最大速度，单位 degree/s */
    uint16_t position_max_speed_dps;


    /* --------------------------------------------------------
     * CAN 反馈
     * -------------------------------------------------------- */

    /* 电机温度，单位 °C */
    int8_t temperature;

    /* MF 电机转矩电流原始值 */
    int16_t iq_raw;

    /* 电机实际转速，协议单位 degree/s */
    int16_t speed_dps;

    /* CAN 返回的 16 bit 编码器位置 */
    uint16_t encoder;


    /* --------------------------------------------------------
     * 连续角度计算
     * -------------------------------------------------------- */

    /* 是否已经至少收到过一帧有效反馈 */
    uint8_t has_feedback;

    /* 上一帧编码器值 */
    uint16_t last_encoder;

    /* 累计编码器 count */
    int32_t total_encoder_counts;

    /* 相对于程序启动位置的连续角度 */
    float relative_angle_deg;


    /* --------------------------------------------------------
     * 通信状态
     * -------------------------------------------------------- */

    /* 上一次有效反馈的时间 */
    uint32_t last_feedback_tick;

} LK9025_t;


/* ============================================================
 * 7. 遥测结构体
 *
 * 用于 DebugTask / printf / 上位机显示。
 * ============================================================ */

typedef struct
{
    LK9025_ControlMode_t mode;

    float target_speed_rpm;

    float actual_speed_rpm;

    float target_angle_deg;

    float relative_angle_deg;

    float iq_amp;

    int8_t temperature;

    uint8_t feedback_online;

} LK9025_Telemetry_t;


/* ============================================================
 * 8. 对外 API
 * ============================================================ */


/*
 * 初始化一台 MF9025v2。
 *
 * 初始化时自动向 CANIO 注册：
 *
 * CAN ID
 * ↓
 * LK9025_RxCallback
 * ↓
 * motor 对象
 */
bool LK9025_Init(
    LK9025_t *motor,
    CANIO_Bus_t *bus,
    uint8_t motor_id
);


/* 设置控制模式 */
void LK9025_SetMode(
    LK9025_t *motor,
    LK9025_ControlMode_t mode
);


/* 设置目标转速，单位 rpm */
void LK9025_SetTargetSpeed(
    LK9025_t *motor,
    float target_speed_rpm
);


/* 设置多圈位置目标，单位 degree */
void LK9025_SetTargetAngle(
    LK9025_t *motor,
    float target_angle_deg
);


/* 设置位置模式最大转速，单位 degree/s */
void LK9025_SetPositionMaxSpeed(
    LK9025_t *motor,
    uint16_t max_speed_dps
);


/* 发送 0x88 */
HAL_StatusTypeDef LK9025_Run(
    LK9025_t *motor
);


/* 发送 0x81 */
HAL_StatusTypeDef LK9025_Stop(
    LK9025_t *motor
);


/* 发送 0x80 */
HAL_StatusTypeDef LK9025_Off(
    LK9025_t *motor
);


/* 主动读取状态 2：0x9C */
HAL_StatusTypeDef LK9025_ReadStatus2(
    LK9025_t *motor
);


/*
 * 根据当前 mode 和 command，
 * 向 MF9025v2 发送实际控制命令。
 *
 * RobotTask 可以固定 500 Hz 调用。
 */
HAL_StatusTypeDef LK9025_UpdateControl(
    LK9025_t *motor,
    uint32_t now_ms
);


/* 获取一份状态快照 */
void LK9025_GetTelemetry(
    const LK9025_t *motor,
    uint32_t now_ms,
    LK9025_Telemetry_t *telemetry
);


#endif /* LK9025_H */
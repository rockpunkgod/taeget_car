#include "LK9025.h"

#include <math.h>
#include <stddef.h>
#include <string.h>


/* ============================================================
 * 1. 浮点数限幅
 *
 * 例如：
 *
 * value = 150
 * limit = 100
 *
 * 返回 100。
 *
 * static：
 * 这个函数只允许 lk9025.c 内部使用。
 * ============================================================ */

static float LK9025_Clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}


/* ============================================================
 * 2. 从小端数据读取 int16_t
 *
 * LK 协议：
 *
 * data[0] = 低字节
 * data[1] = 高字节
 *
 * 例如：
 *
 * 34 12
 *
 * 得到：
 *
 * 0x1234
 * ============================================================ */

static int16_t LK9025_ReadInt16LE(const uint8_t *data)
{
    return (int16_t)(
        (uint16_t)data[0]
        |
        ((uint16_t)data[1] << 8U)
    );
}


/* ============================================================
 * 3. 从小端数据读取 uint16_t
 * ============================================================ */

static uint16_t LK9025_ReadUInt16LE(const uint8_t *data)
{
    return (uint16_t)(
        (uint16_t)data[0]
        |
        ((uint16_t)data[1] << 8U)
    );
}


/* ============================================================
 * 4. 把 uint16_t 写成小端格式
 *
 * value = 0x1234
 *
 * 得到：
 *
 * data[0] = 0x34
 * data[1] = 0x12
 * ============================================================ */

static void LK9025_WriteUInt16LE(
    uint8_t *data,
    uint16_t value)
{
    data[0] =
        (uint8_t)(value & 0xFFU);

    data[1] =
        (uint8_t)(value >> 8U);
}


/* ============================================================
 * 5. 把 int32_t 写成 4 Byte 小端格式
 *
 * 例如：
 *
 * value = 0x12345678
 *
 * data：
 *
 * 78 56 34 12
 * ============================================================ */

static void LK9025_WriteInt32LE(
    uint8_t *data,
    int32_t value)
{
    uint32_t raw =
        (uint32_t)value;

    data[0] =
        (uint8_t)(
            raw & 0xFFU
        );

    data[1] =
        (uint8_t)(
            (raw >> 8U) & 0xFFU
        );

    data[2] =
        (uint8_t)(
            (raw >> 16U) & 0xFFU
        );

    data[3] =
        (uint8_t)(
            (raw >> 24U) & 0xFFU
        );
}


/* ============================================================
 * 6. CANIO RX Callback
 *
 * CANIO 收到当前电机 CAN ID 后会调用这里。
 *
 * CANIO 不知道这是 MF9025。
 *
 * user_data 里面保存的是 LK9025_t *。
 * ============================================================ */

static void LK9025_RxCallback(
    const CANIO_Frame_t *frame,
    void *user_data)
{
    LK9025_t *motor =
        (LK9025_t *)user_data;

    uint16_t encoder;

    int16_t speed_dps;

    int16_t iq_raw;

    int32_t encoder_delta;


    /* 防御性空指针检查 */
    if ((motor == NULL)
        || (frame == NULL))
    {
        return;
    }


    /* 必须是当前电机的 8 Byte 报文 */
    if ((frame->id != motor->can_id)
        || (frame->dlc != 8U))
    {
        return;
    }


    /*
     * 以下命令返回的状态区均可按：
     *
     * DATA[1]   temperature
     * DATA[2:3] iq
     * DATA[4:5] speed
     * DATA[6:7] encoder
     *
     * 进行解析。
     */
    switch (frame->data[0])
    {
        case LK9025_CMD_TORQUE:

        case LK9025_CMD_SPEED:

        case LK9025_CMD_POSITION:

        case LK9025_CMD_POSITION_SPEED:

        case LK9025_CMD_SINGLE_POSITION:

        case LK9025_CMD_SINGLE_POSITION_SPEED:

        case LK9025_CMD_INCREMENT_POSITION:

        case LK9025_CMD_READ_STATUS2:

            break;

        default:

            return;
    }


    /* DATA[2:3]：Iq */
    iq_raw =
        LK9025_ReadInt16LE(
            &frame->data[2]
        );


    /* DATA[4:5]：实际速度，单位 1 dps/LSB */
    speed_dps =
        LK9025_ReadInt16LE(
            &frame->data[4]
        );


    /* DATA[6:7]：编码器 */
    encoder =
        LK9025_ReadUInt16LE(
            &frame->data[6]
        );


    /* --------------------------------------------------------
     * 第一次反馈
     * -------------------------------------------------------- */

    if (motor->has_feedback == 0U)
    {
        /* 第一次当前位置作为相对角度零点 */
        motor->last_encoder =
            encoder;

        motor->total_encoder_counts =
            0;

        motor->relative_angle_deg =
            0.0f;
    }


    /* --------------------------------------------------------
     * 后续反馈
     * -------------------------------------------------------- */

    else
    {
        /*
         * 计算当前编码器值与上一帧的差。
         */
        encoder_delta =
            (int32_t)encoder
            -
            (int32_t)motor->last_encoder;


        /*
         * 处理：
         *
         * 65535 → 0
         *
         * 的正方向跨零。
         */
        if (encoder_delta
            > LK9025_ENCODER_HALF_RANGE)
        {
            encoder_delta -=
                LK9025_ENCODER_RESOLUTION;
        }


        /*
         * 处理：
         *
         * 0 → 65535
         *
         * 的反方向跨零。
         */
        else if (
            encoder_delta
            < -LK9025_ENCODER_HALF_RANGE)
        {
            encoder_delta +=
                LK9025_ENCODER_RESOLUTION;
        }


        /*
         * 累计编码器变化量。
         */
        motor->total_encoder_counts +=
            encoder_delta;


        /*
         * 转换成连续角度。
         *
         * 65536 count ≈ 360°。
         */
        motor->relative_angle_deg =
            (
                (float)motor->total_encoder_counts
                * 360.0f
            )
            /
            (float)LK9025_ENCODER_RESOLUTION;


        /*
         * 保存当前编码器，
         * 给下一帧使用。
         */
        motor->last_encoder =
            encoder;
    }


    /* --------------------------------------------------------
     * 更新反馈状态
     * -------------------------------------------------------- */

    motor->temperature =
        (int8_t)frame->data[1];

    motor->iq_raw =
        iq_raw;

    motor->speed_dps =
        speed_dps;

    motor->encoder =
        encoder;


    /*
     * CANIO 已经记录了接收时间。
     */
    motor->last_feedback_tick =
        frame->timestamp_ms;


    /*
     * 确保上面的反馈字段写完以后，
     * 再发布 has_feedback。
     */
    __DMB();

    motor->has_feedback =
        1U;
}


/* ============================================================
 * 7. 发送简单控制命令
 *
 * 用于：
 *
 * 0x80 OFF
 * 0x81 STOP
 * 0x88 RUN
 * 0x9C READ STATUS2
 * ============================================================ */

static HAL_StatusTypeDef LK9025_SendSimpleCommand(
    LK9025_t *motor,
    uint8_t command)
{
    uint8_t data[8] =
    {
        0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U
    };


    if ((motor == NULL)
        || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }


    /* 第一字节为命令字 */
    data[0] =
        command;


    /*
     * 使用新的 CANIO API：
     *
     * bus
     * id
     * data[8]
     */
    if (CANIO_Send(
            motor->bus,
            motor->can_id,
            data))
    {
        return HAL_OK;
    }


    return HAL_ERROR;
}


/* ============================================================
 * 8. 发送速度闭环命令 0xA2
 *
 * 输入：
 *
 * rpm
 *
 * 协议需要：
 *
 * int32_t speedControl
 * 0.01 degree/s / LSB
 * ============================================================ */

static HAL_StatusTypeDef LK9025_SendSpeedCommand(
    LK9025_t *motor,
    float speed_rpm)
{
    uint8_t data[8] =
    {
        0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U
    };

    float speed_dps;

    int32_t speed_control;


    if ((motor == NULL)
        || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }


    /*
     * rpm → degree/s
     *
     * 1 rpm =
     * 360° / 60s =
     * 6 degree/s
     */
    speed_dps =
        speed_rpm * 6.0f;


    /*
     * 协议单位：
     *
     * 0.01 dps / LSB
     *
     * 所以乘 100。
     */
    speed_control =
        (int32_t)(
            speed_dps * 100.0f
        );


    /* Command */
    data[0] =
        LK9025_CMD_SPEED;


    /*
     * DATA[4:7]
     *
     * int32_t
     * little endian
     */
    LK9025_WriteInt32LE(
        &data[4],
        speed_control
    );


    if (CANIO_Send(
            motor->bus,
            motor->can_id,
            data))
    {
        return HAL_OK;
    }


    return HAL_ERROR;
}


/* ============================================================
 * 9. 发送带限速的多圈位置命令 0xA4
 *
 * DATA[2:3]
 *     maxSpeed
 *     uint16_t
 *     1 dps/LSB
 *
 * DATA[4:7]
 *     angleControl
 *     int32_t
 *     0.01 degree/LSB
 * ============================================================ */

static HAL_StatusTypeDef LK9025_SendPositionCommand(
    LK9025_t *motor,
    float target_angle_deg,
    uint16_t max_speed_dps)
{
    uint8_t data[8] =
    {
        0U, 0U, 0U, 0U,
        0U, 0U, 0U, 0U
    };

    int32_t angle_control;


    if ((motor == NULL)
        || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }


    /*
     * degree → 0.01 degree / LSB
     */
    angle_control =
        (int32_t)(
            target_angle_deg
            * 100.0f
        );


    /* Command */
    data[0] =
        LK9025_CMD_POSITION_SPEED;


    /*
     * DATA[2:3]
     *
     * 最大速度。
     */
    LK9025_WriteUInt16LE(
        &data[2],
        max_speed_dps
    );


    /*
     * DATA[4:7]
     *
     * 多圈位置目标。
     */
    LK9025_WriteInt32LE(
        &data[4],
        angle_control
    );


    if (CANIO_Send(
            motor->bus,
            motor->can_id,
            data))
    {
        return HAL_OK;
    }


    return HAL_ERROR;
}


/* ============================================================
 * 10. 初始化 MF9025v2
 * ============================================================ */

bool LK9025_Init(
    LK9025_t *motor,
    CANIO_Bus_t *bus,
    uint8_t motor_id)
{
    /* 参数检查 */
    if ((motor == NULL)
        || (bus == NULL))
    {
        return false;
    }


    /* Motor ID 检查 */
    if ((motor_id < LK9025_MIN_MOTOR_ID)
        || (motor_id > LK9025_MAX_MOTOR_ID))
    {
        return false;
    }


    /*
     * 清空整个电机对象。
     */
    memset(
        motor,
        0,
        sizeof(*motor)
    );


    /* 保存 CAN Bus */
    motor->bus =
        bus;


    /* 保存 Motor ID */
    motor->motor_id =
        motor_id;


    /*
     * 单电机 CAN ID：
     *
     * 0x140 + Motor ID
     */
    motor->can_id =
        LK9025_CAN_BASE_ID
        +
        (uint32_t)motor_id;


    /* 默认保护模式 */
    motor->command_mode =
        LK9025_MODE_PROTECT;

    motor->active_mode =
        LK9025_MODE_PROTECT;


    /* 初始命令 */
    motor->command_speed_rpm =
        0.0f;

    motor->command_angle_deg =
        0.0f;


    /* 位置模式默认最大速度 */
    motor->position_max_speed_dps =
        LK9025_POSITION_MAX_SPEED_DPS;


    /*
     * 向 CANIO 注册。
     *
     * 例如 Motor ID = 1：
     *
     * 0x141
     * ↓
     * LK9025_RxCallback
     * ↓
     * motor
     */
    if (!CANIO_Register(
            bus,
            motor->can_id,
            LK9025_RxCallback,
            motor))
    {
        return false;
    }


    return true;
}


/* ============================================================
 * 11. 设置控制模式
 * ============================================================ */

void LK9025_SetMode(
    LK9025_t *motor,
    LK9025_ControlMode_t mode)
{
    uint32_t primask;


    if ((motor == NULL)
        || (mode < LK9025_MODE_PROTECT)
        || (mode > LK9025_MODE_POSITION))
    {
        return;
    }


    /*
     * 保存当前中断状态。
     */
    primask =
        __get_PRIMASK();


    /*
     * 进入极短临界区。
     */
    __disable_irq();


    motor->command_mode =
        mode;


    __DMB();


    /*
     * 只有进入函数前中断是开启的，
     * 才重新开启。
     */
    if (primask == 0U)
    {
        __enable_irq();
    }
}


/* ============================================================
 * 12. 设置目标速度
 * ============================================================ */

void LK9025_SetTargetSpeed(
    LK9025_t *motor,
    float target_speed_rpm)
{
    uint32_t primask;


    if ((motor == NULL)
        || (!isfinite(target_speed_rpm)))
    {
        return;
    }


    /*
     * 软件安全限幅。
     */
    target_speed_rpm =
        LK9025_Clamp(
            target_speed_rpm,
            LK9025_SPEED_COMMAND_LIMIT_RPM
        );


    primask =
        __get_PRIMASK();


    __disable_irq();


    motor->command_speed_rpm =
        target_speed_rpm;


    __DMB();


    if (primask == 0U)
    {
        __enable_irq();
    }
}


/* ============================================================
 * 13. 设置目标角度
 * ============================================================ */

void LK9025_SetTargetAngle(
    LK9025_t *motor,
    float target_angle_deg)
{
    uint32_t primask;


    if ((motor == NULL)
        || (!isfinite(target_angle_deg)))
    {
        return;
    }


    primask =
        __get_PRIMASK();


    __disable_irq();


    /*
     * A4 是多圈位置，
     * 因此不做 -180~180 wrap。
     *
     * 可以给：
     *
     * 360°
     * 720°
     * -360°
     * 等。
     */
    motor->command_angle_deg =
        target_angle_deg;


    __DMB();


    if (primask == 0U)
    {
        __enable_irq();
    }
}


/* ============================================================
 * 14. 设置位置模式最大速度
 * ============================================================ */

void LK9025_SetPositionMaxSpeed(
    LK9025_t *motor,
    uint16_t max_speed_dps)
{
    uint32_t primask;


    if (motor == NULL)
    {
        return;
    }


    primask =
        __get_PRIMASK();


    __disable_irq();


    motor->position_max_speed_dps =
        max_speed_dps;


    __DMB();


    if (primask == 0U)
    {
        __enable_irq();
    }
}


/* ============================================================
 * 15. Motor Run
 * ============================================================ */

HAL_StatusTypeDef LK9025_Run(
    LK9025_t *motor)
{
    return LK9025_SendSimpleCommand(
        motor,
        LK9025_CMD_RUN
    );
}


/* ============================================================
 * 16. Motor Stop
 * ============================================================ */

HAL_StatusTypeDef LK9025_Stop(
    LK9025_t *motor)
{
    return LK9025_SendSimpleCommand(
        motor,
        LK9025_CMD_STOP
    );
}


/* ============================================================
 * 17. Motor Off
 * ============================================================ */

HAL_StatusTypeDef LK9025_Off(
    LK9025_t *motor)
{
    return LK9025_SendSimpleCommand(
        motor,
        LK9025_CMD_OFF
    );
}


/* ============================================================
 * 18. 主动读取状态 2
 * ============================================================ */

HAL_StatusTypeDef LK9025_ReadStatus2(
    LK9025_t *motor)
{
    return LK9025_SendSimpleCommand(
        motor,
        LK9025_CMD_READ_STATUS2
    );
}


/* ============================================================
 * 19. 更新控制
 *
 * 建议：
 *
 * RobotTask 固定 500 Hz 调用。
 *
 * 这里不运行 STM32 速度 PID。
 *
 * SPEED：
 *     发送 A2
 *
 * POSITION：
 *     发送 A4
 *
 * PROTECT：
 *     Stop
 * ============================================================ */

HAL_StatusTypeDef LK9025_UpdateControl(
    LK9025_t *motor,
    uint32_t now_ms)
{
    uint32_t primask;

    uint8_t has_feedback;

    uint32_t last_feedback_tick;

    LK9025_ControlMode_t command_mode;

    LK9025_ControlMode_t active_mode;

    float command_speed_rpm;

    float command_angle_deg;

    uint16_t max_speed_dps;

    HAL_StatusTypeDef status;


    if ((motor == NULL)
        || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }


    /*
     * --------------------------------------------------------
     * 取一致性快照
     * --------------------------------------------------------
     */

    primask =
        __get_PRIMASK();


    __disable_irq();


    has_feedback =
        motor->has_feedback;

    last_feedback_tick =
        motor->last_feedback_tick;

    command_mode =
        motor->command_mode;

    active_mode =
        motor->active_mode;

    command_speed_rpm =
        motor->command_speed_rpm;

    command_angle_deg =
        motor->command_angle_deg;

    max_speed_dps =
        motor->position_max_speed_dps;


    if (primask == 0U)
    {
        __enable_irq();
    }


    /*
     * --------------------------------------------------------
     * 如果以前已经收到过反馈，
     * 但现在反馈超时，则进入保护。
     *
     * 注意：
     *
     * 第一次启动时 has_feedback == 0，
     * 仍然允许发送第一条控制命令。
     *
     * 否则会出现：
     *
     * 没反馈 → 不发命令
     * 不发命令 → 电机不回复
     * 永远没有反馈
     *
     * 的死锁。
     * --------------------------------------------------------
     */

    if ((has_feedback != 0U)
        &&
        (
            (uint32_t)(
                now_ms
                -
                last_feedback_tick
            )
            >
            LK9025_FEEDBACK_TIMEOUT_MS
        ))
    {
        motor->active_mode =
            LK9025_MODE_PROTECT;

        return LK9025_Stop(
            motor
        );
    }


    /*
     * --------------------------------------------------------
     * 模式切换
     * --------------------------------------------------------
     */

    if (command_mode != active_mode)
    {
        /*
         * 切入保护模式：
         *
         * 只在模式变化时发送一次 Stop。
         */
        if (command_mode
            == LK9025_MODE_PROTECT)
        {
            status =
                LK9025_Stop(
                    motor
                );

            if (status == HAL_OK)
            {
                motor->active_mode =
                    LK9025_MODE_PROTECT;
            }

            return status;
        }


        /*
         * 从 Protect 进入运动模式时，
         * 先发送 Run。
         */
        if (active_mode
            == LK9025_MODE_PROTECT)
        {
            status =
                LK9025_Run(
                    motor
                );

            if (status != HAL_OK)
            {
                return status;
            }
        }


        /*
         * 更新 active mode。
         */
        motor->active_mode =
            command_mode;
    }


    /*
     * --------------------------------------------------------
     * Protect
     *
     * 已经在保护模式时不需要 500 Hz
     * 重复发送 Stop。
     * --------------------------------------------------------
     */

    if (command_mode
        == LK9025_MODE_PROTECT)
    {
        return HAL_OK;
    }


    /*
     * --------------------------------------------------------
     * Speed Mode
     * --------------------------------------------------------
     */

    if (command_mode
        == LK9025_MODE_SPEED)
    {
        return LK9025_SendSpeedCommand(
            motor,
            command_speed_rpm
        );
    }


    /*
     * --------------------------------------------------------
     * Position Mode
     * --------------------------------------------------------
     */

    if (command_mode
        == LK9025_MODE_POSITION)
    {
        return LK9025_SendPositionCommand(
            motor,
            command_angle_deg,
            max_speed_dps
        );
    }


    return HAL_ERROR;
}


/* ============================================================
 * 20. 获取 Telemetry
 * ============================================================ */

void LK9025_GetTelemetry(
    const LK9025_t *motor,
    uint32_t now_ms,
    LK9025_Telemetry_t *telemetry)
{
    uint32_t primask;

    uint8_t has_feedback;

    uint32_t last_feedback_tick;

    int16_t iq_raw;

    int16_t speed_dps;


    if ((motor == NULL)
        || (telemetry == NULL))
    {
        return;
    }


    /*
     * 取一份一致性快照。
     */
    primask =
        __get_PRIMASK();


    __disable_irq();


    has_feedback =
        motor->has_feedback;

    last_feedback_tick =
        motor->last_feedback_tick;

    telemetry->mode =
        motor->active_mode;

    telemetry->target_speed_rpm =
        motor->command_speed_rpm;

    telemetry->target_angle_deg =
        motor->command_angle_deg;

    telemetry->relative_angle_deg =
        motor->relative_angle_deg;

    telemetry->temperature =
        motor->temperature;

    iq_raw =
        motor->iq_raw;

    speed_dps =
        motor->speed_dps;


    if (primask == 0U)
    {
        __enable_irq();
    }


    /*
     * dps → rpm
     *
     * rpm = dps / 6
     */
    telemetry->actual_speed_rpm =
        (float)speed_dps
        /
        6.0f;


    /*
     * MF 系列：
     *
     * iq raw ±2048
     * 对应约 ±16.5 A。
     */
    telemetry->iq_amp =
        (
            (float)iq_raw
            * 16.5f
        )
        /
        2048.0f;


    /*
     * 在线判断。
     */
    telemetry->feedback_online =
        (
            (has_feedback != 0U)
            &&
            (
                (uint32_t)(
                    now_ms
                    -
                    last_feedback_tick
                )
                <=
                LK9025_FEEDBACK_TIMEOUT_MS
            )
        )
        ? 1U
        : 0U;
}
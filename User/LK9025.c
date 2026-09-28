#include "LK9025.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Protocol conversions. The motor expects little-endian values. */
static uint16_t LK9025_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
                      ((uint16_t)data[1] << 8U));
}

static int16_t LK9025_ReadS16LE(const uint8_t *data)
{
    return (int16_t)LK9025_ReadU16LE(data);
}

static void LK9025_WriteU16LE(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}

static void LK9025_WriteS32LE(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    data[0] = (uint8_t)(raw & 0xFFU);
    data[1] = (uint8_t)((raw >> 8U) & 0xFFU);
    data[2] = (uint8_t)((raw >> 16U) & 0xFFU);
    data[3] = (uint8_t)((raw >> 24U) & 0xFFU);
}

static float LK9025_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static void LK9025_Lock(void)
{
    __disable_irq();
}

static void LK9025_Unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static bool LK9025_IsStatusReplyCommand(uint8_t command)
{
    switch (command)
    {
        case LK9025_CMD_READ_STATUS2:
        case LK9025_CMD_TORQUE:
        case LK9025_CMD_SPEED:
        case LK9025_CMD_POSITION:
        case LK9025_CMD_POSITION_SPEED:
        case LK9025_CMD_SINGLE_POSITION:
        case LK9025_CMD_SINGLE_POSITION_SPEED:
        case LK9025_CMD_INCREMENT_POSITION:
            return true;
        default:
            return false;
    }
}

static void LK9025_RxCallback(const CANIO_Frame_t *frame, void *user_data)
{
    LK9025_t *motor = (LK9025_t *)user_data;
    uint16_t encoder;
    int32_t encoder_delta;
    uint32_t primask;

    if ((motor == NULL) || (frame == NULL) ||
        (frame->id != motor->can_id) || (frame->dlc != 8U) ||
        !LK9025_IsStatusReplyCommand(frame->data[0]))
    {
        return;
    }

    encoder = LK9025_ReadU16LE(&frame->data[6]);
    primask = __get_PRIMASK();
    LK9025_Lock();

    if (motor->has_feedback == 0U)
    {
        motor->last_encoder = encoder;
        motor->total_encoder_counts = 0;
        motor->relative_angle_deg = 0.0f;
    }
    else
    {
        encoder_delta = (int32_t)encoder - (int32_t)motor->last_encoder;
        if (encoder_delta > LK9025_ENCODER_HALF_RANGE)
        {
            encoder_delta -= LK9025_ENCODER_RESOLUTION;
        }
        else if (encoder_delta < -LK9025_ENCODER_HALF_RANGE)
        {
            encoder_delta += LK9025_ENCODER_RESOLUTION;
        }

        motor->total_encoder_counts += (int64_t)encoder_delta;
        motor->relative_angle_deg =
            (float)motor->total_encoder_counts * 360.0f /
            (float)LK9025_ENCODER_RESOLUTION;
        motor->last_encoder = encoder;
    }

    motor->temperature = (int8_t)frame->data[1];
    motor->iq_raw = LK9025_ReadS16LE(&frame->data[2]);
    motor->speed_dps = LK9025_ReadS16LE(&frame->data[4]);
    motor->encoder = encoder;
    motor->last_feedback_tick = frame->timestamp_ms;
    motor->has_feedback = 1U;

    LK9025_Unlock(primask);
}

static HAL_StatusTypeDef LK9025_Send(LK9025_t *motor,
                                      const uint8_t data[8])
{
    if ((motor == NULL) || (motor->bus == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    return CANIO_Send(motor->bus, motor->can_id, data) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef LK9025_SendSimple(LK9025_t *motor, uint8_t command)
{
    uint8_t data[8] = {0U};

    data[0] = command;
    return LK9025_Send(motor, data);
}

static HAL_StatusTypeDef LK9025_SendSpeed(LK9025_t *motor, float speed_rpm)
{
    uint8_t data[8] = {0U};
    int32_t speed_control;

    speed_rpm = LK9025_ClampFloat(speed_rpm,
                                  -LK9025_SPEED_COMMAND_LIMIT_RPM,
                                  LK9025_SPEED_COMMAND_LIMIT_RPM);
    /* 1 rpm = 6 degree/s; the command unit is 0.01 degree/s. */
    speed_control = (int32_t)(speed_rpm * 600.0f);

    data[0] = LK9025_CMD_SPEED;
    LK9025_WriteS32LE(&data[4], speed_control);
    return LK9025_Send(motor, data);
}

static HAL_StatusTypeDef LK9025_SendPosition(LK9025_t *motor,
                                              float angle_deg,
                                              uint16_t max_speed_dps)
{
    uint8_t data[8] = {0U};
    int32_t angle_control;

    angle_deg = LK9025_ClampFloat(angle_deg,
                                  -LK9025_POSITION_ANGLE_LIMIT_DEG,
                                  LK9025_POSITION_ANGLE_LIMIT_DEG);
    angle_control = (int32_t)(angle_deg * 100.0f);

    data[0] = LK9025_CMD_POSITION_SPEED;
    LK9025_WriteU16LE(&data[2], max_speed_dps);
    LK9025_WriteS32LE(&data[4], angle_control);
    return LK9025_Send(motor, data);
}

bool LK9025_Init(LK9025_t *motor, CANIO_Bus_t *bus, uint8_t motor_id)
{
    if ((motor == NULL) || (bus == NULL) ||
        (motor_id < LK9025_MIN_MOTOR_ID) ||
        (motor_id > LK9025_MAX_MOTOR_ID))
    {
        return false;
    }

    memset(motor, 0, sizeof(*motor));
    motor->bus = bus;
    motor->motor_id = motor_id;
    motor->can_id = LK9025_CAN_BASE_ID + (uint32_t)motor_id;
    motor->command_mode = LK9025_MODE_PROTECT;
    motor->active_mode = LK9025_MODE_PROTECT;
    motor->position_max_speed_dps = LK9025_POSITION_MAX_SPEED_DPS;

    return CANIO_Register(bus, motor->can_id, LK9025_RxCallback, motor);
}

void LK9025_SetMode(LK9025_t *motor, LK9025_ControlMode_t mode)
{
    uint32_t primask;

    if ((motor == NULL) || (mode > LK9025_MODE_POSITION))
    {
        return;
    }

    primask = __get_PRIMASK();
    LK9025_Lock();
    motor->command_mode = mode;
    LK9025_Unlock(primask);
}

void LK9025_SetTargetSpeed(LK9025_t *motor, float target_speed_rpm)
{
    uint32_t primask;

    if ((motor == NULL) || !isfinite(target_speed_rpm))
    {
        return;
    }

    target_speed_rpm = LK9025_ClampFloat(target_speed_rpm,
                                         -LK9025_SPEED_COMMAND_LIMIT_RPM,
                                         LK9025_SPEED_COMMAND_LIMIT_RPM);
    primask = __get_PRIMASK();
    LK9025_Lock();
    motor->command_speed_rpm = target_speed_rpm;
    LK9025_Unlock(primask);
}

void LK9025_SetTargetAngle(LK9025_t *motor, float target_angle_deg)
{
    uint32_t primask;

    if ((motor == NULL) || !isfinite(target_angle_deg))
    {
        return;
    }

    target_angle_deg = LK9025_ClampFloat(target_angle_deg,
                                         -LK9025_POSITION_ANGLE_LIMIT_DEG,
                                         LK9025_POSITION_ANGLE_LIMIT_DEG);
    primask = __get_PRIMASK();
    LK9025_Lock();
    motor->command_angle_deg = target_angle_deg;
    LK9025_Unlock(primask);
}

void LK9025_SetPositionMaxSpeed(LK9025_t *motor, uint16_t max_speed_dps)
{
    uint32_t primask;

    if (motor == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    LK9025_Lock();
    motor->position_max_speed_dps = max_speed_dps;
    LK9025_Unlock(primask);
}

HAL_StatusTypeDef LK9025_Run(LK9025_t *motor)
{
    return LK9025_SendSimple(motor, LK9025_CMD_RUN);
}

HAL_StatusTypeDef LK9025_Stop(LK9025_t *motor)
{
    return LK9025_SendSimple(motor, LK9025_CMD_STOP);
}

HAL_StatusTypeDef LK9025_Off(LK9025_t *motor)
{
    return LK9025_SendSimple(motor, LK9025_CMD_OFF);
}

HAL_StatusTypeDef LK9025_ReadStatus2(LK9025_t *motor)
{
    return LK9025_SendSimple(motor, LK9025_CMD_READ_STATUS2);
}

HAL_StatusTypeDef LK9025_UpdateControl(LK9025_t *motor, uint32_t now_ms)
{
    uint32_t primask;
    uint8_t has_feedback;
    uint32_t last_feedback_tick;
    LK9025_ControlMode_t command_mode;
    LK9025_ControlMode_t active_mode;
    float speed_rpm;
    float angle_deg;
    uint16_t max_speed_dps;
    HAL_StatusTypeDef status;

    if ((motor == NULL) || (motor->bus == NULL))
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    LK9025_Lock();
    has_feedback = motor->has_feedback;
    last_feedback_tick = motor->last_feedback_tick;
    command_mode = motor->command_mode;
    active_mode = motor->active_mode;
    speed_rpm = motor->command_speed_rpm;
    angle_deg = motor->command_angle_deg;
    max_speed_dps = motor->position_max_speed_dps;
    LK9025_Unlock(primask);

    if ((has_feedback != 0U) &&
        ((uint32_t)(now_ms - last_feedback_tick) >
         LK9025_FEEDBACK_TIMEOUT_MS))
    {
        /* A lost motor must be explicitly re-commanded after recovery. */
        primask = __get_PRIMASK();
        LK9025_Lock();
        motor->command_mode = LK9025_MODE_PROTECT;
        motor->active_mode = LK9025_MODE_PROTECT;
        LK9025_Unlock(primask);
        /* STOP acknowledgements do not contain status-2 telemetry.
         * After stopping, query real status to allow link recovery. */
        if (motor->timeout_stop_sent == 0U) {
            status = LK9025_Stop(motor);
            if (status == HAL_OK) {
                motor->timeout_stop_sent = 1U;
            }
            return status;
        }
        return LK9025_ReadStatus2(motor);
    }

    motor->timeout_stop_sent = 0U;

    if (command_mode != active_mode)
    {
        if (command_mode == LK9025_MODE_PROTECT)
        {
            status = LK9025_Stop(motor);
            if (status == HAL_OK)
            {
                primask = __get_PRIMASK();
                LK9025_Lock();
                motor->active_mode = LK9025_MODE_PROTECT;
                LK9025_Unlock(primask);
            }
            return status;
        }

        if (active_mode == LK9025_MODE_PROTECT)
        {
            status = LK9025_Run(motor);
            if (status != HAL_OK)
            {
                return status;
            }
        }

        primask = __get_PRIMASK();
        LK9025_Lock();
        motor->active_mode = command_mode;
        LK9025_Unlock(primask);
    }

    if (command_mode == LK9025_MODE_PROTECT)
    {
        return HAL_OK;
    }
    if (command_mode == LK9025_MODE_SPEED)
    {
        return LK9025_SendSpeed(motor, speed_rpm);
    }
    if (command_mode == LK9025_MODE_POSITION)
    {
        return LK9025_SendPosition(motor, angle_deg, max_speed_dps);
    }

    return HAL_ERROR;
}

void LK9025_GetTelemetry(const LK9025_t *motor,
                         uint32_t now_ms,
                         LK9025_Telemetry_t *telemetry)
{
    uint32_t primask;
    uint8_t has_feedback;
    uint32_t last_feedback_tick;
    int16_t iq_raw;
    int16_t speed_dps;

    if ((motor == NULL) || (telemetry == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    LK9025_Lock();
    telemetry->mode = motor->active_mode;
    telemetry->target_speed_rpm = motor->command_speed_rpm;
    telemetry->target_angle_deg = motor->command_angle_deg;
    telemetry->relative_angle_deg = motor->relative_angle_deg;
    telemetry->temperature = motor->temperature;
    has_feedback = motor->has_feedback;
    last_feedback_tick = motor->last_feedback_tick;
    iq_raw = motor->iq_raw;
    speed_dps = motor->speed_dps;
    LK9025_Unlock(primask);

    telemetry->actual_speed_rpm = (float)speed_dps / 6.0f;
    telemetry->iq_amp = (float)iq_raw * 16.5f / 2048.0f;
    telemetry->feedback_online =
        ((has_feedback != 0U) &&
         ((uint32_t)(now_ms - last_feedback_tick) <=
          LK9025_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
}

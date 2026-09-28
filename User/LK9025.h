#ifndef LK9025_H
#define LK9025_H

#include "main.h"
#include "canio.h"

#include <stdbool.h>
#include <stdint.h>

/* MF9025 V2 uses standard CAN IDs: 0x140 + motor ID. */
#define LK9025_CAN_BASE_ID                0x140U
#define LK9025_MIN_MOTOR_ID               1U
#define LK9025_MAX_MOTOR_ID               32U

/* Command bytes defined by the MF9025 V2 protocol. */
#define LK9025_CMD_OFF                    0x80U
#define LK9025_CMD_STOP                   0x81U
#define LK9025_CMD_RUN                    0x88U
#define LK9025_CMD_READ_STATUS2           0x9CU
#define LK9025_CMD_TORQUE                 0xA1U
#define LK9025_CMD_SPEED                  0xA2U
#define LK9025_CMD_POSITION               0xA3U
#define LK9025_CMD_POSITION_SPEED         0xA4U
#define LK9025_CMD_SINGLE_POSITION        0xA5U
#define LK9025_CMD_SINGLE_POSITION_SPEED  0xA6U
#define LK9025_CMD_INCREMENT_POSITION     0xA7U

/* These are software limits, not the motor's electrical limits. */
#define LK9025_SPEED_COMMAND_LIMIT_RPM    100.0f
#define LK9025_POSITION_ANGLE_LIMIT_DEG  21474836.0f
#define LK9025_POSITION_MAX_SPEED_DPS     360U
#define LK9025_FEEDBACK_TIMEOUT_MS        100U

/* The normal status frame reports a 16-bit encoder value. */
#define LK9025_ENCODER_RESOLUTION         65536L
#define LK9025_ENCODER_HALF_RANGE         32768L

typedef enum
{
    LK9025_MODE_PROTECT = 0,
    LK9025_MODE_SPEED,
    LK9025_MODE_POSITION
} LK9025_ControlMode_t;

typedef struct
{
    CANIO_Bus_t *bus;
    uint8_t motor_id;
    uint32_t can_id;

    LK9025_ControlMode_t command_mode;
    LK9025_ControlMode_t active_mode;

    float command_speed_rpm;
    float command_angle_deg;
    uint16_t position_max_speed_dps;

    int8_t temperature;
    int16_t iq_raw;
    int16_t speed_dps;
    uint16_t encoder;

    uint8_t has_feedback;
    uint16_t last_encoder;
    int64_t total_encoder_counts;
    float relative_angle_deg;

    uint32_t last_feedback_tick;
    uint8_t timeout_stop_sent;
} LK9025_t;

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

bool LK9025_Init(LK9025_t *motor, CANIO_Bus_t *bus, uint8_t motor_id);

void LK9025_SetMode(LK9025_t *motor, LK9025_ControlMode_t mode);
void LK9025_SetTargetSpeed(LK9025_t *motor, float target_speed_rpm);
void LK9025_SetTargetAngle(LK9025_t *motor, float target_angle_deg);
void LK9025_SetPositionMaxSpeed(LK9025_t *motor, uint16_t max_speed_dps);

HAL_StatusTypeDef LK9025_Run(LK9025_t *motor);
HAL_StatusTypeDef LK9025_Stop(LK9025_t *motor);
HAL_StatusTypeDef LK9025_Off(LK9025_t *motor);
HAL_StatusTypeDef LK9025_ReadStatus2(LK9025_t *motor);

HAL_StatusTypeDef LK9025_UpdateControl(LK9025_t *motor, uint32_t now_ms);

void LK9025_GetTelemetry(const LK9025_t *motor,
                         uint32_t now_ms,
                         LK9025_Telemetry_t *telemetry);

#endif /* LK9025_H */

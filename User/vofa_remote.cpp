#include "vofa_remote.hpp"

#include "remoteio.hpp"

#include <cstring>

extern UART_HandleTypeDef huart1;

volatile uint32_t vofa_remote_send_count = 0U;
volatile uint32_t vofa_remote_error_count = 0U;
volatile HAL_StatusTypeDef vofa_remote_last_status = HAL_OK;

namespace
{
constexpr uint32_t kChannelCount = 12U;
constexpr uint32_t kFrameSize = kChannelCount * sizeof(float) + 4U;

void WriteFloat(uint8_t *frame, uint32_t channel, float value)
{
    std::memcpy(
        frame + channel * sizeof(float),
        &value,
        sizeof(value));
}
}

HAL_StatusTypeDef VOFA_Remote_Send(void)
{
    uint8_t frame[kFrameSize] = {0};
    int16_t channels[8];
    uint8_t failsafe;
    uint8_t frame_lost;
    uint8_t offline;
    uint32_t decoded_count;
    uint32_t primask = __get_PRIMASK();

    /* Take a short coherent snapshot; do not transmit with IRQs disabled. */
    __disable_irq();

    for (uint32_t i = 0U; i < 8U; i++)
    {
        channels[i] = rc_ctrl.rc.ch[i];
    }
    failsafe = rc_ctrl.sbus_status.failsafe;
    frame_lost = rc_ctrl.sbus_status.frame_lost;
    offline = rc_ctrl.sbus_status.rc_offline;
    decoded_count = remote_debug.decoded_count;

    if (primask == 0U)
    {
        __enable_irq();
    }

    /* Channels 0..7: decoded RC values. */
    for (uint32_t i = 0U; i < 8U; i++)
    {
        WriteFloat(frame, i, static_cast<float>(channels[i]));
    }

    /* Channels 8..11: link state and valid-frame count. */
    WriteFloat(frame, 8U, offline != 0U ? 1.0f : 0.0f);
    WriteFloat(frame, 9U, failsafe != 0U ? 1.0f : 0.0f);
    WriteFloat(frame, 10U, frame_lost != 0U ? 1.0f : 0.0f);
    WriteFloat(frame, 11U, static_cast<float>(decoded_count));

    /* JustFloat synchronization tail: 0x0000807F, little endian. */
    frame[kFrameSize - 4U] = 0x00U;
    frame[kFrameSize - 3U] = 0x00U;
    frame[kFrameSize - 2U] = 0x80U;
    frame[kFrameSize - 1U] = 0x7FU;

    vofa_remote_last_status = HAL_UART_Transmit(
        &huart1,
        frame,
        static_cast<uint16_t>(sizeof(frame)),
        10U);

    if (vofa_remote_last_status == HAL_OK)
    {
        vofa_remote_send_count++;
    }
    else
    {
        vofa_remote_error_count++;
    }

    return vofa_remote_last_status;
}

#ifndef VOFA_REMOTE_HPP
#define VOFA_REMOTE_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

extern volatile uint32_t vofa_remote_send_count;
extern volatile uint32_t vofa_remote_error_count;
extern volatile HAL_StatusTypeDef vofa_remote_last_status;

HAL_StatusTypeDef VOFA_Remote_Send(void);

#ifdef __cplusplus
}
#endif

#endif

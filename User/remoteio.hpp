//
// Created by Yummy on 25-9-20.
//

#ifndef REMOTEIO_H
#define REMOTEIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH_DJI 18u
#define RC_FRAME_LENGTH_WFLY 25u
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

/* ----------------------------遥控器各摇杆回中值------------------------------ */
/* 此处为原始数据的回中值 可以根据实际情况修改 */
#define RC_CH0_VALUE_OFFSET      ((uint16_t)1024)  //右横摇杆
#define RC_CH1_VALUE_OFFSET      ((uint16_t)1024)  //右竖摇杆
#define RC_CH2_VALUE_OFFSET      ((uint16_t)1024+4)  //左横摇杆
#define RC_CH3_VALUE_OFFSET      ((uint16_t)1016)  //左竖摇杆

    /**
     * SBUS 帧结构（25字节，WFLY标准）：
     *   byte[0]     : 0x0F  帧头
     *   byte[1..22] : 16通道数据（每通道11bit，共176bit=22字节）
     *   byte[23]    : 标志字节（flags）
     *   byte[24]    : 0x00  帧尾
     *
     * 标志字节 bit 定义：
     *   bit2 (0x04) : Frame Lost  - 当前帧信号短暂丢失
     *   bit3 (0x08) : Failsafe    - 接收机已激活失控保护
     */
#define SBUS_FLAG_FRAME_LOST    ((uint8_t)(1 << 2))  /*!< 0x04: 帧丢失（短暂）*/
#define SBUS_FLAG_FAILSAFE      ((uint8_t)(1 << 3))  /*!< 0x08: 失控保护已激活 */

#define RC_OFFLINE_TIMEOUT      ((uint16_t)500)

    void REMOTEIO_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
    void RemoteIO_IRQHandler(void);
    void REMOTEIO_UpdateStatus(void);  /*!< 在 1ms 定时中断中调用，维护离线看门狗 */
    uint8_t REMOTEIO_IsOffline(void);     /*!< 返回1=遥控器断电/离线/Failsafe，0=正常在线 */

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/* ----------------------- Data Struct ------------------------------------- */
#pragma pack(push, 1)      // 保存当前对齐方式，设置为1字节对齐

    typedef struct __attribute__((packed)) {
        uint8_t failsafe   : 1;  /*!< 1 = 接收机已触发失控保护（遥控断电/超距） */
        uint8_t frame_lost : 1;  /*!< 1 = 当前帧信号短暂丢失 */
        uint8_t rc_offline : 1;  /*!< 1 = 软件看门狗判定离线（超时未收到有效帧） */
        uint8_t reserved   : 5;
    } SBUS_Status_t;

typedef struct
{
    struct
    {
        /* 遥控器摇杆和拨杆 */
        int16_t ch[8];
        char s[4];
    } rc;

    SBUS_Status_t sbus_status;
    uint16_t      rc_live_cnt_WFLY;  /*!< 在线倒计时，由 REMOTEIO_UpdateStatus() 递减 */
} RC_ctrl_t;

#pragma pack(pop)  // 恢复之前的对齐方式

extern RC_ctrl_t rc_ctrl;

#ifdef __cplusplus
class remoteio {

};
#endif

#ifdef __cplusplus
}
#endif

#endif //REMOTEIO_H

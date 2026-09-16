#ifndef CANIO_H
#define CANIO_H

#include "main.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include <stdbool.h>

#define CANIO_RX_RING_SIZE  32
#define CANIO_MAX_ROUTES    8

#define CANIO_RX_FLAG       (1U << 0)

/* 一帧 CAN 数据 */
typedef struct
{
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    uint32_t timestamp_ms;

} CANIO_Frame_t;


/* 回调函数类型 */
typedef void (*CANIO_RxCallback_t)(
    const CANIO_Frame_t *frame,
    void *user_data
);


/* CAN ID 路由表的一项 */
typedef struct
{
    uint32_t id;

    CANIO_RxCallback_t callback;

    void *user_data;

} CANIO_Route_t;


/* 一整条 CAN 总线 */
typedef struct
{
    CAN_HandleTypeDef *hcan;

    /* RX RingBuffer */
    CANIO_Frame_t rx_ring[CANIO_RX_RING_SIZE];

    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;

    /* ID → callback */
    CANIO_Route_t routes[CANIO_MAX_ROUTES];

    uint8_t route_count;

    /* 处理 CAN 的任务 */
    osThreadId_t rx_task;

    /* 调试统计 */
    uint32_t rx_count;
    uint32_t rx_overflow;

} CANIO_Bus_t;


void CANIO_Init(
    CANIO_Bus_t *bus,
    CAN_HandleTypeDef *hcan
);

bool CANIO_Register(
    CANIO_Bus_t *bus,
    uint32_t id,
    CANIO_RxCallback_t callback,
    void *user_data
);

void CANIO_RxIRQ(
    CANIO_Bus_t *bus
);

void CANIO_ProcessRx(
    CANIO_Bus_t *bus
);

bool CANIO_Send(
    CANIO_Bus_t *bus,
    uint32_t id,
    const uint8_t data[8]
);

#endif
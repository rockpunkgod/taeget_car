#ifndef CANIO_H /* 头文件保护：首次包含时，CANIO_H 尚未定义。 */
#define CANIO_H /* 定义标记，防止同一编译单元重复定义类型。 */

#include "stm32f4xx_hal.h" /* 提供 CAN_HandleTypeDef；硬件操作只放在 canio.c。 */
#include "cmsis_os2.h"     /* 提供 CMSIS-RTOS V2 的任务类型和 Thread Flags。 */
#include <stdint.h>        /* uint8_t/uint16_t/uint32_t：无符号定宽整数。 */
#include <stdbool.h>       /* 提供 bool、true、false。 */
#include <stddef.h>        /* 提供空指针常量 NULL。 */

#define CANIO_RX_RING_SIZE 32U /* 静态数组长度；留一个空位，实际容量是 31 帧。 */
#define CANIO_MAX_ROUTES    8U /* 最多注册 8 条路由；U 表示无符号整数常量。 */
#define CANIO_RX_FLAG (1U << 0) /* 左移 0 位，选用任务标志的第 0 位。 */

/* typedef 为类型命名；struct 把多个成员组成一个整体；末尾必须有分号。 */
typedef struct
{
    uint32_t id;           /* 标准 CAN ID，合法范围 0..0x7FF。 */
    uint8_t dlc;           /* 有效数据字节数，0..8。 */
    uint8_t data[8];       /* [] 声明数组；CANIO 只搬运字节，不解释内容。 */
    uint32_t timestamp_ms; /* 从硬件 FIFO 读出时的 HAL 毫秒时间，不是总线到达时刻。 */
} CANIO_Frame_t;           /* 类型名，不是一个实际变量。 */


typedef void (*CANIO_RxCallback_t)(const CANIO_Frame_t *frame, void *user_data);

typedef struct
{
    uint32_t id;                  /* 按此 ID 匹配。 */
    CANIO_RxCallback_t callback;  /* 存函数地址，之后通过它调用函数。 */
    void *user_data;             /* 存调用者的对象地址；允许为 NULL。 */
} CANIO_Route_t;

typedef struct
{
    CAN_HandleTypeDef *hcan; /* * 在声明中表示指针；不复制 HAL 句柄。 */
    CANIO_Frame_t rx_ring[CANIO_RX_RING_SIZE]; /* 内嵌数组，不分配堆内存。 */
    volatile uint16_t rx_head; /* 下次写入位置：运行中只由 FIFO0 ISR 修改。 */
    volatile uint16_t rx_tail; /* 下次读取位置：运行中只由唯一 RX 任务修改。 */
    CANIO_Route_t routes[CANIO_MAX_ROUTES]; /* 固定容量的路由表。 */
    uint8_t route_count;     /* 已注册条数；只在启动接收前修改。 */
    osThreadId_t rx_task;    /* Phase 2 创建任务后填写；初始为 NULL。 */
    volatile uint32_t rx_count;    /* 从硬件成功读出的总帧数，包括被丢弃帧。 */
    volatile uint32_t rx_overflow; /* 软件环满而丢弃的新帧数。 */
    volatile uint32_t rx_rejected; /* 非标准数据帧或 DLC 非法的帧数。 */
    volatile uint32_t rx_errors;   /* HAL 读取失败次数；不是完整 CAN 错误监测。 */
    volatile uint32_t wake_errors; /* Thread Flags 通知失败次数。 */
} CANIO_Bus_t;

void CANIO_Init(CANIO_Bus_t *bus, CAN_HandleTypeDef *hcan);
bool CANIO_Register(CANIO_Bus_t *bus, uint32_t id,
                    CANIO_RxCallback_t callback, void *user_data);
void CANIO_RxIRQ(CANIO_Bus_t *bus);
void CANIO_ProcessRx(CANIO_Bus_t *bus);

bool CANIO_Send(CANIO_Bus_t *bus, uint32_t id, const uint8_t data[8]);

#endif

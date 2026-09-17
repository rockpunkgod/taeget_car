#include "canio.h" 
#include <string.h> /* 提供 memset：按字节设置内存。 */


static bool CANIO_RingPush(CANIO_Bus_t *bus, const CANIO_Frame_t *frame)
{
    /* 计算下一格；% 是取余，(uint16_t) 是显式类型转换。 */
    uint16_t next = (uint16_t)((bus->rx_head + 1U) % CANIO_RX_RING_SIZE);
    if (next == bus->rx_tail) /* == 比较；下一格追上 tail 表示满。 */
    {
        bus->rx_overflow++; /* ++ 加一；保留旧帧，丢弃本次新帧。 */
        return false; /* return 结束函数并返回布尔值。 */
    }
    __DMB(); /* 观察 tail 后才允许复用空位，与消费者发布 tail 配对。 */
    bus->rx_ring[bus->rx_head] = *frame; /* * 在表达式中解引用；结构体整体复制。 */
    __DMB(); /* 先写完整帧，再公布 head；屏障同时约束内存访问顺序。 */
    bus->rx_head = next; /* = 赋值；生产者不修改 tail。 */
    return true;
}

/* RX RingBuffer Pop：唯一消费者取最早的一帧，frame 是输出参数。 */
static bool CANIO_RingPop(CANIO_Bus_t *bus, CANIO_Frame_t *frame)
{
    if (bus->rx_tail == bus->rx_head) /* 两个索引相等表示空。 */
    {
        return false;
    }
    __DMB(); /* 确认 head 已公布后，再读取对应帧。 */
    *frame = bus->rx_ring[bus->rx_tail]; /* 写入调用者提供的结构体对象。 */
    __DMB(); /* 先复制完整帧，再把这一格交还给生产者。 */
    bus->rx_tail = (uint16_t)((bus->rx_tail + 1U) % CANIO_RX_RING_SIZE);
    return true;
}

/* Dispatcher：查表并调用回调；这里不知道任何设备协议。 */
static void CANIO_Dispatch(CANIO_Bus_t *bus, const CANIO_Frame_t *frame)
{
    /* for 的三个部分分别是：初始化；继续条件；每轮之后的更新。 */
    for (uint8_t i = 0U; i < bus->route_count; i++)
    {
        const CANIO_Route_t *route = &bus->routes[i]; /* & 取第 i 项地址，const 只读。 */
        if (route->id == frame->id)
        {
            route->callback(frame, route->user_data); /* 函数指针调用，原样转交对象地址。 */
            return; /* void 函数不返回值；唯一 ID 匹配后立即结束。 */
        }
    }
    /* 未注册的 ID 没有处理者，直接忽略。 */
}

/* 软件初始化：只在接收中断和消费任务尚未使用 bus 时调用。 */
void CANIO_Init(CANIO_Bus_t *bus, CAN_HandleTypeDef *hcan)
{
    if (bus == NULL) /* NULL 是空指针；不能通过空指针访问成员。 */
    {
        return;
    }
    memset(bus, 0, sizeof(*bus)); /* sizeof(*bus) 是整个对象大小，不是指针大小。
                                 * 本 STM32 ARM 平台全零可表示空指针。 */
    bus->hcan = hcan; /* 保存地址；若 hcan 为 NULL，其他公共接口会拒绝操作。 */
}

/* 路由注册：输入总线、ID、回调及上下文；成功 true，失败 false。 */
bool CANIO_Register(CANIO_Bus_t *bus, uint32_t id,
                    CANIO_RxCallback_t callback, void *user_data)
{
    /* || 是逻辑或，左侧为真就不再计算右侧，因此不会解引用空 bus。 */
    if ((bus == NULL) || (callback == NULL) || (bus->hcan == NULL))
    {
        return false;
    }
    if ((id > 0x7FFU) || (bus->route_count >= CANIO_MAX_ROUTES))
    {
        return false; /* 0x 前缀表示十六进制；标准 ID 最多 11 位。 */
    }
    for (uint8_t i = 0U; i < bus->route_count; i++)
    {
        if (bus->routes[i].id == id) /* [] 先取数组元素，再用 . 访问元素成员。 */
        {
            return false; /* 一个 ID 只允许一条路由。 */
        }
    }
    CANIO_Route_t *route = &bus->routes[bus->route_count]; /* 下一条空路由的地址。 */
    route->id = id;
    route->callback = callback; /* 保存函数地址，不是在这里调用函数。 */
    route->user_data = user_data; /* 只保存地址，不复制对象，也不解释其类型。 */
    bus->route_count++; /* 三个字段填好后，已注册数量才增加。 */
    return true;
}

/* RX IRQ：仅搬运、计数、通知；Phase 2 的 HAL 回调将调用此入口。
 * 每次最多读入口时已有的三帧，持续来帧不会使本次 ISR 无限延长。
 */
void CANIO_RxIRQ(CANIO_Bus_t *bus)
{
    if ((bus == NULL) || (bus->hcan == NULL))
    {
        return;
    }
    uint32_t pending = HAL_CAN_GetRxFifoFillLevel(bus->hcan, CAN_RX_FIFO0);
    if (pending > 3U) /* bxCAN 每个接收 FIFO 深度为 3；明确限制工作量。 */
    {
        pending = 3U;
    }
    bool queued = false; /* 局部布尔变量：本次是否至少成功入队一帧。 */
    for (uint32_t i = 0U; i < pending; i++)
    {
        CAN_RxHeaderTypeDef header = {0}; /* {0} 初始化结构体的所有字段。 */
        CANIO_Frame_t frame = {0}; /* 局部临时帧，每次循环重新初始化。 */
        if (HAL_CAN_GetRxMessage(bus->hcan, CAN_RX_FIFO0,
                                 &header, frame.data) != HAL_OK) /* != 表示不等于。 */
        {
            bus->rx_errors++;
            break; /* 读取失败结束循环，不在 ISR 中反复重试。 */
        }
        bus->rx_count++; /* 成功读取硬件帧即计数，即使随后过滤或软件队列满。 */
        if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA) ||
            (header.DLC > 8U))
        {
            bus->rx_rejected++;
            continue; /* 跳过本轮剩余语句，处理下一帧。 */
        }
        frame.id = header.StdId; /* 对象用 .，指针才用 ->。 */
        frame.dlc = (uint8_t)header.DLC; /* 已检查 <=8，缩窄转换不会丢失有效值。 */
        frame.timestamp_ms = HAL_GetTick(); /* 记录毫秒，不使用 RTOS tick 冒充毫秒。 */
        if (CANIO_RingPush(bus, &frame)) /* & 传地址；Push 会复制完整对象。 */
        {
            queued = true;
        }
    }
    if (queued && (bus->rx_task != NULL)) /* && 是逻辑与，两条件都真才执行。 */
    {
        uint32_t flags = osThreadFlagsSet(bus->rx_task, CANIO_RX_FLAG);
        if ((flags & osFlagsError) != 0U) /* 此处 & 是按位与，不是取地址。 */
        {
            bus->wake_errors++; /* 记录错误，不在 ISR 中打印或阻塞重试。 */
        }
    }
}

/* 任务入口：消费软件队列，不轮询 CAN 外设；回调在这个调用者上下文运行。 */
void CANIO_ProcessRx(CANIO_Bus_t *bus)
{
    if ((bus == NULL) || (bus->hcan == NULL))
    {
        return;
    }
    CANIO_Frame_t frame; /* Pop 成功时才读取，因此不需要预先初始化。 */
    while (CANIO_RingPop(bus, &frame)) /* 每轮消费一帧；空队列立即退出。 */
    {
        CANIO_Dispatch(bus, &frame); /* 回调不能保存这个临时 frame 指针供之后使用。 */
    }
}

/* TX：固定发送 8 字节标准数据帧，只尝试一次，不等待空邮箱。
 * 每条总线必须由单一发送任务使用，详见头文件中的并发约束。
 */
bool CANIO_Send(CANIO_Bus_t *bus, uint32_t id, const uint8_t data[8])
{
    if ((bus == NULL) || (bus->hcan == NULL) || (data == NULL) || (id > 0x7FFU))
    {
        return false;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(bus->hcan) == 0U)
    {
        return false; /* 邮箱满立即失败；绝不 busy wait。 */
    }
    CAN_TxHeaderTypeDef header = {0}; /* 所有字段先初始化，包括不用的 ExtId。 */
    uint32_t mailbox; /* HAL 的输出参数：接收所选邮箱编号。 */
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    /* 项目中的 HAL 接受 const 数据指针，无需强转去掉 const。 */
    return HAL_CAN_AddTxMessage(bus->hcan, &header, data, &mailbox) == HAL_OK;
}

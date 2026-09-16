#include "canio.h"
#include <string.h>


/* ========================================================================== */ // 分隔线：下面开始 RingBuffer 私有函数。
/*                         1. RX RingBuffer：写入数据                           */ // 这个模块负责把 CAN ISR 收到的数据放进软件缓冲区。
/* ========================================================================== */ // 分隔线。

static bool CANIO_RingPush(CANIO_Bus_t *bus, const CANIO_Frame_t *frame)          // static 表示这个函数只能被 canio.c 内部使用。
{                                                                                 // 函数体开始。

    uint16_t next = (bus->rx_head + 1U) % CANIO_RX_RING_SIZE;                     // 计算 head 的下一个位置，超过数组末尾后自动绕回 0。

    if (next == bus->rx_tail)                                                      // 如果 head 的下一个位置撞上 tail，说明 RingBuffer 已经满了。
    {                                                                             // 缓冲区已满的处理开始。

        bus->rx_overflow++;                                                        // 记录一次软件接收缓冲区溢出，便于后续调试 CAN 是否处理不过来。

        return false;                                                              // 当前 CAN 帧没有成功放入 RingBuffer，返回失败。

    }                                                                             // 缓冲区已满处理结束。

    bus->rx_ring[bus->rx_head] = *frame;                                           // 把 frame 指向的整帧 CAN 数据复制到当前 head 指向的位置。

    __DMB();                                                                       // 数据内存屏障，确保帧数据写完之后再更新 head，避免任务读到未完全写好的数据。

    bus->rx_head = next;                                                           // 更新 head，告诉消费者：又多了一帧可以读取的数据。

    return true;                                                                   // 数据成功写入 RingBuffer，返回成功。

}                                                                                 // CANIO_RingPush() 结束。



/* ========================================================================== */ // 分隔线：下面开始 RingBuffer 读取。
/*                         2. RX RingBuffer：读取数据                           */ // CANRxTask 会通过这个函数从软件缓冲区取出 CAN 帧。
/* ========================================================================== */ // 分隔线。

static bool CANIO_RingPop(CANIO_Bus_t *bus, CANIO_Frame_t *frame)                 // 从 RingBuffer 取出最早进入的一帧数据。
{                                                                                 // 函数体开始。

    if (bus->rx_head == bus->rx_tail)                                              // 如果 head 和 tail 相等，说明 RingBuffer 当前没有任何未处理数据。
    {                                                                             // 空缓冲区处理开始。

        return false;                                                              // 没有数据可以读取，直接返回失败。

    }                                                                             // 空缓冲区处理结束。

    *frame = bus->rx_ring[bus->rx_tail];                                           // 把 tail 指向的那一帧复制到调用者提供的 frame 中。

    __DMB();                                                                       // 确保当前帧已经读取完毕之后，再移动 tail。

    bus->rx_tail = (bus->rx_tail + 1U) % CANIO_RX_RING_SIZE;                       // tail 向后移动一格，表示这一帧已经被消费。

    return true;                                                                   // 成功取出一帧 CAN 数据。

}                                                                                 // CANIO_RingPop() 结束。



/* ========================================================================== */ // 分隔线：下面是 CAN ID 路由器。
/*                         3. Dispatcher：按 CAN ID 分发                        */ // 根据 CAN ID 找出应该调用哪个设备驱动的 Callback。
/* ========================================================================== */ // 分隔线。

static void CANIO_Dispatch(CANIO_Bus_t *bus, const CANIO_Frame_t *frame)          // 输入一帧 CAN 数据，并在 Route Table 中寻找对应处理函数。
{                                                                                 // 函数体开始。

    for (uint8_t i = 0U; i < bus->route_count; i++)                               // 从 routes[0] 开始遍历所有已经注册的 Route。
    {                                                                             // for 循环开始。

        CANIO_Route_t *route = &bus->routes[i];                                    // 取得当前第 i 条 Route 的地址，方便后续访问。

        if (route->id != frame->id)                                                // 如果当前 Route 的 CAN ID 和收到的 CAN ID 不相同。
        {                                                                         // ID 不匹配处理开始。

            continue;                                                             // 跳过当前 Route，继续检查下一条 Route。

        }                                                                         // ID 不匹配处理结束。

        if (route->callback == NULL)                                               // 如果虽然 ID 匹配，但没有合法的回调函数。
        {                                                                         // 空回调保护开始。

            break;                                                                // 停止处理，避免调用 NULL 函数指针导致 MCU HardFault。

        }                                                                         // 空回调保护结束。

        route->callback(frame, route->user_data);                                  // 调用设备自己的 Callback，并把 CAN 帧和设备对象地址一起传进去。

        break;                                                                    // 当前设计一个 CAN ID 只允许对应一个设备，因此找到后停止继续查表。

    }                                                                             // for 循环结束。

}                                                                                 // CANIO_Dispatch() 结束。



/* ========================================================================== */ // 分隔线：下面开始 CANIO 初始化。
/*                              4. CANIO 初始化                                 */ // 初始化软件对象，但这里暂时不启动真正的 CAN 外设。
/* ========================================================================== */ // 分隔线。

void CANIO_Init(CANIO_Bus_t *bus, CAN_HandleTypeDef *hcan)                        // 初始化一个 CANIO_Bus_t，并把它绑定到某个 STM32 CAN 外设。
{                                                                                 // 函数体开始。

    if ((bus == NULL) || (hcan == NULL))                                           // 检查调用者有没有传入空指针。
    {                                                                             // 参数错误处理开始。

        return;                                                                   // 参数非法时直接退出，避免访问 NULL 地址。

    }                                                                             // 参数错误处理结束。

    memset(bus, 0, sizeof(CANIO_Bus_t));                                           // 把整个 CANIO_Bus_t 清零，让 head、tail、计数器、Route 等都从 0 开始。

    bus->hcan = hcan;                                                              // 保存 HAL CAN 句柄，例如把 &hcan1 保存到 can1_bus 中。

}                                                                                 // CANIO_Init() 结束。



/* ========================================================================== */ // 分隔线：下面是设备注册。
/*                         5. 注册 CAN ID 与 Callback                            */ // 设备驱动通过这个接口告诉 CANIO：“这个 ID 属于我”。
/* ========================================================================== */ // 分隔线。

bool CANIO_Register(CANIO_Bus_t *bus,                                             // bus 指向需要注册设备的 CAN 总线。
                    uint32_t id,                                                   // id 是设备监听的标准 CAN ID。
                    CANIO_RxCallback_t callback,                                   // callback 是收到这个 ID 后需要执行的函数。
                    void *user_data)                                               // user_data 是对应设备对象的地址。
{                                                                                 // 函数体开始。

    if ((bus == NULL) || (callback == NULL))                                       // CAN Bus 或 Callback 为 NULL 都属于非法参数。
    {                                                                             // 参数检查开始。

        return false;                                                              // 注册失败。

    }                                                                             // 参数检查结束。

    if (id > 0x7FFU)                                                              // 当前版本只支持 11-bit 标准 CAN ID，所以最大值只能是 0x7FF。
    {                                                                             // CAN ID 合法性检查开始。

        return false;                                                              // 超过 0x7FF 表示不是合法标准帧 ID，本次注册失败。

    }                                                                             // CAN ID 合法性检查结束。

    if (bus->route_count >= CANIO_MAX_ROUTES)                                      // 如果注册数量已经达到 Route Table 最大容量。
    {                                                                             // Route Table 满处理开始。

        return false;                                                              // 没有空间继续注册设备。

    }                                                                             // Route Table 满处理结束。

    for (uint8_t i = 0U; i < bus->route_count; i++)                               // 遍历已经注册的 Route，检查 CAN ID 是否重复。
    {                                                                             // 重复检查循环开始。

        if (bus->routes[i].id == id)                                               // 如果当前 CAN ID 已经注册过。
        {                                                                         // 重复 ID 处理开始。

            return false;                                                          // 拒绝重复注册，避免同一个 ID 对应两个不同设备。

        }                                                                         // 重复 ID 处理结束。

    }                                                                             // 重复检查循环结束。

    CANIO_Route_t *route = &bus->routes[bus->route_count];                         // 找到 routes[] 中下一块还没被使用的位置。

    route->id = id;                                                               // 保存需要监听的 CAN ID。

    route->callback = callback;                                                   // 保存这个 CAN ID 对应的处理函数地址。

    route->user_data = user_data;                                                 // 保存这个 CAN ID 对应的设备对象地址。

    bus->route_count++;                                                           // 已注册 Route 数量增加 1。

    return true;                                                                  // CAN ID 注册成功。

}                                                                                 // CANIO_Register() 结束。



/* ========================================================================== */ // 分隔线：下面是 CAN 接收中断真正调用的入口。
/*                           6. CAN RX 中断处理                                  */ // 这里只负责“从硬件 FIFO 搬数据到软件 RingBuffer”。
/* ========================================================================== */ // 分隔线。

void CANIO_RxIRQ(CANIO_Bus_t *bus)                                                // 当 CAN FIFO0 收到新数据时，由 HAL Callback 调用这个函数。
{                                                                                 // 函数体开始。

    if ((bus == NULL) || (bus->hcan == NULL))                                      // 检查 CANIO 对象和 HAL CAN 句柄是否有效。
    {                                                                             // 参数异常处理开始。

        return;                                                                   // 无效时直接退出中断处理。

    }                                                                             // 参数异常处理结束。

    CAN_RxHeaderTypeDef rx_header;                                                // 定义 STM32 HAL 使用的 CAN 接收帧头。

    CANIO_Frame_t frame;                                                          // 定义 CANIO 自己使用的统一 CAN 帧对象。

    bool received_any = false;                                                    // 用于记录这次 ISR 是否真的成功收到过至少一帧数据。

    while (HAL_CAN_GetRxFifoFillLevel(bus->hcan, CAN_RX_FIFO0) > 0U)               // 只要硬件 FIFO0 里面还有 CAN 帧，就继续快速把它们搬出来。
    {                                                                             // FIFO 搬运循环开始。

        memset(&frame, 0, sizeof(CANIO_Frame_t));                                  // 每次读取新帧前先清空临时 frame，避免残留上一次数据。

        if (HAL_CAN_GetRxMessage(bus->hcan,                                        // 指定从当前 CAN 外设读取数据。
                                 CAN_RX_FIFO0,                                     // 指定从硬件 FIFO0 读取。
                                 &rx_header,                                       // 把 CAN 帧头信息写到 rx_header 中。
                                 frame.data) != HAL_OK)                            // 把最多 8 Byte 数据直接写入 frame.data。
        {                                                                         // HAL 读取失败处理开始。

            break;                                                                // 如果硬件读取失败，就停止继续处理 FIFO。

        }                                                                         // HAL 读取失败处理结束。

        bus->rx_count++;                                                          // 成功从硬件 CAN FIFO 取出了一帧，总接收帧计数加 1。

        received_any = true;                                                      // 标记本次 IRQ 确实处理过 CAN 数据。

        if (rx_header.IDE != CAN_ID_STD)                                           // 当前 CANIO 教学版本只支持标准帧。
        {                                                                         // 扩展帧过滤开始。

            continue;                                                             // 如果收到扩展帧，当前版本直接忽略。

        }                                                                         // 扩展帧过滤结束。

        if (rx_header.RTR != CAN_RTR_DATA)                                         // 当前只处理真正携带数据的 Data Frame。
        {                                                                         // Remote Frame 过滤开始。

            continue;                                                             // 如果是 Remote Frame，则当前版本直接忽略。

        }                                                                         // Remote Frame 过滤结束。

        if (rx_header.DLC > 8U)                                                   // 经典 CAN 数据区最大只能是 8 Byte。
        {                                                                         // DLC 防御性检查开始。

            continue;                                                             // 如果 DLC 异常，直接丢弃这一帧。

        }                                                                         // DLC 防御性检查结束。

        frame.id = rx_header.StdId;                                                // 从 HAL 帧头中取出 11-bit 标准 CAN ID。

        frame.dlc = (uint8_t)rx_header.DLC;                                        // 保存这一帧实际携带的数据长度。

        frame.timestamp_ms = HAL_GetTick();                                        // 保存当前毫秒时间戳，后面可以用于判断设备是否掉线。

        (void)CANIO_RingPush(bus, &frame);                                         // 尝试把 CAN 帧写入软件 RingBuffer；满了时函数内部会增加 overflow_count。

    }                                                                             // FIFO 搬运循环结束。

    if ((received_any == true) && (bus->rx_task != NULL))                          // 只有确实收到数据并且 CANRxTask 已经创建时才发送任务通知。
    {                                                                             // Task 通知条件满足。

        (void)osThreadFlagsSet(bus->rx_task, CANIO_RX_FLAG);                       // 设置 CANIO_RX_FLAG，把睡眠中的 CANRxTask 唤醒。

    }                                                                             // Task 通知结束。

}                                                                                 // CANIO_RxIRQ() 结束。



/* ========================================================================== */ // 分隔线：下面是在普通 FreeRTOS Task 上下文处理 CAN。
/*                         7. 处理软件 RX RingBuffer                             */ // 这一层已经离开中断，因此可以安全做 Route 查找与协议 Callback。
/* ========================================================================== */ // 分隔线。

void CANIO_ProcessRx(CANIO_Bus_t *bus)                                            // CANRxTask 被唤醒以后调用这个函数。
{                                                                                 // 函数体开始。

    if (bus == NULL)                                                              // 防止调用者传入空 CAN Bus。
    {                                                                             // 参数检查开始。

        return;                                                                   // 参数错误时直接退出。

    }                                                                             // 参数检查结束。

    CANIO_Frame_t frame;                                                          // 定义临时变量，用于每次从 RingBuffer 中取出一帧。

    while (CANIO_RingPop(bus, &frame) == true)                                     // 只要 RingBuffer 里面还有未处理 CAN 帧，就持续处理。
    {                                                                             // 软件缓冲区消费循环开始。

        CANIO_Dispatch(bus, &frame);                                               // 根据 frame.id 查询 Route Table，并调用对应设备 Callback。

    }                                                                             // 软件缓冲区消费循环结束。

}                                                                                 // CANIO_ProcessRx() 结束。



/* ========================================================================== */ // 分隔线：下面是发送接口。
/*                              8. CAN 发送                                      */ // 上层设备驱动通过这个函数发送标准 8 Byte CAN 帧。
/* ========================================================================== */ // 分隔线。

bool CANIO_Send(CANIO_Bus_t *bus, uint32_t id, const uint8_t data[8])             // 向指定 CAN ID 发送一帧 8 Byte 标准数据帧。
{                                                                                 // 函数体开始。

    if ((bus == NULL) || (bus->hcan == NULL) || (data == NULL))                    // 检查 CAN Bus、HAL CAN 和数据指针是否有效。
    {                                                                             // 参数错误处理开始。

        return false;                                                             // 参数非法，不能发送。

    }                                                                             // 参数错误处理结束。

    if (id > 0x7FFU)                                                              // 当前版本只发送 11-bit 标准 CAN ID。
    {                                                                             // CAN ID 合法性检查开始。

        return false;                                                             // ID 超出标准帧范围则发送失败。

    }                                                                             // CAN ID 合法性检查结束。

    CAN_TxHeaderTypeDef tx_header;                                                // 定义 STM32 HAL CAN 发送帧头。

    uint32_t tx_mailbox;                                                          // HAL 会通过这个变量告诉我们使用了哪一个 CAN TX Mailbox。

    memset(&tx_header, 0, sizeof(CAN_TxHeaderTypeDef));                            // 先把帧头全部清零，避免结构体中存在未初始化字段。

    tx_header.StdId = id;                                                         // 设置标准 CAN ID，例如 MF9025v2 ID=1 时为 0x141。

    tx_header.ExtId = 0U;                                                         // 当前不用扩展 CAN ID，所以 ExtId 清零。

    tx_header.IDE = CAN_ID_STD;                                                   // 明确告诉 bxCAN：当前发送的是 11-bit 标准帧。

    tx_header.RTR = CAN_RTR_DATA;                                                 // 当前发送的是 Data Frame，而不是 Remote Frame。

    tx_header.DLC = 8U;                                                           // 当前 CANIO_Send() 固定发送完整 8 Byte 数据。

    tx_header.TransmitGlobalTime = DISABLE;                                        // 不使用 bxCAN 的全局时间戳发送功能。

    if (HAL_CAN_GetTxMailboxesFreeLevel(bus->hcan) == 0U)                         // 查询三个硬件 TX Mailbox 是否至少还有一个空闲。
    {                                                                             // Mailbox 全满处理开始。

        return false;                                                             // 当前基础版本不 busy wait，也不排队，直接告诉上层“暂时发送失败”。

    }                                                                             // Mailbox 全满处理结束。

    if (HAL_CAN_AddTxMessage(bus->hcan,                                            // 指定使用当前 CAN Bus 对应的 STM32 CAN 外设。
                             &tx_header,                                           // 把刚才配置好的发送帧头交给 HAL。
                             (uint8_t *)data,                                      // 把 8 Byte 数据交给 HAL；HAL 接口不是 const，所以这里需要转换指针类型。
                             &tx_mailbox) != HAL_OK)                               // HAL 会选择一个可用 TX Mailbox 并开始发送。
    {                                                                             // HAL 发送提交失败处理开始。

        return false;                                                             // CAN 帧没有成功加入硬件发送 Mailbox。

    }                                                                             // HAL 发送提交失败处理结束。

    return true;                                                                  // CAN 帧已经成功提交给 STM32 CAN 外设。

}                                                                                 // CANIO_Send() 结束。
//
// Created by Yummy on 25-5-31.
//

#include "remoteio.hpp"
#include <cstdlib>

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RC_ctrl_t rc_ctrl;

/**
 * @brief 内部函数声明
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
void RC_DataHandle(RC_ctrl_t *rc_ctrl);

/**
 * @name REMOTEIO_init
 * @brief 初始化遥控接收缓冲区
 * @param uint8_t *rx1_buf , uint8_t *rx2_buf , uint16_t dma_buf_num DMA缓冲区子字节数 由宏定义给出
 */
void REMOTEIO_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enable idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    //    usart_printf("hh\r\n");
}


/**
 * @name Remote_bsp_IRQHandler
 * @brief 串口接收中断处理函数 在remote.c中由REMOTE_UartIrqHandler()调用
 *        调用sbus_to_rc函数解算
 *
 */
void RemoteIO_IRQHandler(void)
{
    //usart_printf("ok\r\n");
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);
            //usart_printf("%d\r\n",this_time_rx_len);

                if(this_time_rx_len == RC_FRAME_LENGTH_WFLY)
                {
                    sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
                }

        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度q
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH_WFLY)
            {
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }

        }
    }
}



/*
    * @name sbus_to_rc
    * @brief 将串口接收到的数据解算为遥控器的杆量和拨杆位置
    *
*/
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == nullptr || rc_ctrl == NULL)
    {
        return;
    }
    // usart_printf("WFLY\r\n");
    rc_ctrl->rc.ch[0] = (sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[3] = ((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) |          //!< Channel 2
                     (sbus_buf[5] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[2] = ((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.ch[4] = ((sbus_buf[7] << 4) | (sbus_buf[6] >> 4)) & 0x07ff;
    rc_ctrl->rc.ch[5] = ((sbus_buf[9] << 9) | (sbus_buf[8] << 1) | (sbus_buf[7] >> 7)) & 0x07ff;
    rc_ctrl->rc.ch[6] = ((sbus_buf[10] << 6) | (sbus_buf[9] >> 2)) & 0x07ff;
    rc_ctrl->rc.ch[7] = ((sbus_buf[11] << 3) | (sbus_buf[10] >> 5)) & 0x07ff;

    // usart_printf("%d,%d,%d,%d,\r\n",rc_ctrl->rc.ch[0],rc_ctrl->rc.ch[1],rc_ctrl->rc.ch[2],rc_ctrl->rc.ch[3]);


    rc_ctrl->rc.ch[0] -= RC_CH0_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH1_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH2_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH3_VALUE_OFFSET;

    if (rc_ctrl->rc.ch[0] < -800 || rc_ctrl->rc.ch[1] < -800 || rc_ctrl->rc.ch[2] < -800 || rc_ctrl->rc.ch[3] < -800 ||
        rc_ctrl->rc.ch[0] > 800 || rc_ctrl->rc.ch[1] > 800 ||rc_ctrl->rc.ch[2] > 800 || rc_ctrl->rc.ch[3] > 800 )
    {
        rc_ctrl->rc.ch[0] = 0;
        rc_ctrl->rc.ch[1] = 0;
        rc_ctrl->rc.ch[2] = 0;
        rc_ctrl->rc.ch[3] = 0;
    }
    //rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] = -rc_ctrl->rc.ch[2];

    if (rc_ctrl->rc.ch[4] < 500)
    {
        rc_ctrl->rc.s[0] = 1; //最右侧拨杆上置
    }
    else {
        rc_ctrl->rc.s[0] = 2; //最右侧拨杆下置
    }

    if (rc_ctrl->rc.ch[5] < 500)
    {
        rc_ctrl->rc.s[1] = 1;
    }
    else if (rc_ctrl->rc.ch[5] > 500 && rc_ctrl->rc.ch[5] < 1500)
    {
        rc_ctrl->rc.s[1] = 2;
    }
    else {
        rc_ctrl->rc.s[1] = 3;
    }

    if (rc_ctrl->rc.ch[7] < 500)
    {
        rc_ctrl->rc.s[2] = 1; //最左侧拨杆上置
    }
    else {
        rc_ctrl->rc.s[2] = 2; //最左侧拨杆下置
    }

    uint8_t flags = sbus_buf[23];
    rc_ctrl->sbus_status.failsafe   = (flags & SBUS_FLAG_FAILSAFE)   ? 1 : 0;
    rc_ctrl->sbus_status.frame_lost = (flags & SBUS_FLAG_FRAME_LOST) ? 1 : 0;

    // 收到有效帧：重置软件看门狗（无论 failsafe 状态，接收到帧说明链路通畅）
    rc_ctrl->rc_live_cnt_WFLY            = RC_OFFLINE_TIMEOUT;
    rc_ctrl->sbus_status.rc_offline = 0;

    RC_DataHandle(rc_ctrl);

    //打印遥控器原始数据

    // usart_printf("%d, %d, %d, %d, %d, %d, %d\r\n",sbus_buf[0],sbus_buf[1],sbus_buf[2],sbus_buf[3],sbus_buf[4],sbus_buf[5],sbus_buf[6]);

    // 打印遥控器解算后数据

     // usart_printf("%d, %d, %d, %d, %d, %d, %d, %d\r\n",
     // rc_ctrl->rc.ch[0],rc_ctrl->rc.ch[1], rc_ctrl->rc.ch[2], rc_ctrl->rc.ch[3],
     // rc_ctrl->rc.ch[4],rc_ctrl->rc.ch[5], rc_ctrl->rc.ch[6], rc_ctrl->rc.ch[7]);
    /* 分别对应 右横 右竖 左横 左竖 SD SC SB SA */
    /* 前四个值在回中时应该为0（若不是则去修改.h文件中原始数据的回中值），后四个值在回中时应该为1024 */
}

/**
 * @name  REMOTEIO_UpdateStatus
 * @brief 软件看门狗递减函数，需在 1ms 定时中断（SysTick/TIMx）中调用
 *        超过 RC_OFFLINE_TIMEOUT ms 未收到有效帧则置位 rc_offline
 */
void REMOTEIO_UpdateStatus(void)
{
    if (rc_ctrl.rc_live_cnt_WFLY > 0)
    {
        rc_ctrl.rc_live_cnt_WFLY--;
    }
    else
    {
        rc_ctrl.sbus_status.rc_offline = 1;  // 超时，判定离线
    }
}


/**
 * @name  REMOTEIO_IsOffline
 * @brief 查询遥控器是否处于失能状态（断电/超距/Failsafe/超时离线）
 * @retval 1 = 遥控器离线/Failsafe，应执行机构失能
 *         0 = 遥控器正常在线
 */
uint8_t REMOTEIO_IsOffline(void)
{
    return (rc_ctrl.sbus_status.rc_offline || rc_ctrl.sbus_status.failsafe) ? 1 : 0;
}

void RC_DataHandle(RC_ctrl_t *rc_ctrl)  //抑制零漂
{
    if (abs(rc_ctrl->rc.ch[0]) < 10) rc_ctrl->rc.ch[0] = 0;
    if (abs(rc_ctrl->rc.ch[1]) < 10) rc_ctrl->rc.ch[1] = 0;
    if (abs(rc_ctrl->rc.ch[2]) < 10) rc_ctrl->rc.ch[2] = 0;
    if (abs(rc_ctrl->rc.ch[3]) < 10) rc_ctrl->rc.ch[3] = 0;
}

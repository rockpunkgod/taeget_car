/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"


#include "remoteio.hpp"
#include "canio.h"
#include "LK9025.h"
#include "GM6020.h"
#include "vofa_remote.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MF9025_ALL_ID_TEST 0 /* Single connected motor only; 0 restores normal control. */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
static CANIO_Bus_t can1_bus;

/* Diagnostic swap: object/telemetry names are retained for comparison.
 * mf9025_id1 / mf1_* now address physical ID 5 (0x145), sent first.
 * mf9025_id5 / mf5_* now address physical ID 1 (0x141), sent second.
 * This does not change IDs stored in the motors. */
static LK9025_t mf9025_id1;
static LK9025_t mf9025_id5;

/* 使用 GM6020.c 已经定义的 motor[3]，motor[0] 代表 ID 6 */

static LK9025_Telemetry_t mf1_telemetry;
static LK9025_Telemetry_t mf5_telemetry;
static GM6020_Telemetry_t gm6_telemetry;

static volatile HAL_StatusTypeDef mf1_tx_status;
static volatile HAL_StatusTypeDef mf5_tx_status;
static volatile HAL_StatusTypeDef gm6_tx_status;
static volatile int16_t gm6_control_output;
static volatile uint8_t mf_test_last_id;
static volatile uint8_t mf_test_last_command;
static volatile float mf_test_speed_rpm;
static volatile HAL_StatusTypeDef mf_test_tx_status;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void CAN1_Start(void);
static void Update9025FromRemote(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void GM6020_RxRoute(
    const CANIO_Frame_t *frame,
    void *user_data
);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  REMOTEIO_Init(
      sbus_rx_buf[0],
      sbus_rx_buf[1],
      SBUS_RX_BUF_NUM
      );

  CAN1_Start();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

static void CAN1_Start(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  CANIO_Init(&can1_bus, &hcan1);

  if (!LK9025_Init(&mf9025_id1, &can1_bus, 5U) ||
      !LK9025_Init(&mf9025_id5, &can1_bus, 1U))
  {
    Error_Handler();
  }

  GM6020_Init(&motor[0], 6U);
  if (!CANIO_Register(&can1_bus,
                     GM6020_FEEDBACK_BASE_ID + 6U,
                     GM6020_RxRoute,
                     &motor[0]))
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(
          &hcan1,
          CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Update9025FromRemote(void)
{
  uint32_t primask = __get_PRIMASK();
  int16_t speed_input;
  int16_t mode_input;
  uint8_t remote_offline;
  float target_speed_rpm;

  __disable_irq();
  speed_input = rc_ctrl.rc.ch[3];
  mode_input = rc_ctrl.rc.ch[4];
  remote_offline = (uint8_t)(
      (rc_ctrl.sbus_status.rc_offline != 0U) ||
      (rc_ctrl.sbus_status.failsafe != 0U) ||
      (rc_ctrl.sbus_status.frame_lost != 0U));
  __DMB();
  if (primask == 0U)
  {
    __enable_irq();
  }

  /* I4: 352 = protect, 1695 = speed. Allow +/-30 raw counts
   * around the speed detent; all other values select protection. */
  if ((remote_offline != 0U) || (mode_input < 1665) || (mode_input > 1725))
  {
    LK9025_SetTargetSpeed(&mf9025_id1, 0.0f);
    LK9025_SetTargetSpeed(&mf9025_id5, 0.0f);
    LK9025_SetMode(&mf9025_id1, LK9025_MODE_PROTECT);
    LK9025_SetMode(&mf9025_id5, LK9025_MODE_PROTECT);
    return;
  }

  /* RC channel 3 is the left vertical stick, about -800..800. */
  target_speed_rpm = (float)speed_input * 0.1f;
  LK9025_SetTargetSpeed(&mf9025_id1, target_speed_rpm);
  LK9025_SetTargetSpeed(&mf9025_id5, target_speed_rpm);
  LK9025_SetMode(&mf9025_id1, LK9025_MODE_SPEED);
  LK9025_SetMode(&mf9025_id5, LK9025_MODE_SPEED);
}

/* One frame per control tick. Retry a busy mailbox without skipping an ID. */
static void MF9025_TestAllIds(void)
{
  static uint8_t next_id = LK9025_MIN_MOTOR_ID;
  static uint8_t was_enabled;
  static uint8_t run_sweep;
  uint8_t data[8] = {0};
  uint8_t enabled = (mf9025_id1.command_mode == LK9025_MODE_SPEED);
  float speed = mf9025_id1.command_speed_rpm;

  if (enabled != was_enabled) {
    next_id = LK9025_MIN_MOTOR_ID;
    run_sweep = enabled;
    was_enabled = enabled;
  }
  if (speed > 10.0f) { speed = 10.0f; }
  if (speed < -10.0f) { speed = -10.0f; }
  mf_test_speed_rpm = enabled ? speed : 0.0f;

  if (!enabled) {
    data[0] = LK9025_CMD_STOP;
  } else if (run_sweep) {
    data[0] = LK9025_CMD_RUN;
  } else {
    uint32_t raw = (uint32_t)(int32_t)(speed * 600.0f);
    data[0] = LK9025_CMD_SPEED;
    for (uint8_t i = 0U; i < 4U; ++i) {
      data[4U + i] = (uint8_t)(raw >> (8U * i));
    }
  }
  mf_test_last_id = next_id;
  mf_test_last_command = data[0];
  mf_test_tx_status = CANIO_Send(&can1_bus,
      LK9025_CAN_BASE_ID + next_id, data) ? HAL_OK : HAL_ERROR;
  if (mf_test_tx_status == HAL_OK) {
    if (++next_id > LK9025_MAX_MOTOR_ID) {
      next_id = LK9025_MIN_MOTOR_ID;
      run_sweep = 0U;
    }
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 100000;
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_EVEN;
  huart3.Init.Mode = UART_MODE_RX;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void GM6020_RxRoute(
    const CANIO_Frame_t *frame,
    void *user_data)
{
  GM6020_t *gm6020 = (GM6020_t *)user_data;

  if ((frame == NULL) || (gm6020 == NULL)) {
    return;
  }

  GM6020_ParseFeedback(gm6020, frame);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if ((hcan != NULL) && (hcan->Instance == CAN1))
  {
    CANIO_RxIRQ(&can1_bus);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  uint32_t last_vofa_tick = 0U;
  uint32_t last_control_tick = 0U;

  /* USER CODE BEGIN 5 */

  for(;;)
  {
    REMOTEIO_UpdateStatus();
    CANIO_ProcessRx(&can1_bus);
    /* RX timestamps must not be newer than the control snapshot. */
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_control_tick) >= 2U)
    {
      last_control_tick = now;
      Update9025FromRemote();
      if (MF9025_ALL_ID_TEST != 0U) {
        MF9025_TestAllIds();
      } else {
      mf1_tx_status = LK9025_UpdateControl(&mf9025_id1, now);
      mf5_tx_status = LK9025_UpdateControl(&mf9025_id5, now);
      gm6_control_output = GM6020_CalculateControl(&motor[0], now);
      gm6_tx_status = GM6020_SendControl(
          &can1_bus,
          6U,
          gm6_control_output);
      }
    }

    if ((uint32_t)(now - last_vofa_tick) >= 20U)
    {
      last_vofa_tick = now;
      LK9025_GetTelemetry(&mf9025_id1, now, &mf1_telemetry);
      LK9025_GetTelemetry(&mf9025_id5, now, &mf5_telemetry);
      GM6020_GetTelemetry(&motor[0], now, &gm6_telemetry);
      (void)VOFA_Remote_Send();
    }

    osDelay(1);
  }

  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

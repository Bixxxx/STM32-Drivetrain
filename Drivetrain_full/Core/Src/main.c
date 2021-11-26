/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

/* Definitions for StartEncoder */
osThreadId_t StartEncoderHandle;
const osThreadAttr_t StartEncoder_attributes = {
  .name = "StartEncoder",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for StartStepper */
osThreadId_t StartStepperHandle;
const osThreadAttr_t StartStepper_attributes = {
  .name = "StartStepper",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for startThrust */
osThreadId_t startThrustHandle;
const osThreadAttr_t startThrust_attributes = {
  .name = "startThrust",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void ReadEncoder(void *argument);
void ControlStepper(void *argument);
void ControlThrust(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int MANUAL = 0;
int16_t MANUAL_ANGLE_REF = 0;
uint8_t MANUAL_THRUST_REF[1] = {128};


//SPI Transmit and Receive buffers
uint8_t SPI_tx1[2] = {0xFF, 0xFF}; //0xFFFF is a read angle command for the AS5048A magnetic encoder
uint8_t SPI_rx1[2];

uint8_t SPI_tx2[2] = {0xFF, 0xFF}; //0xFFFF is a read angle command for the AS5048A magnetic encoder
uint8_t SPI_rx2[2];


//CAN headers
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

//CAN Transmit and Receive buffers
uint8_t CAN_tx[8];
uint8_t CAN_rx[8];

uint32_t TxMailbox;

//ENCODER 1 VARAIBLES
uint16_t INITIAL_ANGLE_1 = 15092; //15092 raw value
static uint16_t ENCODER_ANGLE_1 = 0;
static float ANGLE_REF_1 = 0;
static int16_t ANGLE_ERROR_1;
float ANGLE_DEGREE_1;

//ENCODER 2 VARIABLES
uint16_t INITIAL_ANGLE_2 = 14706; //14706 raw value
static uint16_t ENCODER_ANGLE_2 = 0;
static float ANGLE_REF_2 = 0;
static int16_t ANGLE_ERROR_2;
float ANGLE_DEGREE_2;

float VOLTAGE_1 = 0;
float VOLTAGE_2 = 0;
float MAX_VOLTAGE = 11.1;

//THRUST UART 1 (left)
static float THRUST_1_REF;
uint8_t  THRUST_1[1] = {128};
//THRUST UART 2 (right)
static float THRUST_2_REF;
uint8_t  THRUST_2[1] = {128};

int8_t THRUST_1_CAN[4] = {0, 0, 0, 0};
int8_t THRUST_2_CAN[4] = {0, 0, 0, 0};
int8_t ANGLE_REF_1_CAN[4] = {0, 0, 0, 0};
int8_t ANGLE_REF_2_CAN[4] = {0, 0, 0, 0};

const float ConvertToDegree = 360.0/16383;

//Used for filtering out bit 14 & 15
uint16_t clearbits = 0x3FFF;

uint16_t PWM_FREQ;
uint16_t PWM_PERIOD = 64000;
uint16_t PWM_DUTY_CYCLE;

int thread1 = 0;
int thread2 = 0;
int thread3 = 0;
int status = 0;

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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_Delay(10);

  // INITIALIZE PIN STATE AS HIGH FOR ENCODER 1
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(10);

  // INITIALIZE PIN STATE AS HIGH FOR ENCODER 2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(10);

  // READ INITAL ANGLE OFFSET ENCODER 1
 /* HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &SPI_tx1[0], 2, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_Receive(&hspi1, &SPI_rx1[0], 2, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  INITIAL_ANGLE_1 = (SPI_rx1[0] << 8 | SPI_rx1[1])&clearbits;
  HAL_Delay(10);

  // READ INITAL ANGLE OFFSET ENCODER 2
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &SPI_tx2[0], 2, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Receive(&hspi2, &SPI_rx2[0], 2, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  INITIAL_ANGLE_2 = (SPI_rx2[0] << 8 | SPI_rx2[1])&clearbits;
*/
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
  /* creation of StartEncoder */
  StartEncoderHandle = osThreadNew(ReadEncoder, NULL, &StartEncoder_attributes);

  /* creation of StartStepper */
  StartStepperHandle = osThreadNew(ControlStepper, NULL, &StartStepper_attributes);

  /* creation of startThrust */
  startThrustHandle = osThreadNew(ControlThrust, NULL, &startThrust_attributes);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  //CAN FILTER CONFIGURATION
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x5<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x0<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF4 PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* CAN RX0 CALLBACK FUNCTION */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_rx);
	if(RxHeader.DLC == 8){
		if(RxHeader.StdId == 0x01){
			THRUST_1_CAN[0] = CAN_rx[0];
			THRUST_1_CAN[1] = CAN_rx[1];
			THRUST_1_CAN[2] = CAN_rx[2];
			THRUST_1_CAN[3] = CAN_rx[3];	//need to change the correct bits for each motor thrust

			//combining bytes into float (Left motor thrust)
			THRUST_1_REF = *(float*)&THRUST_1_CAN; // max = 160, min = -40 N

			THRUST_2_CAN[0] = CAN_rx[4];
			THRUST_2_CAN[1] = CAN_rx[5];
			THRUST_2_CAN[2] = CAN_rx[6];
			THRUST_2_CAN[3] = CAN_rx[7];

			//combining bytes into float (Left motor thrust)
			THRUST_2_REF = *(float*)&THRUST_2_CAN; // max = 160, min = -40 N


		}
		else if(RxHeader.StdId == 0x02){
			ANGLE_REF_1_CAN[0] = CAN_rx[0];
			ANGLE_REF_1_CAN[1] = CAN_rx[1];
			ANGLE_REF_1_CAN[2] = CAN_rx[2];
			ANGLE_REF_1_CAN[3] = CAN_rx[3];

			ANGLE_REF_1 = (*(float*)&ANGLE_REF_1_CAN)*2; // max = 30, min = -30 degree --> +/- 60 degree (stepper gear ratio)

			ANGLE_REF_2_CAN[0] = CAN_rx[4];
			ANGLE_REF_2_CAN[1] = CAN_rx[5];
			ANGLE_REF_2_CAN[2] = CAN_rx[6];
			ANGLE_REF_2_CAN[3] = CAN_rx[7];

			ANGLE_REF_2 = (*(float*)&ANGLE_REF_2_CAN)*2; // max = 30, min = -30 degree --> +/- 60 degree (stepper gear ratio)
		}
		else{
			printf("Wrong message ID, motor thrust and angle unchanged");
		}
		thread1=0;
		thread2=0;
		thread3=0;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ReadEncoder */
/**
  * @brief  Function implementing the StartEncoder thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadEncoder */
void ReadEncoder(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  	  	  	  	  /************************
	  	  	  	  	   * ---- ENCODER 1 ----- *
	  	  	  	  	   ************************/

	// READ ENCODER 1
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //PULL CSn LOW
	HAL_SPI_Transmit(&hspi1, &SPI_tx1[0], 2, 1); //TRANSMIT READ COMMAND(0xFFFF)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //PULL CSn HIGH
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //PULL CSn LOW
	HAL_SPI_Receive(&hspi1, &SPI_rx1[0], 2, 1); //RECEIVE ANGLE READING
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //PULL CSn HIGH

	//CLEAR NON-DATA BITS ENCODER 1
	ENCODER_ANGLE_1 = (SPI_rx1[0] << 8 | SPI_rx1[1])&clearbits; //FILTER OUT BIT 14&15
	ENCODER_ANGLE_1 = (ENCODER_ANGLE_1 - INITIAL_ANGLE_1)&clearbits; //REMOVE INITIAL ANGLE OFFSET

	//RECOMPUTE ANGLE FROM 14-BIT DATA TO -180 TO 180 DEGREE ANGLE FOR ENCODER 1
	ANGLE_DEGREE_1 = ENCODER_ANGLE_1*ConvertToDegree; //CONVERTS 14-bit number to 360 degree
	if(ANGLE_DEGREE_1 > 180){
			ANGLE_DEGREE_1 = ANGLE_DEGREE_1 - 360;
	}
			//TODO:REMOVE COMMENTS
			ANGLE_ERROR_1 = ANGLE_REF_1 - ANGLE_DEGREE_1; //CALCULATE ANGLE ERROR

		 	 	   	   /************************
		 	 	   	    * ---- ENCODER 2 ----- *
		 	 	   	    ************************/

	//READ ENCODER 2
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //PULL CSn LOW
	HAL_SPI_Transmit(&hspi2, &SPI_tx2[0], 2, 1); //TRANSMIT READ COMMAND(0xFF)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //PULL CSn HIGH
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //PULL CSn LOW
	HAL_SPI_Receive(&hspi2, &SPI_rx2[0], 2, 1); //RECEIVE ANGLE READING
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //PULL CSn HIGH

	//CLEAR NON-DATA BITS ENCODER 2
	ENCODER_ANGLE_2 = (SPI_rx2[0] << 8 | SPI_rx2[1])&clearbits; //FILTER OUT BIT 14&15
	ENCODER_ANGLE_2 = (ENCODER_ANGLE_2 - INITIAL_ANGLE_2)&clearbits; //REMOVE INITIAL ANGLE OFFSET

	//RECOMPUTE ANGLE FROM 14-BIT DATA TO -180 TO 180 DEGREE ANGLE FOR ENCODER 2
	ANGLE_DEGREE_2 = ENCODER_ANGLE_2*ConvertToDegree; //CONVERTS 14-bit number to 360 degree
	if(ANGLE_DEGREE_2 > 180){
			ANGLE_DEGREE_2 = ANGLE_DEGREE_2 - 360;
	}
			//TODO: REMOVE COMMENTS
			ANGLE_ERROR_2 = ANGLE_REF_2 - ANGLE_DEGREE_2; //CALCULATE ANGLE ERROR

	thread1++;
    osThreadFlagsWait(0x01,osFlagsWaitAny, osWaitForever); // START ControlStepper thread

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_ControlStepper */
/**
* @brief Function implementing the StartStepper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlStepper */
void ControlStepper(void *argument)
{
  /* USER CODE BEGIN ControlStepper */
  /* Infinite loop */
  for(;;)
  {
	osDelay(1U);
	if(MANUAL == 1){
		ANGLE_ERROR_1 = MANUAL_ANGLE_REF - ANGLE_DEGREE_1;
		ANGLE_ERROR_2 = MANUAL_ANGLE_REF - ANGLE_DEGREE_2;
	}else{
	  	  	  	  	  /************************
	 	  	  	  	   * ---- STEPPER 1 ----- *
	 	  	  	  	   ************************/
	//saturation of reference angles to their maximum/minimum
	if (ANGLE_REF_1>60){ANGLE_REF_1=60;}
	else if(ANGLE_REF_1<-60){ANGLE_REF_1=-60;}
	if (ANGLE_REF_2>60){ANGLE_REF_2=60;}
	else if(ANGLE_REF_2<-60){ANGLE_REF_2=-60;}
	}
	//CHANGE DIRECTION FOR STEPPER 1 DEPENDING ON ERROR SIGN
	if (ANGLE_ERROR_1 > 0){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET);
	}

	ANGLE_ERROR_1 = abs(ANGLE_ERROR_1);
	//UPDATE CCR REGISTER FOR STEPPER 1
	if(ANGLE_ERROR_1 > 0.9){
		TIM1->ARR  = PWM_PERIOD;
		TIM1->CCR3 = PWM_PERIOD/2;
	}
	else{
		TIM1->ARR = PWM_PERIOD;
		TIM1->CCR3 = PWM_PERIOD;

	}
	  	  	  	  	  	  /************************
			  	  	  	   * ---- STEPPER 2 ----- *
			  	  	  	   ************************/

	//CHANGE DIRECTION FOR STEPPER 1 DEPENDING ON ERROR SIGN

	if (ANGLE_ERROR_2 > 0){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
		}
		else{
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
		}
	//UPDATE CCR REGISTER FOR STEPPER 1
	ANGLE_ERROR_2 = abs(ANGLE_ERROR_2);
	if(ANGLE_ERROR_2 > 0.9){
		TIM1->ARR  = PWM_PERIOD;
		TIM1->CCR2 = PWM_PERIOD/2;
		}
		else{
			TIM1->ARR = PWM_PERIOD;
			TIM1->CCR2 = PWM_PERIOD;
		}



	if( (ANGLE_ERROR_1 <= 2) && (ANGLE_ERROR_2 <= 2) ){
		osThreadFlagsSet(startThrustHandle, 0x03); //FLAG THRUST THREAD
		osThreadFlagsWait(0x02, osFlagsWaitAny, osWaitForever); //START THRUST THREAD
	}


	//START ENCODER THREAD
	thread2++;
	osThreadFlagsSet(StartEncoderHandle, 0x01);

  }
  /* USER CODE END ControlStepper */
}

/* USER CODE BEGIN Header_ControlThrust */
/**
* @brief Function implementing the StartThrust thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlThrust */
void ControlThrust(void *argument)
{
  /* USER CODE BEGIN ControlThrust */
HAL_UART_Transmit(&huart4, THRUST_1, sizeof(THRUST_1), 2); //THRUST 1 (left)
HAL_UART_Transmit(&huart5, THRUST_2, sizeof(THRUST_2), 2); //THRUST 2 (right)
  /* Infinite loop */
  for(;;)
  {
	//TIM3 ARR = 65535

	osThreadFlagsWait(0x03,osFlagsWaitAny, osWaitForever); // STOP THREAD

	//IF CAN DOESNT SEND
	/*if(thread3>5000){	//if statement to disable thrust thread if we stop recieving can messages
		THRUST_1[0] = 128;
		THRUST_2[0] = 128;
		HAL_UART_Transmit(&huart4, THRUST_1, sizeof(THRUST_1), 2); //THRUST 1 (left)
		HAL_UART_Transmit(&huart5, THRUST_2, sizeof(THRUST_2), 2); //THRUST 2 (right)
		osDelay(1);
		osThreadFlagsSet(StartStepperHandle, 0x02);
	} */
						   /************************
			  	  	  	    * ---- THRUST 1 ----- *
			  	  	  	    ************************/
	if(MANUAL == 1){
		HAL_UART_Transmit(&huart4, MANUAL_THRUST_REF, sizeof(MANUAL_THRUST_REF), 2); //THRUST 1 (left)
		HAL_UART_Transmit(&huart5, MANUAL_THRUST_REF, sizeof(MANUAL_THRUST_REF), 2); //THRUST 2 (right)
	}else{

	if(THRUST_1_REF > 0){ //scaling from
		VOLTAGE_1 = (-3.44+sqrt(3.44*3.44+4*1.003*THRUST_1_REF))/(2*1.002);
		THRUST_1[0] = 127*(VOLTAGE_1/MAX_VOLTAGE)+128;
  	}
	else if (THRUST_1_REF < 0){
		VOLTAGE_1 = (-0.17+sqrt(0.17*0.17+4*0.302*-1*THRUST_1_REF))/(2*0.302);
		THRUST_1[0] = -127*(VOLTAGE_1/MAX_VOLTAGE) + 128;
	}
	else{
		THRUST_1[0] = 128;
	}
    					   /************************
    			  	  	  	* ---- THRUST 2 ----- *
    			  	  	  	************************/

	if(THRUST_2_REF > 0){ //scaling from
		VOLTAGE_2 = (-3.44+sqrt(3.44*3.44+4*1.003*THRUST_2_REF))/(2*1.002);
		THRUST_2[0] = 127*(VOLTAGE_2/MAX_VOLTAGE)+128;
  	}
	else if (THRUST_2_REF < 0){
		VOLTAGE_2 = (-0.17+sqrt(0.17*0.17+4*0.302*-1*THRUST_2_REF))/(2*0.302);
		THRUST_2[0] = -127*(VOLTAGE_2/MAX_VOLTAGE)+128;
	}
	else{
		THRUST_2[0] = 128;
	}
	HAL_UART_Transmit(&huart4, THRUST_1, sizeof(THRUST_1), 2); //THRUST 1 (left)
	HAL_UART_Transmit(&huart5, THRUST_2, sizeof(THRUST_2), 2); //THRUST 2 (right)
	}


	status++;
	thread3++;
	osDelay(1);
	osThreadFlagsSet(StartStepperHandle, 0x02);


  }
  /* USER CODE END ControlThrust */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

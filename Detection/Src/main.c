/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4s5i_iot01_qspi.h"
#include "stm32l4s5i_iot01.h"
#include "stm32l4s5i_iot01_hsensor.h"
#include "stm32l4s5i_iot01_gyro.h"
#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_tsensor.h"
#include "../Components/lps22hb/lps22hb.c"
#include "../Components/lsm6dsl/lsm6dsl.c"
#include "../Components/lis3mdl/lis3mdl.c"
#include "../Components/hts221/hts221.c"
#include <math.h>
#include <stdio.h>
#include "arm_math.h"
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
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId handle_quakeHandle;
osThreadId handle_tempHandle;
osThreadId handle_speakerHandle;
osThreadId send_terminalHandle;
osSemaphoreId tempLockHandle;
osSemaphoreId gyroLockHandle;
osSemaphoreId tempTerminalLockHandle;
osSemaphoreId gyroTerminalLockHandle;
/* USER CODE BEGIN PV */

#define data_array_size 20
#define wave_size 33075

char buffer[100];
int count = 0;
float humidity;
float gyro[3];
float gyro_x[data_array_size];
float gyro_y[data_array_size];
float gyro_z[data_array_size];

int timeCounter = 0;
int timeCycle = 30;
int timeTaken = 0;

//sound waves
uint8_t G6[28];
uint8_t E7[16];
uint8_t A6[12];

float temp;
int flag = 0;
int temp_warningLevel = 0;//0 no danger, 1 warning, 2 severe, 3 danger.
int quake_warningLevel = 0;
int earthquake = 0;
int mode=0;//mode 0: disable detect, mode 1: detect temp, mode 2: detect earthquake

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DAC1_Init(void);
void earthquakeHandler(void const * argument);
void temperatureHandler(void const * argument);
void speakerHandler(void const * argument);
void sendTerminal(void const * argument);

/* USER CODE BEGIN PFP */

void init_sound_wave();
void write_sound_wave_into_mem();
void init_gyro_record();
void push_data_into_gyro_record();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t play[wave_size]={1};//33075

//each time the blue button is pressed, modify the counter to change the detected sensor.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
	if (GPIO_PIN == GPIO_PIN_13) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		mode=(mode+1)%3;
	}
}

/**
 * Initialize the warning sound wave.
 * Level1: with frequency: G6
 * Level2: with frequency: E7
 * Level3: with frequency: A6
 */
void init_sound_wave(){
	//Tone 1
	//G6 1567.98 Hz
	//sample n = 44.1k/1567.98 = 28
	for(int i = 0; i < 28; i++){
		G6[i] =  0.33*(1 + arm_sin_f32(2*PI*i/28))*256;
	}

	//Tone 2
	//E7 2637.02 Hz
	//sample n = 44.1k/1318.5 = 16
	for(int i = 0; i < 16; i++){
		E7[i] =  0.33*(1 + arm_sin_f32(2*PI*i/16))*256;
	}

	//Tone 3
	//A6 3520.0 Hz
	//sample n = 44.1k/3520 = 12
	for(int i = 0; i < 12; i++){
		A6[i] =  0.33*(1 + arm_sin_f32(2*PI*i/12))*256;
	}
}

/**
 * Write the sound waves into the flash.
 */
void write_sound_wave_into_mem(){
	uint32_t tone_addr = 0x000000;

	//Write in tone1 C6, start at 0
	for(int i = 0; i < 1181; i++){
	  if(BSP_QSPI_Write((uint8_t *)G6, tone_addr, 28) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 28;
	}
	tone_addr = 0x008133;//33075


  //Write in tone2 E6, start at 33075
  for(int i = 0; i < 2067; i++){
	  if(BSP_QSPI_Write((uint8_t *)E7, tone_addr, 16) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 16;
  }
  tone_addr = 0x010266;//66150

  //Write in tone3 G6, start at 66150
  for(int i = 0; i < 2756; i++){
	  if(BSP_QSPI_Write((uint8_t *)A6, tone_addr, 12) != QSPI_OK){
		  Error_Handler();
	  }
	  tone_addr += 12;
  }
}

/**
 * Initialize the gyro sensor data array.
 */
void init_gyro_record(){
	BSP_GYRO_GetXYZ(gyro);
	for(int i=0;i<data_array_size;i++){
		gyro_x[i]=gyro[0];
		gyro_y[i]=gyro[1];
		gyro_z[i]=gyro[2];
	}
}

/**
 * Update the gyro record.
 */
void push_data_into_gyro_record(){
	for(int i=0;i<data_array_size-1;i++){
		gyro_x[i]=gyro_x[i+1];
		gyro_y[i]=gyro_y[i+1];
		gyro_z[i]=gyro_z[i+1];
	}
	gyro_x[data_array_size-1]=gyro[0];
	gyro_y[data_array_size-1]=gyro[1];
	gyro_z[data_array_size-1]=gyro[2];
}
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_OCTOSPI1_Init();
  MX_TIM2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
  //component init
  BSP_QSPI_Init();
  HAL_TIM_Base_Start_IT(&htim2);
  BSP_HSENSOR_Init();
  BSP_GYRO_Init();
  BSP_TSENSOR_Init();
  //private function init
  init_sound_wave();
  init_gyro_record();


  //Erase 3 blocks prior to write in
  if(BSP_QSPI_Erase_Block(0) != QSPI_OK){
	  Error_Handler();
  }
  if(BSP_QSPI_Erase_Block(65536) != QSPI_OK){
	  Error_Handler();
  }
  if(BSP_QSPI_Erase_Block(131072) != QSPI_OK){
	  Error_Handler();
  }

  //After erased the blocks, we can write in the samples of tones
  write_sound_wave_into_mem();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of tempLock */
  osSemaphoreDef(tempLock);
  tempLockHandle = osSemaphoreCreate(osSemaphore(tempLock), 1);

  /* definition and creation of gyroLock */
  osSemaphoreDef(gyroLock);
  gyroLockHandle = osSemaphoreCreate(osSemaphore(gyroLock), 1);

  /* definition and creation of tempTerminalLock */
  osSemaphoreDef(tempTerminalLock);
  tempTerminalLockHandle = osSemaphoreCreate(osSemaphore(tempTerminalLock), 1);

  /* definition and creation of gyroTerminalLock */
  osSemaphoreDef(gyroTerminalLock);
  gyroTerminalLockHandle = osSemaphoreCreate(osSemaphore(gyroTerminalLock), 1);

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
  /* definition and creation of handle_quake */
  osThreadDef(handle_quake, earthquakeHandler, osPriorityHigh, 0, 128);
  handle_quakeHandle = osThreadCreate(osThread(handle_quake), NULL);

  /* definition and creation of handle_temp */
  osThreadDef(handle_temp, temperatureHandler, osPriorityAboveNormal, 0, 128);
  handle_tempHandle = osThreadCreate(osThread(handle_temp), NULL);

  /* definition and creation of handle_speaker */
  osThreadDef(handle_speaker, speakerHandler, osPriorityNormal, 0, 128);
  handle_speakerHandle = osThreadCreate(osThread(handle_speaker), NULL);

  /* definition and creation of send_terminal */
  osThreadDef(send_terminal, sendTerminal, osPriorityNormal, 0, 128);
  send_terminalHandle = osThreadCreate(osThread(send_terminal), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim2);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : blue_button_Pin */
  GPIO_InitStruct.Pin = blue_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blue_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_LED_Pin */
  GPIO_InitStruct.Pin = BLUE_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvHalfCpltCallbackCh1 (DAC_HandleTypeDef * hdac){
//	HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
//	HAL_GPIO_TogglePin(RED_LED_GPIO_Port,RED_LED_Pin);
//	HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);

//	sound_counter = (sound_counter+1) %3;

}

void HAL_DAC_ConvCpltCallbackCh1 (DAC_HandleTypeDef * hdac){

//	switch(counter){
//	    case 0  :
//			if(BSP_QSPI_Read((uint8_t *)play, 0x000000, size) != QSPI_OK) Error_Handler();
//			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, size, DAC_ALIGN_8B_R);
//	       break;
//	    case 1  :
//	    	if(BSP_QSPI_Read((uint8_t *)play, 0x008133, size) != QSPI_OK) Error_Handler();
//	    	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, size, DAC_ALIGN_8B_R);
//	       break;
//	    case 2  :
//	    	if(BSP_QSPI_Read((uint8_t *)play, 0x010266, size) != QSPI_OK) Error_Handler();
//	    	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, size, DAC_ALIGN_8B_R);
//	       break;
//	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_earthquakeHandler */
/**
  * @brief  Function implementing the handle_quake thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_earthquakeHandler */
void earthquakeHandler(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	for(;;)
	{
		osDelay(50);
		BSP_GYRO_GetXYZ(gyro);
		push_data_into_gyro_record();

		float std_x=0;
		float std_y=0;
		float std_z=0;
		arm_std_f32(&gyro_x,10,&std_x);
		arm_std_f32(&gyro_y,10,&std_y);
		arm_std_f32(&gyro_z,10,&std_z);
		float shake = std_x + std_y + std_z;
		if(shake < 30000){
			quake_warningLevel=0;
		}else {
			if(shake > 30000 && shake < 45000){
				quake_warningLevel=1;
			}else if(shake > 45000 && shake < 70000){
				quake_warningLevel=2;
			}else{
				quake_warningLevel=3;
			}
			osSemaphoreRelease(gyroLockHandle);
		}
		osSemaphoreRelease(gyroTerminalLockHandle);
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_temperatureHandler */
/**
* @brief Function implementing the handle_temp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_temperatureHandler */
void temperatureHandler(void const * argument)
{
  /* USER CODE BEGIN temperatureHandler */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(50);
	  temp=BSP_TSENSOR_ReadTemp();

	  if(temp<35){
		  temp_warningLevel=0;
	  }else{
		  if(temp>=35&&temp<37){
			  temp_warningLevel=1;
		  }else if(temp>=37&&temp<42){
			  temp_warningLevel=2;
		  }else{
			  temp_warningLevel=3;
		  }
		  osSemaphoreRelease(tempLockHandle);
	  }

	  osSemaphoreRelease(tempTerminalLockHandle);
  }
  /* USER CODE END temperatureHandler */
}

/* USER CODE BEGIN Header_speakerHandler */
/**
* @brief Function implementing the handle_speaker thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_speakerHandler */
void speakerHandler(void const * argument)
{
  /* USER CODE BEGIN speakerHandler */
  /* Infinite loop */
  for(;;)
  {
//	  osSemaphoreWait(tempLockHandle, osWaitForever);
	  if(osSemaphoreWait(tempLockHandle, osWaitForever)!=osOK || osSemaphoreWait(gyroLockHandle, osWaitForever)!= osOK){
		  continue;
	  }

	osDelay(10);
	int warningLevel = (temp_warningLevel > quake_warningLevel) ? temp_warningLevel : quake_warningLevel;

	if(warningLevel != 0){
		switch(warningLevel){
			case 1  :
				if(BSP_QSPI_Read((uint8_t *)play, 0x000000, wave_size) != QSPI_OK) Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, wave_size, DAC_ALIGN_8B_R);
			   break;
			case 2  :
				if(BSP_QSPI_Read((uint8_t *)play, 0x008133, wave_size) != QSPI_OK) Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, wave_size, DAC_ALIGN_8B_R);
			   break;
			case 3  :
				if(BSP_QSPI_Read((uint8_t *)play, 0x010266, wave_size) != QSPI_OK) Error_Handler();
				HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, play, wave_size, DAC_ALIGN_8B_R);
			   break;
		}
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		HAL_DAC_Stop_DMA (&hdac1, DAC_CHANNEL_1);

		osSemaphoreRelease(tempLockHandle);
	}
  }
  /* USER CODE END speakerHandler */
}

/* USER CODE BEGIN Header_sendTerminal */
/**
* @brief Function implementing the send_terminal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sendTerminal */
void sendTerminal(void const * argument)
{
  /* USER CODE BEGIN sendTerminal */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(tempTerminalLockHandle, 1000)!=osOK || osSemaphoreWait(gyroTerminalLockHandle, 1000)!= osOK){
		  continue;
	  }

	  timeCounter++;
	  osDelay(1);

	  	for (int i = 0; i < 100; i++){
	  		buffer[i] = '\0';
	  	}

	  	if(mode==1){
	  		if(temp<38){
	  			sprintf(buffer, "OK, temperature is %d \r\n", (int) temp);
	  		}else if(temp>=38&&temp<40){
	  			sprintf(buffer, "Warning, temperature is %d !!\r\n", (int) temp);
	  		}else if(temp>=40&&temp<45){
	  			sprintf(buffer, "Danger, temperature is %d !!!!!!\r\n", (int) temp);
	  		}else{
	  			sprintf(buffer, "Extremely danger, temperature is %d !!!!!!!!!!!!!Please leave the house!!!!!!!!!!\r\n", (int) temp);
	  		}
	  	}else if(mode==2){
	  		float std_x=0;
	  		float std_y=0;
	  		float std_z=0;
	  		arm_std_f32(&gyro_x,10,&std_x);
	  		arm_std_f32(&gyro_y,10,&std_y);
	  		arm_std_f32(&gyro_z,10,&std_z);
	  		sprintf(buffer, "Gyro: %d, %d, %d \r\n", (int) std_x, (int) std_y, (int) std_z);
	  	}else{
	  		sprintf(buffer, "Detection disabled now. \r\n");
	  	}

	  	HAL_Delay(100);
	  	if(timeCounter==timeCycle) {
	  		char timeString[30];
	  		sprintf(timeString, "time taken: %d\r\n", timeTaken);
	  		strcat(buffer, timeString);
	  		timeCounter = 0;
	  		timeTaken = 0;
	  	}
	      HAL_UART_Transmit(&huart1, (uint8_t*) buffer, sizeof(buffer), 10000);

  }
  /* USER CODE END sendTerminal */
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
  }else if (htim->Instance == TIM2){
	  //every time 0.1ms
	  timeTaken++;
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

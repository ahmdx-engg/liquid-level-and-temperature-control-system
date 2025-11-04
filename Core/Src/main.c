/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_ll_usart.h"
#include "i2c-lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile float DS18B20_Temp;
int receivedValue;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId TEMPHandle;
osThreadId DISTHandle;
osThreadId COMMHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);
void TEMP_init(void const * argument);
void DIST_init(void const * argument);
void COMM_init(void const * argument);

/* USER CODE BEGIN PFP */
static uint8_t DS18B20_Init(void);
static uint8_t DS18B20_ReadBit(void);
static uint8_t DS18B20_ReadByte(void);
static void DS18B20_WriteByte(uint8_t);
void DS18B20_SampleTemp(void);
float DS18B20_ReadTemp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void delay(uint16_t time)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < time)
    ;
}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0; // is the first value captured ?
uint8_t Distance = 0;

#define TRIG_PIN GPIO_PIN_15
#define TRIG_PORT GPIOB

// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
  {
    if (Is_First_Captured == 0) // if the first value is not captured
    {
      IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
      Is_First_Captured = 1;                                    // set the first captured as true
      // Now change the polarity to falling edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }

    else if (Is_First_Captured == 1) // if the first is already captured
    {
      IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
      __HAL_TIM_SET_COUNTER(htim, 0);                           // reset the counter

      if (IC_Val2 > IC_Val1)
      {
        Difference = IC_Val2 - IC_Val1;
      }

      else if (IC_Val1 > IC_Val2)
      {
        Difference = (0xffff - IC_Val1) + IC_Val2;
      }

      Distance = Difference * .034 / 2;
      Is_First_Captured = 0; // set it back to false

      // set polarity to rising edge
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
      __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
    }
  }
}

void HCSR04_Read(void)
{
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);   // pull the TRIG pin HIGH
  delay(10);                                              // wait for 10 us
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TEMP */
  osThreadDef(TEMP, TEMP_init, osPriorityIdle, 0, 128);
  TEMPHandle = osThreadCreate(osThread(TEMP), NULL);

  /* definition and creation of DIST */
  osThreadDef(DIST, DIST_init, osPriorityNormal, 0, 128);
  DISTHandle = osThreadCreate(osThread(DIST), NULL);

  /* definition and creation of COMM */
  osThreadDef(COMM, COMM_init, osPriorityNormal, 0, 128);
  COMMHandle = osThreadCreate(osThread(COMM), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static uint8_t DS18B20_Init(void)
{
  uint8_t ResetByte = 0xF0, PresenceByte;
  LL_USART_SetBaudRate(huart1.Instance, HAL_RCC_GetPCLK2Freq(), 16, 9600);
  // Send reset pulse (0xF0)
  HAL_UART_Transmit(&huart1, &ResetByte, 1, 1);
  // Wait for the presence pulse
  HAL_UART_Receive(&huart1, &PresenceByte, 1, 1);
  LL_USART_SetBaudRate(huart1.Instance, HAL_RCC_GetPCLK2Freq(), 16, 115200);
  // Check presence pulse
  if (PresenceByte != ResetByte)
  {
    return 1; // Presence pulse detected
  }
  else
  {
    return 0; // No presence pulse detected
  }
}

static uint8_t DS18B20_ReadBit(void)
{
  uint8_t ReadBitCMD = 0xFF;
  uint8_t RxBit;

  // Send Read Bit CMD
  HAL_UART_Transmit(&huart1, &ReadBitCMD, 1, 1);
  // Receive The Bit
  HAL_UART_Receive(&huart1, &RxBit, 1, 1);

  return (RxBit & 0x01);
}

static uint8_t DS18B20_ReadByte(void)
{
  uint8_t RxByte = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    RxByte >>= 1;
    if (DS18B20_ReadBit())
    {
      RxByte |= 0x80;
    }
  }
  return RxByte;
}

static void DS18B20_WriteByte(uint8_t data)
{
  uint8_t TxBuffer[8];
  for (int i = 0; i < 8; i++)
  {
    if ((data & (1 << i)) != 0)
    {
      TxBuffer[i] = 0xFF;
    }
    else
    {
      TxBuffer[i] = 0;
    }
  }
  HAL_UART_Transmit(&huart1, TxBuffer, 8, 10);
}

void DS18B20_SampleTemp(void)
{
  DS18B20_Init();
  DS18B20_WriteByte(0xCC); // Skip ROM   (ROM-CMD)
  DS18B20_WriteByte(0x44); // Convert T  (F-CMD)
}

float DS18B20_ReadTemp(void)
{
  uint8_t Temp_LSB, Temp_MSB;
  uint16_t Temp;
  float Temperature;

  DS18B20_Init();
  DS18B20_WriteByte(0xCC); // Skip ROM         (ROM-CMD)
  DS18B20_WriteByte(0xBE); // Read Scratchpad  (F-CMD)
  Temp_LSB = DS18B20_ReadByte();
  Temp_MSB = DS18B20_ReadByte();
  // Temp_LSB+=48;
  // Temp_MSB+=48;
  Temp = ((Temp_MSB << 8)) | Temp_LSB;
  Temperature = (float)Temp / 16.0;

  return Temperature;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TEMP_init */
/**
* @brief Function implementing the TEMP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TEMP_init */
void TEMP_init(void const * argument)
{
  /* USER CODE BEGIN TEMP_init */
  /* Infinite loop */
  for(;;)
  {
	  DS18B20_SampleTemp();                   // Start temperature conversion
	  DS18B20_Temp = DS18B20_ReadTemp();      // Read the converted temperature value
	  osDelay(1000);
  }
  /* USER CODE END TEMP_init */
}

/* USER CODE BEGIN Header_DIST_init */
/**
* @brief Function implementing the DIST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DIST_init */
void DIST_init(void const * argument)
{
  /* USER CODE BEGIN DIST_init */
  /* Infinite loop */
  for(;;)
  {
	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Toggle GPIO pin for indication
	    HCSR04_Read(); // Trigger the ultrasonic sensor to read distance

	    osDelay(250);
  }
  /* USER CODE END DIST_init */
}

/* USER CODE BEGIN Header_COMM_init */
/**
* @brief Function implementing the COMM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_COMM_init */
void COMM_init(void const * argument)
{
  /* USER CODE BEGIN COMM_init */
  /* Infinite loop */
  for(;;)
  {	      uint8_t buffer[2]; // Buffer to store the received ASCII data

	  if (DS18B20_Temp != -1) // Assuming -1 indicates an invalid reading
	      {
	        // Convert the temperature to an integer and separate digits
	        int displayTemp = (int)DS18B20_Temp;             // Convert temperature to integer
	        char onesTemp = (displayTemp % 10) + '0';        // Extract ones digit as ASCII
	        char tensTemp = ((displayTemp / 10) % 10) + '0'; // Extract tens digit as ASCII

	        // Transmit the temperature data via UART
	        HAL_UART_Transmit(&huart2, (uint8_t *)"Temperature = ", 16, 100);
	        HAL_UART_Transmit(&huart2, (uint8_t *)&tensTemp, 1, 1000); // Transmit tens digit
	        HAL_UART_Transmit(&huart2, (uint8_t *)&onesTemp, 1, 1000); // Transmit ones digit
	        HAL_UART_Transmit(&huart2, (uint8_t *)"\t", 2, 1000);    // Transmit newline
	        HAL_UART_Transmit(&huart2, (uint8_t *)"Received Value = ", 17, HAL_MAX_DELAY);
	        HAL_UART_Transmit(&huart2, (uint8_t *)&buffer[0], 1, 1000); // Transmit tens digit
	        HAL_UART_Transmit(&huart2, (uint8_t *)&buffer[1], 1, 1000); // Transmit ones digit
	        HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY); // Transmit newline


	      }
	      else
	      {
	        HAL_UART_Transmit(&huart2, (uint8_t *)"Invalid Temperature\r\n", 22, 1000);
	      }
	      // Convert distance to integer and separate digits
	      int onesDist = (Distance % 10) + 48;        // Extract ones digit as ASCII
	      int tensDist = ((Distance / 10) % 10) + 48; // Extract tens digit as ASCII
	      int hundredsDist = (Distance / 100) + 48;   // Extract hundreds digit as ASCII

	      // Transmit the distance data via UART
	      HAL_UART_Transmit(&huart2, (uint8_t *)"Distance = ", 12, 100);
	      HAL_UART_Transmit(&huart2, (uint8_t *)&hundredsDist, 1, 100); // Transmit hundreds digit
	      HAL_UART_Transmit(&huart2, (uint8_t *)&tensDist, 1, 100);     // Transmit tens digit
	      HAL_UART_Transmit(&huart2, (uint8_t *)&onesDist, 1, 100);     // Transmit ones digit
	      HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 100);        // Transmit newline
	      if (HAL_UART_Receive(&huart2, buffer, 2, 100) == HAL_OK) {
		       receivedValue = (buffer[0] - '0') * 10 + (buffer[1] - '0');
	      } else {
	          // Handle timeout or error
	    	  receivedValue=0;
	    	    HAL_UART_Transmit(&huart2, (uint8_t *)"Error in receiving data\r\n", 25, 100);

	      }
	      // Convert ASCII bytes to an integer
    osDelay(500);
  }
  /* USER CODE END COMM_init */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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

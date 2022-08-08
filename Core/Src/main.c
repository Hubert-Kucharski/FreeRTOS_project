/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "fonts.h"
#include "st7735.h"
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
ADC_HandleTypeDef hadc3;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
typedef enum
{
	LED_source,
	Button_source,
	ADC_source,
	Time_source
} DataSource_t;	// Enum data types for Queue

typedef struct
{
	unsigned char Data;
	DataSource_t DataSource;
} Data_t;	// Struct type with data and data source to be send in Queue

static Data_t DataStructs[ 4 ] =
{
		{ 0, LED_source },
		{ 0, Button_source },
		{ 0, ADC_source },
		{ 0, Time_source },
};	// 4 struct array members created

QueueHandle_t Queue_handle;
TaskHandle_t LCD_UART_handle;


unsigned char counter;			// button
unsigned char seconds;			// timer
unsigned char startup_flag;		// startup flag for timer


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); // button ISR
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); // Timer ISR
void ADC(void *pvParameters);	// Task (P1)
void LED(void *pvParameters);	// Task (P1)
void LCD_UART(void *pvParameters);	// Task (P2)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		seconds++;
		if(seconds >= 5 && startup_flag == 0)
			startup_flag = 1;
		if(seconds >= 100)
			seconds = 0;
		if(startup_flag)
		{
			DataStructs->DataSource = Time_source;
			DataStructs->Data = seconds;

			xQueueSendToBackFromISR(Queue_handle,&DataStructs,NULL);
			vTaskNotifyGiveFromISR(LCD_UART_handle,NULL);
		}
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	// button ISR
{
	counter++;
	if(counter >= 100)  // count only to two digit value
		counter = 0;

	DataStructs->DataSource = Button_source;
	DataStructs->Data = counter;

	xQueueSendToBackFromISR(Queue_handle,&DataStructs,NULL);
	vTaskNotifyGiveFromISR(LCD_UART_handle,NULL);
}
void ADC(void *pvParameters) // Task
{
	unsigned char adc_value;

	for( ;; )
	{
		adc_value = HAL_ADC_GetValue(&hadc3);

		DataStructs->DataSource = ADC_source;
		DataStructs->Data = adc_value;

		HAL_ADC_Start(&hadc3);

		xQueueSendToBackFromISR(Queue_handle,&DataStructs,0);

		xTaskNotifyGive(LCD_UART_handle);
		vTaskDelay(pdMS_TO_TICKS(500));		// wait for 500 ms
	}
}
void LED(void *pvParameters)	// Task
{
	unsigned char led_state;

	for( ;; )
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0); 	// LED toggle
		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0))	// check LED state
			led_state = 1;
		else
			led_state = 0;

		DataStructs->DataSource = LED_source;
		DataStructs->Data = led_state;

		xQueueSendToBack(Queue_handle,&DataStructs,0);

		xTaskNotifyGive(LCD_UART_handle);
		vTaskDelay(pdMS_TO_TICKS(250));		// wait for 250 ms
	}
}
void LCD_UART(void *pvParameters)	// Task
{

	for(;;)
	{
		xQueueReceive(Queue_handle, &DataStructs, pdMS_TO_TICKS(500));


		if(DataStructs->DataSource == LED_source)
		{
			if(DataStructs->Data == GPIO_PIN_SET)
			{
				// Show LED state on LCD and PC terminal:
				ST7735_DrawString(6, 1, "ON ", ST7735_BLACK);
				HAL_UART_Transmit(&hlpuart1, (unsigned char*)"LED: ON\n", sizeof("LED: ON\n"), 50);
			}
			else
			{
				// Show LED state on LCD and PC terminal:
				ST7735_DrawString(6, 1, "OFF", ST7735_BLACK);
				HAL_UART_Transmit(&hlpuart1, (unsigned char*)"LED: OFF\n", sizeof("LED: OFF\n"), 50);
			}
		}
		else if(DataStructs->DataSource == Button_source)
		{
			char buffer[3];
			unsigned char counter;

			// Get data from queue struct:
			counter = DataStructs->Data;
			sprintf(buffer, "%d", counter);

			// Show button presses count on LCD and PC terminal:
			ST7735_DrawString(6, 3, buffer, ST7735_BLACK);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"Button count: ", sizeof("Button count: "), 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)buffer, 3, 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"\n", sizeof("\n"), 50);
		}
		else if(DataStructs->DataSource == ADC_source)
		{
			char buffer[5];
		    unsigned char adc_value,volt,volt_decimal;

		    // Get data from queue struct:
		    adc_value = DataStructs->Data;
		    // Change ADC value to buffer:
		    volt = adc_value * 0.01;
			volt_decimal = adc_value % 100;
			sprintf(buffer, "%d.%d",volt,volt_decimal);

			// Show voltage on LCD and PC terminal:
			ST7735_DrawString(6, 5, buffer, ST7735_BLACK);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"ADC value: ", sizeof("ADC value: "), 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)buffer, 5, 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"\n", sizeof("\n"), 50);
		}

		else if(DataStructs->DataSource == Time_source)
		{
			char buffer[3];
			unsigned char seconds;

			// Get data from queue struct:
			seconds = DataStructs->Data;
			sprintf(buffer, "%d", seconds);

			// Show time on LCD and PC terminal:
			ST7735_DrawString(6, 7, buffer, ST7735_BLACK);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"Time: ", sizeof("Time: "), 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)buffer, 3, 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"\n", sizeof("\n"), 50);
		}
		else
		{
			// In case error happens show that on PC terminal:
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"ERROR", sizeof("ERROR"), 50);
			HAL_UART_Transmit(&hlpuart1, (unsigned char*)"\n", sizeof("\n"), 50);
		}
		// Task Notification working as a counting semaphore,
		// waiting indefinitely:
		ulTaskNotifyTake( 0, pdMS_TO_TICKS(portMAX_DELAY));
	}
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
  MX_ADC3_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  ST7735_Init();									// LCD initialization
  ST7735_FillScreen(ST7735_WHITE);					// White background
  ST7735_DrawString(1, 1, "LED:", ST7735_BLACK);	// Text on LCD
  ST7735_DrawString(1, 3, "CNT:", ST7735_BLACK);
  ST7735_DrawString(1, 5, "ADC:", ST7735_BLACK);
  ST7735_DrawString(1, 7, "TIM:", ST7735_BLACK);
  HAL_TIM_Base_Start_IT(&htim3);					// Start timer

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

  Queue_handle = xQueueCreate( 3, sizeof( Data_t ) );

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  // External LED switching:
  xTaskCreate(LED, "LED", 100, &Queue_handle, 1, NULL);
  // ADC value reading:
  xTaskCreate(ADC, "ADC", 100, &Queue_handle, 1, NULL);
  //LCD refreshing with new values:
  xTaskCreate(LCD_UART, "LCD", 100, &Queue_handle, 2, &LCD_UART_handle);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_8B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

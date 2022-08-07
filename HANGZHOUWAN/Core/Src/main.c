/* USER CODE BEGIN Header */
/**

  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t distance1_received[255];
uint8_t distance1_buff[9];
uint8_t distance1[4];//用字符串表示�?????4位距离（cm�?????
uint8_t distance1_updated=0;
uint8_t sensor_ID1[4]=" s1:";
uint8_t distance1_str[4];
uint8_t distance2_received[255];
uint8_t distance2_buff[9];
uint8_t distance2[4];//用字符串表示�?????4位距离（cm�?????
uint8_t distance2_updated=0;
uint8_t sensor_ID2[4]=" s2:";
uint8_t distance2_str[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void distance_data_process(uint8_t* data, uint8_t* sensor_ID);
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

  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();

  MX_USART3_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
   HAL_UART_Receive_DMA(&huart1, distance1_received, 255);

  //__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  //HAL_UART_Receive_DMA(&huart3, distance2_received, 255);
  char words[10]="hello";
  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	// 停止本次DMA传输
	HAL_UART_DMAStop(huart);
	// 计算接收到的数据长度
	uint8_t data_length  = 255 - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
	/*if(distance1_updated==1)
	{
		memset(distance1_received,1,255);
		// 重启�?????始DMA传输 每次255字节数据
		HAL_UART_Receive_DMA(&huart1, distance1_received, 255);
		return;
	}*/
	uint8_t* this_buff;
	uint8_t* this_received;
	uint8_t* next_received;
	uint8_t* this_ID;
	uint8_t* is_updated;
	if(huart==&huart1)
	{
		this_buff=distance1_buff;
		this_ID=sensor_ID1;
		this_received=distance1_received;
		next_received=distance2_received;
		is_updated=&distance1_updated;
	}
	else if(huart==&huart3)
	{
		this_buff=distance2_buff;
		this_ID=sensor_ID2;
		this_received=distance2_received;
		next_received=distance1_received;
		is_updated=&distance2_updated;
	}
	HAL_UART_Transmit(&huart2, this_ID, 4, 0x200);
	if(*is_updated==0)
	{
		if((data_length==9))
			{
				memcpy(this_buff, this_received,9);
				*is_updated=1;
				HAL_UART_Transmit(&huart2, this_ID, 4, 0x200);
				distance_data_process(this_buff, this_ID);
			}
			else
			{
				HAL_UART_Receive_DMA(huart, (uint8_t*)this_received, 255);
				return;
			}
	}

	memset(this_received, 255, 255);
	__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(huart, (uint8_t*)this_received, 255);
	huart=((huart==&huart1)?&huart2:&huart1);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

	//HAL_UART_Receive_DMA(huart, (uint8_t*)next_received, 255);

	/*
	if(data_length==9)
	{
		memcpy(distance1_buff, distance1_received, 9);
		distance_data_process(distance1_buff, sensor_ID1);
		uint8_t zero=255;
		//HAL_UART_Transmit(&huart2, &zero, 1, 0x200);
		//HAL_UART_Transmit(&huart2, distance1_received, data_length, 0x200);
		distance1_updated=0;
	}
	// 重启�?????始DMA传输 每次255字节数据
	memset(distance1_received,255,255);
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)distance1_received, 255);
	*/
}



void distance_data_process(uint8_t* data, uint8_t* sensor_ID)
{
	uint16_t distance=0;
	uint8_t distance_str[4]="9999";
	uint8_t endl[2]="\r\n";
	uint8_t failed[6]="failed";
	distance+=*(data+3);
	distance*=256;
	distance+=*(data+2);
	if(distance!=0xffff)
	{
		for(int8_t i=4; i>0; i--)
		{
			distance_str[i-1]=(distance%10)+48;
			distance/=10;
		}
	}
	if(sensor_ID==sensor_ID1)
	{
		memcpy(distance1_str, distance_str, 4);
	}
	else if(sensor_ID==sensor_ID2)
	{
		memcpy(distance2_str, distance_str, 4);
	}

	if(distance1_updated*distance2_updated!=0)
	{
		distance1_updated=0;
		distance2_updated=0;
		HAL_UART_Transmit(&huart2, sensor_ID1, 4, 0x200);
		HAL_UART_Transmit(&huart2, distance1_str, 4, 0x200);
		HAL_UART_Transmit(&huart2, endl, 2, 0x200);
		HAL_UART_Transmit(&huart2, sensor_ID2, 4, 0x200);
		HAL_UART_Transmit(&huart2, distance2_str, 4, 0x200);
	}
	else
	{
		//HAL_UART_Transmit(&huart2, failed, 6, 0x200);
	}

}
/* USER CODE END 4 */

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

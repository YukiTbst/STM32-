/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMP280.h"
#include "stdio.h"
#include "front.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
//此处用的是USART2
//你在使用时仅仅需要把’USART2‘改成你的串口就可以�?????????????
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (uint8_t) ch;
    return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
double pressure_temperature[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void oled_display_test();
void oled_cmd(uint8_t cmd);
void oled_data(uint8_t data);
void oled_init();
void temperature_display_test();
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t text[]="12345";
  HAL_UART_Transmit(&huart1, text, 5, 1000);
  printf("start!\r\n");

  Bmp280 sensor1;
  Bmp280_init(&hspi1, CS_GPIO_Port, CS_Pin, 16, 2, 16, &sensor1);
  //Bmp280 sensor1(&hspi1, CS_GPIO_Port, CS_Pin, 16, 1, 2);
  printf("Initiate complete!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  oled_init();
  oled_display_test();

  while (1)
  {
	  //sensor1.pressure_and_temperature_read(pressure_temperature);
	  pressure_and_temperature_read(pressure_temperature, &sensor1);
	  printf("pressure: %.3f, temperature: %.3f\r\n", pressure_temperature[0], pressure_temperature[1]);
	  temperature_display_test();
	  HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OLED_RESET_Pin|OLED_DorC_Pin|CS_Pin|CS_OLED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : OLED_RESET_Pin */
  GPIO_InitStruct.Pin = OLED_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(OLED_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_DorC_Pin CS_Pin CS_OLED_Pin */
  GPIO_InitStruct.Pin = OLED_DorC_Pin|CS_Pin|CS_OLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void oled_data(uint8_t data)
{
	HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OLED_DorC_GPIO_Port, OLED_DorC_Pin, GPIO_PIN_SET);
	HAL_SPI_Transmit(&hspi1, &data, 1, 100);
	HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);
}
void oled_cmd(uint8_t cmd)
{
	HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OLED_DorC_GPIO_Port, OLED_DorC_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
	HAL_GPIO_WritePin(CS_OLED_GPIO_Port, CS_OLED_Pin, GPIO_PIN_SET);

}
void oled_init()
{
	HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(300);
	HAL_GPIO_WritePin(OLED_RESET_GPIO_Port, OLED_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(30);
	uint8_t cmd[]={0xae, 0xd5, 0x80, 0x20,
	0x02, 0xa8, 0x3f, 0xda,
	0x12, 0xa1, 0xc8, 0x40,
	0xd3, 0x00, 0x81, 0xff,
	0xd9, 0xf1, 0xdb, 0x20,
	0x8d, 0x14, 0xa4, 0xa6,
	0xaf};
	for(uint8_t i=0;i<25;i++)
	{
		oled_cmd(cmd[i]);
	}
}
void oled_display_test()
{
	for(uint8_t i=0;i<8;i++)
		{
			oled_cmd(0xb0|i);
			oled_cmd(0x00);
			oled_cmd(0x10);
			for(uint8_t j=0;j<128;j++)
			{
				oled_data(0);
			}
		}
	uint8_t text1[3]={qi,wen, maohao};
	uint8_t start_row=0xb2;
	for(uint8_t i=0; i<3; i++)
	{
		oled_cmd(start_row);
		oled_cmd(0x00+((i*16)&0x0f));
		oled_cmd(0x10+(((i*16)&0xf0)>>4));
		for(uint8_t j=0; j<16;j++)
		{
			oled_data(words[text1[i]*16+j]);
		}
		oled_cmd(start_row+1);
		oled_cmd(0x00+((i*16)&0x0f));
		oled_cmd(0x10+(((i*16)&0xf0)>>4));
		for(uint8_t j=0; j<16;j++)
		{
			oled_data(words[text1[i]*16+j+16]);
		}
	}
	uint8_t text2[3]={qi,ya, maohao};
	start_row=0xb4;
	for(uint8_t i=0; i<3; i++)
	{
		oled_cmd(start_row);
		oled_cmd(0x00+((i*16)&0x0f));
		oled_cmd(0x10+(((i*16)&0xf0)>>4));
		for(uint8_t j=0; j<16;j++)
		{
			oled_data(words[text2[i]*16+j]);
		}
		oled_cmd(start_row+1);
		oled_cmd(0x00+((i*16)&0x0f));
		oled_cmd(0x10+(((i*16)&0xf0)>>4));
		for(uint8_t j=0; j<16;j++)
		{
			oled_data(words[text2[i]*16+j+16]);
		}
	}
	/*
	for(uint8_t i=0;i<8;i++)
	{
		oled_cmd(0xb0|i);
		oled_cmd(0x00);
		oled_cmd(0x10);
		for(uint8_t j=0;j<128;j++)
		{
			if(j<i*8)
			{
				oled_data(0xff);
			}
			else if(j>i*8+64)
			{
				oled_data(0xff);
			}
			else
			{
				oled_data(0);
			}

		}
	}
	oled_cmd(0x26);
	oled_cmd(0x00);
	oled_cmd(0x00);
	oled_cmd(0x00);
	oled_cmd(0x07);
	oled_cmd(0x2f);
	*/
}
void temperature_display_test()
{

	uint8_t start_row=0xb2;
	uint8_t start_col=4*16;
	uint8_t temp[4];
	uint16_t temperature=(int)(pressure_temperature[1]*100);
	for(int8_t i=3;i>=0;i--)
	{
		temp[i]=temperature%10;
		temperature=(temperature-temp[i])/10;
	}
	uint8_t text1[]={num[temp[0]],num[temp[1]],dot,num[temp[2]],num[temp[3]]};
	for(uint8_t i=0; i<5; i++)
	{
		oled_cmd(start_row);
		oled_cmd(0x00+((i*8+start_col)&0x0f));
		oled_cmd(0x10+(((i*8+start_col)&0xf0)>>4));
		for(uint8_t j=0; j<8;j++)
		{

			oled_data(words[text1[i]*16+j]);
		}
		oled_cmd(start_row+1);
		oled_cmd(0x00+((i*8+start_col)&0x0f));
		oled_cmd(0x10+(((i*8+start_col)&0xf0)>>4));
		for(uint8_t j=0; j<8;j++)
		{

			oled_data(words[text1[i]*16+j+8]);
		}
	}
	uint8_t press[6];
	uint32_t pressure=(int)pressure_temperature[0];
	for(int8_t i=5; i>=0; i--)
	{
		press[i]=pressure%10;
		pressure=(pressure-press[i])/10;
	}
	uint8_t text2[]={num[press[0]],num[press[1]],num[press[2]],num[press[3]],num[press[4]],num[press[5]]};
	start_row=0xb4;
	for(uint8_t i=0; i<6; i++)
	{
		oled_cmd(start_row);
		oled_cmd(0x00+((i*8+start_col)&0x0f));
		oled_cmd(0x10+(((i*8+start_col)&0xf0)>>4));
		for(uint8_t j=0; j<8;j++)
		{
			oled_data(words[text2[i]*16+j]);
		}
		oled_cmd(start_row+1);
		oled_cmd(0x00+((i*8+start_col)&0x0f));
		oled_cmd(0x10+(((i*8+start_col)&0xf0)>>4));
		for(uint8_t j=0; j<8;j++)
		{
			oled_data(words[text2[i]*16+j+8]);
		}
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


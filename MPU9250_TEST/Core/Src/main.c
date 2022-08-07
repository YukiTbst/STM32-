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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
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
#define MPU9250_RA_INT_ENABLE       0x38
#define MPU9250_RA_PWR_MGMT_1       0x6B
#define MPU9250_RA_GYRO_CONFIG      0x1B
#define MPU9250_RA_ACCEL_CONFIG     0x1C
#define MPU9250_RA_CONFIG           0x1A
#define MPU9250_RA_SMPLRT_DIV       0x19
#define MPU9250_RA_INT_PIN_CFG      0x37

//设置低通滤波
#define MPU9250_DLPF_BW_256         0x00
#define MPU9250_DLPF_BW_188         0x01
#define MPU9250_DLPF_BW_98          0x02
#define MPU9250_DLPF_BW_42          0x03
#define MPU9250_DLPF_BW_20          0x04
#define MPU9250_DLPF_BW_10          0x05
#define MPU9250_DLPF_BW_5           0x06
uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data);
uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf);
uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t HALIIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
  uint8_t  *pData;
  pData = &data;
  return HAL_I2C_Mem_Write(&hi2c1, I2C_Addr, reg, I2C_MEMADD_SIZE_8BIT, pData, 1, 100);
}
uint8_t HALIIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
  return HAL_I2C_Mem_Read(&hi2c1, I2C_Addr, reg, I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
}

uint8_t HALIIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
  return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, data, length, 200);
}
uint8_t HALIIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
  return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, data, length, 200);
}

void MPU9250_Init(void)
{
	MPU9250_Check(); //通过读取ID，检查MPU9250是否连接

	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x80); //复位MPU9250
	HAL_Delay(100);
	MPU9250_WriteByte(MPU9250_RA_PWR_MGMT_1, 0x00); //唤醒MPU9250，并选择陀螺仪x轴PLL为时钟源 (MPU9250_RA_PWR_MGMT_1, 0x01)
	MPU9250_WriteByte(MPU9250_RA_INT_ENABLE, 0x00); //禁止中断
	MPU9250_WriteByte(MPU9250_RA_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒
	MPU9250_WriteByte(MPU9250_RA_ACCEL_CONFIG, 0x08); //加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8196LSB/g )
	MPU9250_WriteByte(MPU9250_RA_CONFIG, MPU6050_DLPF_BW_20);//设置陀螺的输出为1kHZ,DLPF=20Hz
	MPU9250_WriteByte(MPU9250_RA_SMPLRT_DIV, 0x00);  //采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
	MPU9250_WriteByte(MPU9250_RA_INT_PIN_CFG, 0x02); //MPU 可直接访问MPU9250辅助I2C

}
#define MPU9250_RA_ACCEL_XOUT_H     0x3B

#define MPU9250_RA_TEMP_OUT_H       0x41

#define MPU9250_RA_GYRO_XOUT_H      0x43

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
#define AK8963_ADDR			0X0C	//AK8963的I2C地址
#define AK8963_ID			0X48	//AK8963的器件ID
//AK8963的内部寄存器
#define MAG_WIA				0x00	//AK8963的器件ID寄存器地址
#define MAG_XOUT_L			0X03

/******************************************************************************
* 功  能：读取加速度的原始数据
* 参  数：*accData 原始数据的指针
* 返回值：无
*******************************************************************************/
void MPU9250_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU9250_ReadMultBytes(MPU9250_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

/******************************************************************************
* 功  能：读取陀螺仪的原始数据
* 参  数：*gyroData 原始数据的指针
* 返回值：无
*******************************************************************************/
void MPU9250_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
	MPU9250_ReadMultBytes(MPU9250_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

/******************************************************************************
* 功  能：读取磁力计的原始数据
* 参  数：*magData原始数据的指针
* 返回值：无
*******************************************************************************/
void MPU9250_MagRead(int16_t *magData)
{
    uint8_t buf[6];
    HALIIC_WriteByteToSlave(MPU9250Addr,0x37,0x02);//turn on Bypass Mode
    HAL_Delay(10);
    HALIIC_WriteByteToSlave(AK8963_MAG_ADDRESS,0x0A,0x11);
    HAL_Delay(10);

    HALIIC_ReadMultByteFromSlave(AK8963_MAG_ADDRESS,MAG_XOUT_L, 6, buf);
    magData[0] = (int16_t)((buf[1] << 8) | buf[0]) ;
    magData[1] = (int16_t)((buf[3] << 8) | buf[2]) ;
    magData[2] = (int16_t)((buf[5] << 8) | buf[4]) ;
}

/******************************************************************************
* 功  能：温度值读取
* 参  数：*tempdata 温度数据的指针
* 返回值：无
*******************************************************************************/
void MPU9250_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU9250_ReadMultBytes(MPU9250_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;

}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char text[100]="hello world";
  uint8_t recv = 0x01;
	uint8_t i2c_err;
	//i2c_err = HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x75, I2C_MEMADD_SIZE_8BIT, &recv, 1, 0xfff);
	int16_t acc[3],gyy[3],mag[3];

  while (1)
  {
	  MPU9250_AccRead(acc);
	  	MPU9250_GyroRead(gyy);
	  	MPU9250_MagRead(mag);
	  	recv=acc[0]%0xff;

	  HAL_UART_Transmit(&huart1, &recv, 1, 100);
	  /*
	  i2c_err = HAL_I2C_Mem_Read(&hi2c1, (0x68<<1), 0x75, I2C_MEMADD_SIZE_8BIT, &recv, 1, 0xfff);

	  if(recv == 0x71)
	  	{
		  HAL_UART_Transmit(&huart1, text, 15, 100);
	  	}
	  else
	  {
		  HAL_UART_Transmit(&huart1, &i2c_err, 1, 100);
	  }
	  */
	  //HAL_UART_Transmit(&huart1, text, 15, 100);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
uint8_t MPU9250_WriteByte(uint8_t reg,uint8_t data)
{
	if(HALIIC_WriteByteToSlave(0xd0,reg,data))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* 功  能：从指定MPU6050寄存器读取一个字节数据
* 参  数：reg： 寄存器地址
*         buf:  读取数据存放的地址
* 返回值：1失败 0成功
*****************************************************************************/
uint8_t MPU9250_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(HALIIC_ReadByteFromSlave(0xd0,reg,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* 功  能：从指定寄存器写入指定长度数据
* 参  数：reg：寄存器地址
*         len：写入数据长度
*         buf: 写入数据存放的地址
* 返回值：0成功 1失败
*****************************************************************************/
uint8_t MPU9250_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_WriteMultByteToSlave(0xd0,reg,len,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* 功  能：从指定寄存器读取指定长度数据
* 参  数：reg：寄存器地址
*         len：读取数据长度
*         buf: 读取数据存放的地址
* 返回值：0成功 0失败
*****************************************************************************/
uint8_t MPU9250_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(HALIIC_ReadMultByteFromSlave(0xd0,reg,len,buf))
		return 1;
	else
		return 0;
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


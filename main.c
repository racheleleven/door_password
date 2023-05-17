/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void LCD_init(void);
void LCD(uint8_t val_1, uint8_t cmd);
void LCD_str(unsigned char *str1);

unsigned char key_press(void);
unsigned char check_clm(unsigned char pin_var);          //FUNCTION DECLARATION
void Disp_pass_key(unsigned char key_var);

void stepper_forward(unsigned int steps);
void stepper_reverse(unsigned int steps);
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
	unsigned char var1;
	unsigned char arr[] = "Welcome";                  //LOCAL VARIABLES
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
  /* USER CODE BEGIN 2 */
  	LCD_init();
  	LCD_str(&arr[0]);
  	HAL_Delay(1000);
  	LCD(0x80, 0);                              //FUNCTION INITIALIZATION
  	LCD_str("Enter password");
  	LCD(0xC0, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);      //LED ON
	  HAL_Delay(150);
	  var1 = key_press();                                                 //MAIN LOOP
	  Disp_pass_key(var1);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);    //LED OF
	  HAL_Delay(150);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin
                           PA6 PA7 PA8 PA9
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LCD_init(void)
{
	LCD(0x38, 0);
	LCD(0x0E, 0);
	LCD(0x06, 0);                     //LCD INITIALIZATION FUNCTION
	LCD(0x01, 0);
	LCD(0x80, 0);
}

void LCD(uint8_t val_1, uint8_t cmd)
{
	uint8_t data1;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, cmd);

	data1 = val_1 & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, data1);

	data1 = (val_1>>1) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, data1);

	data1 = (val_1>>2) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, data1);

	data1 = (val_1>>3) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, data1);          //LCD DISPLAY FUNCTION

	data1 = (val_1>>4) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, data1);

	data1 = (val_1>>5) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, data1);

	data1 = (val_1>>6) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, data1);

	data1 = (val_1>>7) & 0x01;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, data1);


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(1);                                               //ENABLE PIN
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

void LCD_str(unsigned char *str1)
{
	while(*str1 != 0)
	{
		LCD(*str1, 1);                    //LCD STRING DISPLAY FUNCTION
		str1++;
	}
}

unsigned char key_press(void)
{
	unsigned char clm_var;
	unsigned char key_arr[] = {7, 8, 9, 15, 4, 5, 6, 14, 1, 2, 3, 13, 10, 0, 11, 12};
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == 0)
	{
		clm_var = check_clm(GPIO_PIN_0);
		return key_arr[clm_var-1];
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0)
	{
		clm_var = check_clm(GPIO_PIN_1);
		return key_arr[clm_var+3];
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == 0)             //KEYPAD FUNCTION
	{                                                             //CHECK ROW DATA PIN
		clm_var = check_clm(GPIO_PIN_3);
		return key_arr[clm_var+7];
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 0)
	{
		clm_var = check_clm(GPIO_PIN_4);
		return key_arr[clm_var+11];
	}
	else
		return 16;
}

unsigned char check_clm(unsigned char pin_var)
{
	unsigned char clm;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOB, pin_var) == 0)
		clm = 1;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOB, pin_var) == 0)
		clm = 2;
                                                                  //CHECK COL FUNCTION
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);           //CHECK COLUMNS CONNECTIONS
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOB, pin_var) == 0)
		clm = 3;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	if(HAL_GPIO_ReadPin(GPIOB, pin_var) == 0)
		clm = 4;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
	return clm;
}

void Disp_pass_key(unsigned char key_var)
{
	static unsigned char pass_var1 = 0;
	static unsigned char key_arr[4];
	unsigned char enter_key[4] = {"1245"};
	unsigned char i;

	if(key_var != 16)
	{
		key_arr[pass_var1] = key_var + 48;
		LCD(key_var + 48, 1);
		pass_var1++;
		if(pass_var1 == 4)
		{
			LCD(0xC0, 0);
			for(i=0; i<4; i++)
			{
				if(key_arr[i] != enter_key[i])
				{                                            //DISPLAY PRESSED KEY FUNCTION
					LCD_str("Re-enter pw");
					HAL_Delay(100);
					pass_var1 = 0;
					break;
				}
			}
			if(i == 4)
			{
				LCD_str("Welcome!!!");
				stepper_forward(10);
				HAL_Delay(100);
				stepper_reverse(10);
			}
			pass_var1 = 0;
			LCD(0x01, 0);
			LCD(0x80, 0);
			LCD_str("Enter Password:");
			LCD(0xC0, 0);
		}

	}
}

void stepper_forward(unsigned int steps)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	for(steps; steps!=0; steps--)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_Delay(20);                                                 //MOTOR CLOCKWİSE FUNCTION
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_Delay(20);
	}
}

void stepper_reverse(unsigned int steps)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

	for(steps; steps!=0; steps--)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_Delay(20);                                             //MOTOR COUNTER-CLOCKWİSE FUNCTION
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_Delay(20);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_Delay(20);
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

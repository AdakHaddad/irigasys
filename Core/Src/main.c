/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "esp8266.h"
#include "string.h"
#include "stdio.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  // Test basic UART communication first
  HAL_Delay(1000);  // Wait for UART to stabilize
  uint8_t testMsg[] = "STM32 Boot Complete - UART Working!\r\n";
  HAL_UART_Transmit(&huart2, testMsg, strlen((char*)testMsg), 1000);
  // Blink LED to indicate STM32 is running
  for(int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);    // LED ON
    HAL_Delay(200);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  // LED OFF
    HAL_Delay(200);
  }
  uint8_t rxBuffer[512] = {0};
  uint8_t ATisOK;
  int channel;
  int onoff;
  int led = 1;
  char ATcommand[64];
  char ATcommandB[1024];
  char ATcommandN[100];
  char ATcommandF[100];
  char ATcommandT[16];
  sprintf(ATcommandB,"<!DOCTYPE html><html>\n<head>\n\
  <title>STM32 - ESP8266</title>\n<link href=\"data:image/x-icon;base64,\
  A\" rel=\"icon\" type=\"image/x-icon\"><style>\nhtml {\
  display: inline-block; margin: 0px auto; text-align: center;}\n\
  body{margin-top: 50px;}\n.button {display: block;\n\
  width: 70px;\nbackground-color: #008000;\nborder: none;\ncolor: white;\n\
  padding: 14px 28px;\ntext-decoration: none;\nfont-size: 24px;\n\
  margin: 0px auto 36px; \nborder-radius: 5px;}\n\
  .button-on {background-color: #008000;}\n.button-on:active\
  {background-color: #008000;}\n.button-off {background-color: #808080;}\n\
  .button-off:active {background-color: #808080;}\n\
  p {font-size: 14px;color: #808080;margin-bottom: 20px;}\n\
  </style>\n</head>\n<body>\n<h1>STM32 - ESP8266</h1>");
  sprintf(ATcommandN,"<p>Light is currently on\
  </p><a class=\"button button-off\" href=\"/lightoff\">OFF</a>");
  sprintf(ATcommandF,"<p>Light is currently off\
  </p><a class=\"button button-on\" href=\"/lighton\">ON</a>");
  sprintf(ATcommandT,"</body></html>");
  int countB = strlen(ATcommandB);
  int countN = strlen(ATcommandN);
  int countF = strlen(ATcommandF);
  int countT = strlen(ATcommandT);

  // Initialize ESP8266
  uint8_t initMsg[] = "Initializing ESP8266...\r\n";
  HAL_UART_Transmit(&huart1, initMsg, strlen((char*)initMsg), 1000);
  
  sprintf(ATcommand,"AT+RST\r\n");
  memset(rxBuffer,0,sizeof(rxBuffer));
  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
  HAL_UART_Receive (&huart1, rxBuffer, 512, 3000);  // Longer timeout for reset
  
  // Print what we received from ESP8266
  uint8_t debugMsg[] = "ESP8266 Reset Response:\r\n";
  HAL_UART_Transmit(&huart1, debugMsg, strlen((char*)debugMsg), 1000);
  HAL_UART_Transmit(&huart1, rxBuffer, strlen((char*)rxBuffer), 1000);
  
  HAL_Delay(2000);  // Give ESP8266 time to boot
  
  // Print startup message
  uint8_t startMsg[] = "STM32-ESP8266 WiFi CCU Starting...\r\n";
  HAL_UART_Transmit(&huart1, startMsg, strlen((char*)startMsg), 1000);

  ATisOK = 0;
  while(!ATisOK){
    sprintf(ATcommand,"AT+CWMODE=1\r\n");  // Set to Station mode to connect to WiFi
      memset(rxBuffer,0,sizeof(rxBuffer));
      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
      HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
    if(strstr((char *)rxBuffer,"OK")){
      ATisOK = 1;
      // Print confirmation
      sprintf(ATcommand,"ESP8266: Station mode set\r\n");
      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    }
    HAL_Delay(500);
  }

  ATisOK = 0;
  while(!ATisOK){
    sprintf(ATcommand,"AT+CWJAP=\"MyTether\",\"asdfghjkl\"\r\n");  // Connect to phone hotspot
      memset(rxBuffer,0,sizeof(rxBuffer));
      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
      HAL_UART_Receive (&huart1, rxBuffer, 512, 5000);  // Longer timeout for connection
    if(strstr((char *)rxBuffer,"WIFI CONNECTED")){
      ATisOK = 1;
      // Print confirmation
      sprintf(ATcommand,"ESP8266: Connected to WiFi!\r\n");
      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    }
    HAL_Delay(1000);
  }

  // Get IP address after WiFi connection
  sprintf(ATcommand,"AT+CIFSR\r\n");
  memset(rxBuffer,0,sizeof(rxBuffer));
  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
  
  // Extract and print IP address (look for IP pattern like "192.168.1.100")
  char ipAddress[20] = {0};
  char *ipStart = strstr((char*)rxBuffer, "+CIFSR:STAIP,\"");
  if (ipStart) {
    ipStart += 14; // Skip "+CIFSR:STAIP,\""
    char *ipEnd = strchr(ipStart, '"');
    if (ipEnd) {
      int ipLen = ipEnd - ipStart;
      if (ipLen < sizeof(ipAddress) - 1) {
        strncpy(ipAddress, ipStart, ipLen);
        ipAddress[ipLen] = '\0';
      }
    }
  }
  
  // Print IP address
  sprintf(ATcommand,"IP Address: %s\r\n", ipAddress);
  HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
  HAL_Delay(1000);

  ATisOK = 0;
  while(!ATisOK){
    sprintf(ATcommand,"AT+CIPMUX=1\r\n");
      memset(rxBuffer,0,sizeof(rxBuffer));
      HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
      HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
      if(strstr((char *)rxBuffer,"OK")){
        ATisOK = 1;
      }
      HAL_Delay(500);
  }

  ATisOK = 0;
  while(!ATisOK){
    sprintf(ATcommand,"AT+CIPSERVER=1,80\r\n");
    memset(rxBuffer,0,sizeof(rxBuffer));
    HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
    if(strstr((char *)rxBuffer,"OK")){
        ATisOK = 1;
    }
    HAL_Delay(500);
  }
	// Test ESP01 basic communication first
	sprintf(ATcommand,"Testing ESP8266 connection...\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	
	if (ESP_TestConnection()) {
		// ESP01 is responding, proceed with full initialization
		sprintf(ATcommand,"ESP8266 responding, initializing...\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
		
		if (ESP_Init()) {
			// ESP01 initialized successfully
			sprintf(ATcommand,"ESP8266 initialization successful!\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
		} else {
			// ESP01 initialization failed
			sprintf(ATcommand,"ESP8266 initialization failed!\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
		}
	} else {
		// ESP01 not responding - check connections and power
		sprintf(ATcommand,"ESP8266 not responding - check connections!\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {memset(rxBuffer,0,sizeof(rxBuffer));
  HAL_UART_Receive (&huart1, rxBuffer, 512, 1000);
  if(strstr((char *)rxBuffer,"+IPD,0")) channel = 0;
  else if(strstr((char *)rxBuffer,"+IPD,1")) channel = 1;
  else if(strstr((char *)rxBuffer,"+IPD,2")) channel = 2;
  else if(strstr((char *)rxBuffer,"+IPD,3")) channel = 3;
  else if(strstr((char *)rxBuffer,"+IPD,4")) channel = 4;
  else if(strstr((char *)rxBuffer,"+IPD,5")) channel = 5;
  else if(strstr((char *)rxBuffer,"+IPD,6")) channel = 6;
  else if(strstr((char *)rxBuffer,"+IPD,7")) channel = 7;
  else channel = 100;

  if(strstr((char *)rxBuffer,"GET /lighton")) onoff = 0;
  else if(strstr((char *)rxBuffer,"GET /lightoff")) onoff = 1;
  else onoff = led;

  if(channel<8 && onoff == 1)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    led = 1;
    sprintf(ATcommand,"AT+CIPSEND=%d,%d\r\n",channel,countB+countF+countT);
    memset(rxBuffer,0,sizeof(rxBuffer));
    HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
    if(strstr((char *)rxBuffer,">"))
    {
      memset(rxBuffer,0,sizeof(rxBuffer));
        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandB,countB,1000);
        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandF,countF,1000);
        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandT,countT,1000);
       HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
    }
    sprintf(ATcommand,"AT+CIPCLOSE=%d\r\n",channel);
    memset(rxBuffer,0,sizeof(rxBuffer));
    HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
    channel=100;
  }
  else if(channel<8 && onoff == 0)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    led = 0;
    sprintf(ATcommand,"AT+CIPSEND=%d,%d\r\n",channel,countB+countN+countT);
    memset(rxBuffer,0,sizeof(rxBuffer));
    HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
    if(strstr((char *)rxBuffer,">"))
    {
      memset(rxBuffer,0,sizeof(rxBuffer));
        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandB,countB,1000);
        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandN,countN,1000);
        HAL_UART_Transmit(&huart1,(uint8_t *)ATcommandT,countT,1000);
        HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
    }
    sprintf(ATcommand,"AT+CIPCLOSE=%d\r\n",channel);
    memset(rxBuffer,0,sizeof(rxBuffer));
    HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
    HAL_UART_Receive (&huart1, rxBuffer, 512, 100);
    channel=100;
  }
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
  RCC_OscInitStruct.PLL.PLLN = 72;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

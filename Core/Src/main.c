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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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
  void send_AT_command(const char *cmd, uint32_t delay_ms) {
      HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
      HAL_Delay(delay_ms);
  }

  void send_and_receive(const char *cmd, uint32_t delay_ms) {
      uint8_t buffer[512] = {0};

      printf("Sending: %s", cmd);
      HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
      HAL_Delay(delay_ms);

      HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, buffer, sizeof(buffer)-1, 2000);
      if(status == HAL_OK) {
          printf("Response: %s\r\n", buffer);
      } else {
          printf("No response (timeout)\r\n");
      }
  }

  void ESP01_ConnectWiFi(const char *ssid, const char *password) {
      printf("Resetting ESP-01...\r\n");
      send_and_receive("AT+RST\r\n", 3000);        // Reset ESP
      HAL_Delay(2000);                             // Wait for boot

      printf("Setting station mode...\r\n");
      send_and_receive("AT+CWMODE=1\r\n", 1000);   // Set mode: Station (client)

      char wifiCommand[128];
      snprintf(wifiCommand, sizeof(wifiCommand),
               "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

      printf("Connecting to WiFi: %s\r\n", ssid);
      send_and_receive(wifiCommand, 15000);         // Connect to Wi-Fi network (increased timeout)
      printf("WiFi connection attempt completed\r\n");
  }
  void send_AT(const char *cmd, uint32_t delay_ms) {
      HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
      HAL_Delay(delay_ms);
  }

  void send_data(uint8_t *data, uint16_t len) {
      HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
      HAL_Delay(500); // small delay after sending data
  }

  void ESP01_ConnectMQTTBroker(const char *broker, uint16_t port) {
      char cmd[128];
      snprintf(cmd, sizeof(cmd),
               "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", broker, port);
      printf("Connecting to MQTT broker: %s:%d\r\n", broker, port);
      send_and_receive(cmd, 10000);  // Increased timeout for TCP connection
      
      // Verify connection status
      printf("Checking TCP connection status...\r\n");
      send_and_receive("AT+CIPSTATUS\r\n", 2000);
  }
  
  void ESP01_CheckConnection(void) {
      printf("Checking ESP-01 and TCP connection...\r\n");
      send_and_receive("AT\r\n", 1000);           // Basic AT test
      send_and_receive("AT+CIPSTATUS\r\n", 2000); // TCP connection status
  }

  void ESP01_Send_MQTT_CONNECT(const char *clientID) {
      uint8_t packet[128];
      uint8_t clientID_len = strlen(clientID);
      uint8_t total_len = 10 + 2 + clientID_len;

      uint8_t index = 0;
      packet[index++] = 0x10;                      // CONNECT command
      packet[index++] = total_len;                 // Remaining length
      packet[index++] = 0x00; packet[index++] = 0x04; // Length of "MQTT"
      packet[index++] = 'M'; packet[index++] = 'Q';
      packet[index++] = 'T'; packet[index++] = 'T';
      packet[index++] = 0x04;                      // Protocol level 4
      packet[index++] = 0x02;                      // Clean session
      packet[index++] = 0x00; packet[index++] = 0x3C; // Keep alive = 60 sec
      packet[index++] = 0x00; packet[index++] = clientID_len;
      memcpy(&packet[index], clientID, clientID_len);
      index += clientID_len;

      printf("Sending MQTT CONNECT packet (%d bytes)...\r\n", index);
      
      // Send CIPSEND and wait for ">" prompt
      char sendCmd[32];
      snprintf(sendCmd, sizeof(sendCmd), "AT+CIPSEND=%d\r\n", index);
      send_and_receive(sendCmd, 2000);  // Wait for ">" prompt
      
      HAL_Delay(100);  // Small delay before sending data

      // Send the actual packet and wait for response
      printf("Sending MQTT CONNECT data...\r\n");
      HAL_UART_Transmit(&huart1, packet, index, HAL_MAX_DELAY);
      
      // Wait for CONNACK response
      uint8_t response[64] = {0};
      HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, response, sizeof(response)-1, 5000);
      if(status == HAL_OK) {
          printf("MQTT CONNECT Response: %s\r\n", response);
      } else {
          printf("No MQTT CONNECT response\r\n");
      }
  }
  void ESP01_Send_MQTT_PUBLISH(const char *topic, const char *message) {
      uint8_t packet[256];
      uint8_t topic_len = strlen(topic);
      uint8_t message_len = strlen(message);

      uint8_t remaining_len = 2 + topic_len + message_len;
      uint8_t index = 0;

      packet[index++] = 0x30;                       // PUBLISH command
      packet[index++] = remaining_len;              // Remaining length
      packet[index++] = 0x00;
      packet[index++] = topic_len;
      memcpy(&packet[index], topic, topic_len);
      index += topic_len;
      memcpy(&packet[index], message, message_len);
      index += message_len;

      // Send CIPSEND first
      char sendCmd[32];
      snprintf(sendCmd, sizeof(sendCmd),
               "AT+CIPSEND=%d\r\n", index);
      send_AT(sendCmd, 1000);

      // Send packet data
      send_data(packet, index);
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n=== STM32F401CCU6 ESP-01 MQTT Client ===\r\n");
  printf("System Clock: %lu Hz\r\n", SystemCoreClock);
  printf("Initializing ESP-01 WiFi connection...\r\n");
  
  // Blink LED to indicate system start
  for(int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
      HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   // LED OFF
      HAL_Delay(200);
  }
  
  HAL_Delay(2000);
  ESP01_ConnectWiFi("MyTether", "asdfghjkl");
  ESP01_ConnectMQTTBroker("test.mosquitto.org", 1883);

  HAL_Delay(2000);

  ESP01_Send_MQTT_CONNECT("STM32Client");
  HAL_Delay(2000);
  
  // Test with short message first
  printf("Testing with short message...\r\n");
  ESP01_Send_MQTT_PUBLISH("test", "STM32_OK");
  HAL_Delay(1000);
  
  printf("Starting MQTT publishing loop...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t message_count = 0;
  while (1)
  {
    /* USER CODE END WHILE */
    // Toggle LED to indicate activity
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    // Check connection every 10 messages
    if(message_count % 10 == 0) {
        ESP01_CheckConnection();
    }
    
    // Send device status every 20 messages (every ~100 seconds) 
    if(message_count % 20 == 0) {
        char status_msg[64];
        int avg_temp = 25 + (message_count / 10) % 8;  // Average temperature trend
        snprintf(status_msg, sizeof(status_msg), 
                 "status=online,uptime=%lu,avgT=%d,msgs=%lu", 
                 HAL_GetTick()/1000, avg_temp, message_count);
        printf("Publishing status: %s\r\n", status_msg);
        ESP01_Send_MQTT_PUBLISH("telemetry/stm32/status", status_msg);
        HAL_Delay(1000); // Small delay between status and data
    }
    
    char temp_message[64];
    // Generate more varied and comprehensive sensor data with realistic patterns
    int soil_temp = 22 + (message_count % 12) + (message_count / 10) % 3;    // 22-36°C with slow drift
    int air_temp = 24 + (message_count % 10) + (message_count / 15) % 4;     // 24-37°C 
    int soil_humidity = 45 + (message_count % 25) + (message_count / 5) % 10; // 45-79% with variation
    int air_humidity = 40 + (message_count % 30) + (message_count / 8) % 8;   // 40-77%
    int pressure = 780 + (message_count % 60) + (message_count / 20) % 15;    // 780-854 realistic pressure range
    int water_level = 8 + (message_count % 35) + (message_count / 12) % 5;    // 8-47cm varying water level
    
    snprintf(temp_message, sizeof(temp_message), 
             "st=%d,at=%d,sh=%d,ah=%d,p=%d,wl=%d,c=%lu", 
             soil_temp, air_temp, soil_humidity, air_humidity, pressure, water_level, message_count);
    
    printf("Publishing message %lu: %s\r\n", message_count, temp_message);
    ESP01_Send_MQTT_PUBLISH("telemetry/stm32/data", temp_message);
    
    message_count++;
    HAL_Delay(5000); // Send every 5 seconds
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

// Printf redirection to USART2
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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

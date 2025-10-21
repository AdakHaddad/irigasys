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
// #include "esp8266.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  void send_AT_command(const char *cmd, uint32_t delay_ms) {
      HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
      HAL_Delay(delay_ms);
  }

  void send_and_receive(const char *cmd, uint32_t delay_ms) {
      uint8_t buffer[512] = {0};

      HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
      HAL_Delay(delay_ms);

      HAL_UART_Receive(&huart1, buffer, sizeof(buffer), 1000);
      printf("%s\n", buffer); // Display received response on serial terminal
  }

  void ESP01_ConnectWiFi(const char *ssid, const char *password) {
      send_and_receive("AT+RST\r\n", 3000);        // Reset ESP
      HAL_Delay(2000);                             // Wait for boot

      send_and_receive("AT+CWMODE=1\r\n", 1000);   // Set mode: Station (client)

      char wifiCommand[128];
      snprintf(wifiCommand, sizeof(wifiCommand),
               "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

      send_and_receive(wifiCommand, 8000);         // Connect to Wi-Fi network
  }
  void send_AT(const char *cmd, uint32_t delay_ms) {
      HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
      HAL_Delay(delay_ms);
  }

  void send_data(uint8_t *data, uint16_t len) {
      HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
      HAL_Delay(500); // small delay after sending data
  }

  void ESP01_CheckMQTTSupport(void) {
      // Check if ESP8266 supports MQTT AT commands
      send_and_receive("AT+MQTTUSERCFG=?\r\n", 2000);
      send_and_receive("AT+MQTTCONN=?\r\n", 2000);
      send_and_receive("AT+CIPSSL=?\r\n", 2000);
  }

  void ESP01_ConnectMQTTBroker_Auth(const char *broker, uint16_t port, const char *username, const char *password) {
      char cmd[128];
      
      // Set MQTT username and password
      snprintf(cmd, sizeof(cmd), "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", 
               "STM32Client", username, password);
      send_and_receive(cmd, 2000);
      
      // Connect to MQTT broker
      snprintf(cmd, sizeof(cmd), "AT+MQTTCONN=0,\"%s\",%d,1\r\n", broker, port);
      send_and_receive(cmd, 5000);
  }

  void ESP01_ConnectMQTTBroker(const char *broker, uint16_t port) {
      char cmd[128];
      snprintf(cmd, sizeof(cmd),
               "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", broker, port);
      send_AT(cmd, 5000);
  }

  void ESP01_Send_MQTT_CONNECT_Auth(const char *clientID, const char *username, const char *password) {
      uint8_t packet[256];
      uint8_t clientID_len = strlen(clientID);
      uint8_t username_len = strlen(username);
      uint8_t password_len = strlen(password);
      
      // Calculate total length: Fixed header (10) + client ID (2 + len) + username (2 + len) + password (2 + len)
      uint8_t total_len = 10 + 2 + clientID_len + 2 + username_len + 2 + password_len;

      uint8_t index = 0;
      packet[index++] = 0x10;                      // CONNECT command
      packet[index++] = total_len;                 // Remaining length
      packet[index++] = 0x00; packet[index++] = 0x04; // Length of "MQTT"
      packet[index++] = 'M'; packet[index++] = 'Q';
      packet[index++] = 'T'; packet[index++] = 'T';
      packet[index++] = 0x04;                      // Protocol level 4
      packet[index++] = 0xC2;                      // Connect flags: Clean session + Username + Password
      packet[index++] = 0x00; packet[index++] = 0x3C; // Keep alive = 60 sec
      
      // Client ID
      packet[index++] = 0x00; packet[index++] = clientID_len;
      memcpy(&packet[index], clientID, clientID_len);
      index += clientID_len;
      
      // Username
      packet[index++] = 0x00; packet[index++] = username_len;
      memcpy(&packet[index], username, username_len);
      index += username_len;
      
      // Password
      packet[index++] = 0x00; packet[index++] = password_len;
      memcpy(&packet[index], password, password_len);
      index += password_len;

      // Send CIPSEND first
      char sendCmd[32];
      snprintf(sendCmd, sizeof(sendCmd),
               "AT+CIPSEND=%d\r\n", index);
      send_AT(sendCmd, 1000);

      // Send the actual packet
      send_data(packet, index);
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

      // Send CIPSEND first
      char sendCmd[32];
      snprintf(sendCmd, sizeof(sendCmd),
               "AT+CIPSEND=%d\r\n", index);
      send_AT(sendCmd, 1000);

      // Send the actual packet
      send_data(packet, index);
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
  /* USER CODE BEGIN 2 */
  HAL_Delay(2000);
  
  // Connect to WiFi
  ESP01_ConnectWiFi("MyTether", "asdfghjkl");
  HAL_Delay(2000);
  
  // Check what MQTT commands are supported
  ESP01_CheckMQTTSupport();
  HAL_Delay(2000);
  
  // Try HiveMQ Cloud with built-in MQTT commands
  ESP01_ConnectMQTTBroker_Auth("b2a051ac43c4410e86861ed01b937dec.s1.eu.hivemq.cloud", 1883, "user1", "P@ssw0rd");
  HAL_Delay(3000);
  
  // If above fails, try manual TCP connection
  // ESP01_ConnectMQTTBroker("b2a051ac43c4410e86861ed01b937dec.s1.eu.hivemq.cloud", 1883);
  // HAL_Delay(2000);
  // ESP01_Send_MQTT_CONNECT_Auth("STM32Client", "user1", "P@ssw0rd");
  // HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t lastSensorSend = 0;
  uint16_t sensorCounter = 0;
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Send sensor data every 10 seconds
    if (HAL_GetTick() - lastSensorSend > 10000) {
        char sensorData[200];
        
        // Simulate sensor readings (replace with actual sensor code)
        int pressure = 700 + (sensorCounter % 200);
        int soilTemp = 20 + (sensorCounter % 15);
        int soilHumidity = 50 + (sensorCounter % 40);
        int waterLevel = 10 + (sensorCounter % 20);
        int airTemp = 25 + (sensorCounter % 10);
        int airHumidity = 60 + (sensorCounter % 30);
        
        // Format JSON payload
        snprintf(sensorData, sizeof(sensorData),
                "{\"pressure\":%d,\"soilTemp\":%d,\"soilHumidity\":%d,\"waterLevel\":%d,\"airTemp\":%d,\"airHumidity\":%d,\"counter\":%d}",
                pressure, soilTemp, soilHumidity, waterLevel, airTemp, airHumidity, sensorCounter);
        
        // Publish to HiveMQ Cloud
        ESP01_Send_MQTT_PUBLISH("devices/stm32-01/telemetry", sensorData);
        
        lastSensorSend = HAL_GetTick();
        sensorCounter++;
        
        // Blink LED to show activity
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    
    HAL_Delay(100); // Small delay to prevent busy waiting
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

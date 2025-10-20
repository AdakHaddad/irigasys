#ifndef __ESP8266_H
#define __ESP8266_H

#include "main.h"
#include <string.h>
#include <stdio.h>

uint8_t ESP_SendCommand(char *cmd);
uint8_t ESP_WaitForResponse(const char *expected, uint32_t timeout);
uint8_t ESP_Init(void);
uint8_t ESP_TestConnection(void);
void ESP_PrintWiFiInfo(void);

// Use UART1 for ESP8266 communication
extern UART_HandleTypeDef huart1;

#endif

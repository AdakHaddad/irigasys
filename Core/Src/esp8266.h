#ifndef __ESP8266_H
#define __ESP8266_H

#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

void ESP_SendCommand(char *cmd);
void ESP_Init(void);

#endif

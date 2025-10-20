#include "esp8266.h"
#include <string.h>
#include <stdint.h>

#define ESP_RESPONSE_TIMEOUT 5000
#define ESP_MAX_RESPONSE_LEN 512

static void ESP_DelayMs(uint32_t ms)
{
  HAL_Delay(ms);
}

// Enhanced command sending with response checking
uint8_t ESP_SendCommand(char *cmd)
{
  if (cmd == NULL) {
    return 0;
  }
  
  // Clear UART receive buffer
  uint8_t dummy;
  while (HAL_UART_Receive(&huart1, &dummy, 1, 10) == HAL_OK);
  
  // Send command
  size_t len = strlen(cmd);
  if (HAL_UART_Transmit(&huart1, (uint8_t *)cmd, (uint16_t)len, 1000) != HAL_OK) {
    return 0;
  }
  
  return 1;
}

// Check for specific response
uint8_t ESP_WaitForResponse(const char* expected, uint32_t timeout)
{
  char response[ESP_MAX_RESPONSE_LEN] = {0};
  uint16_t index = 0;
  uint32_t start_time = HAL_GetTick();
  
  while ((HAL_GetTick() - start_time) < timeout) {
    uint8_t byte;
    if (HAL_UART_Receive(&huart1, &byte, 1, 100) == HAL_OK) {
      if (index < ESP_MAX_RESPONSE_LEN - 1) {
        response[index++] = byte;
        
        // Check if we received the expected response
        if (strstr(response, expected) != NULL) {
          return 1;
        }
        
        // Check for error responses
        if (strstr(response, "ERROR") != NULL || 
            strstr(response, "FAIL") != NULL) {
          return 0;
        }
      }
    }
  }
  
  return 0; // Timeout
}

static void ESP_SendBinary(const uint8_t *data, uint16_t len)
{
  char header[32];
  int headerLen = snprintf(header, sizeof(header), "AT+CIPSEND=%u\r\n", (unsigned)len);
  if (headerLen > 0) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)header, (uint16_t)headerLen, 1000);
  }
  ESP_DelayMs(200);
  if (data != NULL && len > 0) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 3000);
  }
  ESP_DelayMs(300);
}

// MQTT helpers (MQTT 3.1.1)
static uint8_t mqtt_encode_remaining_length(uint8_t *buf, uint32_t len)
{
  uint8_t bytes = 0;
  do {
    uint8_t encodedByte = len % 128;
    len /= 128;
    // if there are more data to encode, set the top bit of this byte
    if (len > 0) encodedByte |= 128;
    buf[bytes++] = encodedByte;
  } while (len > 0 && bytes < 4);
  return bytes;
}

static uint16_t write_utf8(uint8_t *buf, uint16_t offset, const char *str)
{
  uint16_t len = (uint16_t)strlen(str);
  buf[offset++] = (uint8_t)((len >> 8) & 0xFF);
  buf[offset++] = (uint8_t)(len & 0xFF);
  memcpy(&buf[offset], str, len);
  return (uint16_t)(offset + len);
}

static size_t mqtt_build_connect(uint8_t *out, size_t outSize,
                                 const char *clientId,
                                 const char *username,
                                 const char *password)
{
  // Variable header
  uint8_t vh[10 + 2 + 2];
  uint16_t idx = 0;
  // Protocol Name "MQTT"
  idx = write_utf8(vh, idx, "MQTT");
  vh[idx++] = 0x04; // Protocol Level 4 (3.1.1)
  // Connect Flags
  uint8_t connectFlags = 0x02; // Clean session
  if (username && *username) connectFlags |= 0x80;
  if (password && *password) connectFlags |= 0x40;
  vh[idx++] = connectFlags;
  // Keep Alive (seconds)
  vh[idx++] = 0x00;
  vh[idx++] = 60; // 60s

  // Payload
  uint8_t payload[256];
  uint16_t pidx = 0;
  pidx = write_utf8(payload, pidx, clientId);
  if (username && *username) {
    pidx = write_utf8(payload, pidx, username);
  }
  if (password && *password) {
    pidx = write_utf8(payload, pidx, password);
  }

  // Fixed header
  uint32_t remainingLen = idx + pidx;
  uint8_t rl[4];
  uint8_t rlLen = mqtt_encode_remaining_length(rl, remainingLen);
  size_t totalLen = 1 + rlLen + remainingLen;
  if (totalLen > outSize) return 0;

  size_t o = 0;
  out[o++] = 0x10; // CONNECT
  memcpy(&out[o], rl, rlLen); o += rlLen;
  memcpy(&out[o], vh, idx); o += idx;
  memcpy(&out[o], payload, pidx); o += pidx;
  return o;
}

static size_t mqtt_build_publish(uint8_t *out, size_t outSize,
                                 const char *topic,
                                 const char *payload)
{
  uint8_t header = 0x30; // PUBLISH QoS0

  uint8_t varHeader[256];
  uint16_t vhLen = write_utf8(varHeader, 0, topic);

  const uint8_t *pl = (const uint8_t *)payload;
  uint16_t plLen = (uint16_t)strlen(payload);

  uint32_t remainingLen = vhLen + plLen;
  uint8_t rl[4];
  uint8_t rlLen = mqtt_encode_remaining_length(rl, remainingLen);

  size_t totalLen = 1 + rlLen + remainingLen;
  if (totalLen > outSize) return 0;

  size_t o = 0;
  out[o++] = header;
  memcpy(&out[o], rl, rlLen); o += rlLen;
  memcpy(&out[o], varHeader, vhLen); o += vhLen;
  memcpy(&out[o], pl, plLen); o += plLen;
  return o;
}

uint8_t ESP_Init(void)
{
  // Basic init sequence for ESP8266 with AT firmware over USART1
  // NOTE: Replace <SSID> and <PASSWORD> with your WiFi credentials
  // Broker: b2a051ac43c4410e86861ed01b937dec.s1.eu.hivemq.cloud

  // Step 1: Test communication
  if (!ESP_SendCommand("AT\r\n") || !ESP_WaitForResponse("OK", 2000)) {
    return 0; // ESP not responding
  }
  ESP_DelayMs(500);

  // Step 2: Disable echo
  if (!ESP_SendCommand("ATE0\r\n") || !ESP_WaitForResponse("OK", 1000)) {
    return 0;
  }
  ESP_DelayMs(200);

  // Step 3: Set to station mode
  if (!ESP_SendCommand("AT+CWMODE=1\r\n") || !ESP_WaitForResponse("OK", 1000)) {
    return 0;
  }
  ESP_DelayMs(200);

  // Step 4: Join WiFi network
  if (!ESP_SendCommand("AT+CWJAP=\"bawang\",\"12345678\"\r\n")) {
    return 0;
  }
  
  // Wait for WiFi connection (can take longer)
  if (!ESP_WaitForResponse("WIFI CONNECTED", 10000)) {
    return 0; // Failed to connect to WiFi
  }
  
  if (!ESP_WaitForResponse("WIFI GOT IP", 5000)) {
    return 0; // Failed to get IP
  }
  ESP_DelayMs(1000);

  // Step 5: Enable SSL for subsequent TCP connections
  if (!ESP_SendCommand("AT+CIPSSL=1\r\n") || !ESP_WaitForResponse("OK", 1000)) {
    return 0;
  }
  ESP_DelayMs(200);

  // Step 6: Start SSL TCP to HiveMQ (port 8883)
  if (!ESP_SendCommand("AT+CIPSTART=\"SSL\",\"b2a051ac43c4410e86861ed01b937dec.s1.eu.hivemq.cloud\",8883\r\n")) {
    return 0;
  }
  
  if (!ESP_WaitForResponse("CONNECT", 5000)) {
    return 0; // Failed to connect to MQTT broker
  }
  ESP_DelayMs(1000);

  // Step 7: Build and send MQTT CONNECT
  const char *clientId = "stm32-01";
  const char *username = "user1";
  const char *password = "P@ssw0rd";
  uint8_t packet[512];
  size_t pktLen = mqtt_build_connect(packet, sizeof(packet), clientId, username, password);
  if (pktLen > 0) {
    ESP_SendBinary(packet, (uint16_t)pktLen);
    ESP_DelayMs(1500);
  }

  // Step 8: Publish one sample telemetry message
  const char *topic = "devices/stm32-01/telemetry";
  const char *payload = "{\"pressure\":820,\"soilTemp\":36,\"soilHumidity\":72,\"waterLevel\":18,\"airTemp\":34,\"airHumidity\":70}";
  pktLen = mqtt_build_publish(packet, sizeof(packet), topic, payload);
  if (pktLen > 0) {
    ESP_SendBinary(packet, (uint16_t)pktLen);
    ESP_DelayMs(500);
  }

  return 1; // Success
}

// Test basic ESP01 communication
uint8_t ESP_TestConnection(void)
{
  ESP_DelayMs(1000); // Give ESP time to boot
  
  // Test basic AT command
  if (!ESP_SendCommand("AT\r\n")) {
    return 0;
  }
  
  return ESP_WaitForResponse("OK", 2000);
}

// Get and print WiFi connection information
void ESP_PrintWiFiInfo(void)
{
  // Get IP address
  ESP_SendCommand("AT+CIFSR\r\n");
  ESP_DelayMs(1000);
  
  // Get connection status
  ESP_SendCommand("AT+CWJAP?\r\n");
  ESP_DelayMs(1000);
  
  // Get signal strength
  ESP_SendCommand("AT+CWLAP\r\n");
  ESP_DelayMs(2000);
}



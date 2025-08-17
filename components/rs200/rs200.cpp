#include "rs200.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rs200 {

static const char *const TAG = "rs200";

void RS200Sensor::setup() {
  ESP_LOGD(TAG, "RS200 sensor initialized");
  
  // Verify the request CRC is correct
  uint8_t request[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
  uint8_t calculated_crc = this->calculate_crc_(&request[1], 3);
  ESP_LOGD(TAG, "Request CRC verification: calculated=0x%02X, expected=0x%02X", 
           calculated_crc, request[4]);
}

void RS200Sensor::update() {
  // Clear any stale data in the buffer first
  while (this->available()) {
    this->read();
  }
  
  // Request rainfall status
  uint8_t request[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
  
  ESP_LOGV(TAG, "Sending request: %02X %02X %02X %02X %02X", 
           request[0], request[1], request[2], request[3], request[4]);
  
  this->write_array(request, sizeof(request));
  this->flush();  // Ensure data is sent
  
  // Wait for response with timeout
  uint32_t start_time = millis();
  while (this->available() < 5 && millis() - start_time < 200) {
    delay(10);
  }

  if (this->available() >= 5) {
    uint8_t response[5];
    this->read_array(response, sizeof(response));
    
    ESP_LOGV(TAG, "Received response: %02X %02X %02X %02X %02X", 
             response[0], response[1], response[2], response[3], response[4]);

    if (response[0] != 0x3A) {
      ESP_LOGW(TAG, "Invalid header 0x%02X, expected 0x3A", response[0]);
      // Log the full response for debugging
      ESP_LOGW(TAG, "Full response: %02X %02X %02X %02X %02X", 
               response[0], response[1], response[2], response[3], response[4]);
      return;
    }

    uint8_t crc = this->calculate_crc_(&response[1], 3);  // FLAG + DATA_L + DATA_H
    if (crc != response[4]) {
      ESP_LOGW(TAG, "CRC mismatch expected=0x%02X got=0x%02X", crc, response[4]);
      return;
    }

    if (response[1] == 0x81) {  // Rainfall status response
      uint8_t status = response[2];
      this->publish_state(status);
      ESP_LOGD(TAG, "Rain status: %u", status);
    } else {
      ESP_LOGW(TAG, "Unexpected response flag: 0x%02X", response[1]);
    }
  } else {
    ESP_LOGW(TAG, "Timeout waiting for response, available bytes: %d", this->available());
  }
}

uint8_t RS200Sensor::calculate_crc_(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

}  // namespace rs200
}  // namespace esphome

#include "rs200.h"

namespace esphome {
namespace rs200 {

static const char *const TAG = "rs200";

void RS200Sensor::setup() {
  ESP_LOGD(TAG, "RS200 sensor initialized");
}

void RS200Sensor::update() {
  // Request rainfall status
  uint8_t request[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
  this->write_array(request, sizeof(request));
  delay(50);  // wait for response (sensor is slow)

  if (this->available() >= 5) {
    uint8_t response[5];
    this->read_array(response, sizeof(response));

    if (response[0] != 0x3A) {
      ESP_LOGW(TAG, "Invalid header 0x%02X", response[0]);
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
    }
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

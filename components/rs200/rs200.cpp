#include "rs200.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rs200 {

static const char *const TAG = "rs200";

void RS200Component::setup() {
  // Enter real-time rainfall mode
  const uint8_t enter_realtime_mode[] = {0x3A, 0x84, 0x01, 0x00, 0x43};
  this->write_array(enter_realtime_mode, sizeof(enter_realtime_mode));
}

void RS200Component::update() {
  // Request rainfall status
  const uint8_t rainfall_cmd[] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
  this->write_array(rainfall_cmd, sizeof(rainfall_cmd));
  esphome::delay(50);

  // Request system status
  const uint8_t system_cmd[] = {0x3A, 0x02, 0x00, 0x00, 0xC7};
  this->write_array(system_cmd, sizeof(system_cmd));
  esphome::delay(50);

  // Read frames (5 bytes each)
  while (this->available() >= 5) {
    uint8_t response[5];
    this->read_array(response, 5);
    if (response[0] != 0x3A)
      continue;

    uint8_t crc = this->calculate_crc(&response[1], 3);
    if (crc != response[4]) {
      ESP_LOGW(TAG, "CRC mismatch exp=%02X got=%02X", crc, response[4]);
      continue;
    }

    uint8_t flag = response[1];
    uint16_t value = uint16_t(response[2]) | (uint16_t(response[3]) << 8);

    if (flag == 0x81) {  // Rainfall status
      if (this->rainfall_status_sensor_ != nullptr)
        this->rainfall_status_sensor_->publish_state(value);
      ESP_LOGD(TAG, "Rainfall status=%u", value);
    } else if (flag == 0x82) {  // System status
      if (this->system_status_sensor_ != nullptr)
        this->system_status_sensor_->publish_state(value);
      ESP_LOGD(TAG, "System status=%u", value);
    } else {
      ESP_LOGV(TAG, "Unhandled flag=0x%02X value=0x%04X", flag, value);
    }
  }
}

uint8_t RS200Component::calculate_crc(const uint8_t *data, uint8_t len) {
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

#include "rs200.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rs200 {

static const char *const TAG = "rs200";

void RS200Sensor::setup() {
  ESP_LOGD(TAG, "RS200 sensor initialized");
  this->reset_frame_();
}

void RS200Sensor::update() {
  // Drain any leftover bytes to avoid mixing frames
  while (this->available()) {
    uint8_t dump;
    this->read_byte(&dump);
  }

  // Build and send request frame (example: command 0x01, payload 0x0000)
  uint8_t request[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
  this->write_array(request, sizeof(request));
  this->flush();

  // Wait a bit for response (device dependent)
  esphome::delay(60);

  // Read incoming bytes and feed state machine
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b)) break;
    this->process_byte_(b);
  }
}

uint8_t RS200Sensor::calculate_crc(const uint8_t *data, uint8_t len) {
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

void RS200Sensor::reset_frame_() {
  this->frame_index_ = 0;
}

void RS200Sensor::process_byte_(uint8_t b) {
  if (frame_index_ == 0) {
    if (b != 0x3A) {
      // still searching for start byte
      return;
    }
  }

  frame_[frame_index_++] = b;

  if (frame_index_ < FRAME_LEN)
    return;  // need more bytes

  // We have a full frame
  frame_index_ = 0;  // reset for next frame

  // Basic validation
  if (frame_[0] != 0x3A) {
    ESP_LOGW(TAG, "Frame without start marker discarded");
    return;
  }

  uint8_t calc_crc = this->calculate_crc(&frame_[1], 3);  // FLAG + DATA_L + DATA_H
  if (calc_crc != frame_[4]) {
    ESP_LOGW(TAG, "CRC mismatch exp=%02X got=%02X", calc_crc, frame_[4]);
    return;
  }

  uint8_t flag = frame_[1];
  uint16_t data = uint16_t(frame_[2]) | (uint16_t(frame_[3]) << 8);

  if (flag == 0x81) {
    // Interpret data low byte as status (per earlier assumption)
    uint8_t status = frame_[2];
    this->publish_state(status);
    ESP_LOGD(TAG, "Rain status=%u raw=0x%04X", status, data);
  } else if (flag == 0x90) {  // Example: system status
    if (this->system_status_sensor_ != nullptr) {
      this->system_status_sensor_->publish_state(data);
      ESP_LOGD(TAG, "System status=%u", data);
    }
  } else {
    ESP_LOGV(TAG, "Unhandled flag 0x%02X data=0x%04X", flag, data);
  }
}

}  // namespace rs200
}  // namespace esphome

#include "rs200.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rs200 {

static const char *const TAG = "rs200";

// --- CRC helpers to identify correct variant ---
static inline uint8_t crc8_poly31_no_ref(const uint8_t *data, uint8_t len, uint8_t init) {
  uint8_t crc = init;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  return crc;
}

static inline uint8_t reflect8(uint8_t v) {
  v = (v & 0xF0) >> 4 | (v & 0x0F) << 4;
  v = (v & 0xCC) >> 2 | (v & 0x33) << 2;
  v = (v & 0xAA) >> 1 | (v & 0x55) << 1;
  return v;
}

static inline uint8_t crc8_dallas_maxim(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;  // Dallas/Maxim init
  for (uint8_t i = 0; i < len; i++) {
    uint8_t byte = reflect8(data[i]);
    crc ^= byte;
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc <<= 1;
    }
  }
  crc = reflect8(crc);  // reflect output
  return crc;
}
// ------------------------------------------------

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

    const uint8_t *payload = &response[1];  // 3 bytes
    uint8_t crc_dev = response[4];

    uint8_t crc_ff = crc8_poly31_no_ref(payload, 3, 0xFF);      // current implementation variant
    uint8_t crc_00 = crc8_poly31_no_ref(payload, 3, 0x00);      // no-ref init 0x00
    uint8_t crc_maxim = crc8_dallas_maxim(payload, 3);          // reflected Dallas/Maxim

    bool crc_ok = false;
    if (crc_dev == crc_ff) {
      crc_ok = true;  // matches init 0xFF no reflect
    } else if (crc_dev == crc_00) {
      crc_ok = true;  // matches init 0x00 no reflect
    } else if (crc_dev == crc_maxim) {
      crc_ok = true;  // matches Dallas/Maxim
    }

    if (!crc_ok) {
      ESP_LOGW(TAG, "CRC mismatch dev=%02X ours ff=%02X 00=%02X maxim=%02X bytes=%02X %02X %02X", crc_dev, crc_ff, crc_00, crc_maxim, response[1], response[2], response[3]);
      continue;
    } else {
      ESP_LOGV(TAG, "CRC ok dev=%02X (ff=%02X 00=%02X maxim=%02X) flag=%02X data=%02X %02X", crc_dev, crc_ff, crc_00, crc_maxim, response[1], response[2], response[3]);
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
  // Retain old variant (init 0xFF, no reflection)
  return crc8_poly31_no_ref(data, len, 0xFF);
}

}  // namespace rs200
}  // namespace esphome

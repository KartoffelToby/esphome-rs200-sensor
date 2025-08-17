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

static inline uint8_t crc8_poly07(const uint8_t *data, uint8_t len, uint8_t init) {
  uint8_t crc = init;  // poly 0x07, no reflect
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

static inline uint8_t crc8_poly8c(const uint8_t *data, uint8_t len, uint8_t init) {
  // Unreflected representation of Dallas poly (reflected 0x31 -> normal 0x8C)
  uint8_t crc = init;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x8C;
      else
        crc <<= 1;
    }
  }
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

  // Raw capture + sliding window decode
  std::vector<uint8_t> raw;
  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b)) break;
    raw.push_back(b);
  }
  if (!raw.empty()) {
    char buf[256];
    size_t idx = 0;
    std::string line;
    for (uint8_t v : raw) {
      snprintf(buf, sizeof(buf), "%02X ", v);
      line += buf;
      if (++idx >= 32) { ESP_LOGV(TAG, "RAW: %s", line.c_str()); line.clear(); idx = 0; }
    }
    if (!line.empty()) ESP_LOGV(TAG, "RAW: %s", line.c_str());
  }

  // Process raw with 5-byte sliding window
  for (size_t i = 0; i + 5 <= raw.size(); ++i) {
    if (raw[i] != 0x3A) continue;  // start marker
    uint8_t frame[5];
    memcpy(frame, &raw[i], 5);
    // Candidate payload bytes positions 1..3
    uint8_t crc_dev = frame[4];
    const uint8_t *payload = &frame[1];
    uint8_t crc_ff = crc8_poly31_no_ref(payload, 3, 0xFF);
    uint8_t crc_00 = crc8_poly31_no_ref(payload, 3, 0x00);
    uint8_t crc_max = crc8_dallas_maxim(payload, 3);
    uint8_t crc_07 = crc8_poly07(payload, 3, 0x00);
    uint8_t crc_8c_ff = crc8_poly8c(payload, 3, 0xFF);
    uint8_t crc_8c_00 = crc8_poly8c(payload, 3, 0x00);
    uint8_t sum = (uint8_t)((payload[0] + payload[1] + payload[2]) & 0xFF);
    if (crc_dev == crc_ff || crc_dev == crc_00 || crc_dev == crc_max || crc_dev == crc_07 || crc_dev == crc_8c_ff || crc_dev == crc_8c_00 || crc_dev == sum) {
      ESP_LOGD(TAG, "FRAME i=%u bytes=%02X %02X %02X %02X %02X dev=%02X ff=%02X 00=%02X max=%02X 07=%02X 8cFF=%02X 8c00=%02X sum=%02X", (unsigned)i, frame[0], frame[1], frame[2], frame[3], frame[4], crc_dev, crc_ff, crc_00, crc_max, crc_07, crc_8c_ff, crc_8c_00, sum);
      uint8_t flag = frame[1];
      uint16_t value = uint16_t(frame[2]) | (uint16_t(frame[3]) << 8);
      if (flag == 0x81 && this->rainfall_status_sensor_)
        this->rainfall_status_sensor_->publish_state(value);
      else if (flag == 0x82 && this->system_status_sensor_)
        this->system_status_sensor_->publish_state(value);
    } else {
      ESP_LOGV(TAG, "NOCRC i=%u bytes=%02X %02X %02X %02X %02X dev=%02X ff=%02X 00=%02X max=%02X 07=%02X 8cFF=%02X 8c00=%02X sum=%02X", (unsigned)i, frame[0], frame[1], frame[2], frame[3], frame[4], crc_dev, crc_ff, crc_00, crc_max, crc_07, crc_8c_ff, crc_8c_00, sum);
    }
  }
}

uint8_t RS200Component::calculate_crc(const uint8_t *data, uint8_t len) {
  // Retain old variant (init 0xFF, no reflection)
  return crc8_poly31_no_ref(data, len, 0xFF);
}

}  // namespace rs200
}  // namespace esphome

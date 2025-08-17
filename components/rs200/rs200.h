#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace rs200 {

class RS200Sensor : public PollingComponent, public uart::UARTDevice, public sensor::Sensor {
 public:
  explicit RS200Sensor(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}
  void setup() override;
  void update() override;

 protected:
  uint8_t calculate_crc(const uint8_t *data, uint8_t len);
  // Frame parsing helpers
  void reset_frame_();
  void process_byte_(uint8_t b);
  static const uint8_t FRAME_LEN = 5;  // 0x3A FLAG DATA_L DATA_H CRC
  uint8_t frame_[FRAME_LEN]{};
  uint8_t frame_index_{0};
};

}  // namespace rs200
}  // namespace esphome

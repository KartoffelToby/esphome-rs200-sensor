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
};

}  // namespace rs200
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace rs200 {

class RS200Component : public PollingComponent, public uart::UARTDevice {
 public:
  explicit RS200Component(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void setup() override;
  void update() override;

  void set_rainfall_sensor(sensor::Sensor *sensor) { this->rainfall_status_sensor_ = sensor; }
  void set_system_sensor(sensor::Sensor *sensor) { this->system_status_sensor_ = sensor; }

 protected:
  uint8_t calculate_crc(const uint8_t *data, uint8_t len);

  sensor::Sensor *rainfall_status_sensor_{nullptr};
  sensor::Sensor *system_status_sensor_{nullptr};
};

}  // namespace rs200
}  // namespace esphome

#include "esphome.h"
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

void RS200Sensor::setup() {
  // Optional: Setup routines, maybe query firmware version
}

void RS200Sensor::update() {
  // Send rainfall status request
  uint8_t request[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
  this->write_array(request, 5);
  delay(50);  // wait for response

  // Read response
  if (available() >= 5) {
    uint8_t response[5];
    read_array(response, 5);

    if (response[0] != 0x3A) return;

    uint8_t crc = calculate_crc(&response[1], 3);  // FLAG + DATA_L + DATA_H
    if (crc != response[4]) return;

    if (response[1] == 0x81) {  // Rainfall status response
      uint8_t status = response[2];  // 0: no rain, 1: light, etc.
      publish_state(status);
    }
  }
}

// CRC-8: polynomial 0x31, init 0xFF, MSB first
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

}  // namespace rs200
}  // namespace esphome
#include "esphome.h"

class RS200Component : public PollingComponent, public UARTDevice {
 public:
  RS200Component(UARTComponent *parent) : UARTDevice(parent) {}

  Sensor *rainfall_status_sensor{nullptr};
  Sensor *system_status_sensor{nullptr};

  void setup() override {
    // Enter real-time rainfall mode
    uint8_t enter_realtime_mode[] = {0x3A, 0x84, 0x01, 0x00, 0x43};
    this->write_array(enter_realtime_mode, 5);
  }

  void update() override {
    // Request rainfall status
    const uint8_t rainfall_cmd[] = {0x3A, 0x01, 0x00, 0x00, 0x0D};
    this->write_array(rainfall_cmd, 5);
    delay(50);

    // Request system status
    const uint8_t system_cmd[] = {0x3A, 0x02, 0x00, 0x00, 0xC7};
    this->write_array(system_cmd, 5);
    delay(50);

    // Read response (possibly multiple frames)
    while (available() >= 5) {
      uint8_t response[5];
      read_array(response, 5);
      if (response[0] != 0x3A)
        continue;

      uint8_t crc = calculate_crc(&response[1], 3);
      if (crc != response[4])
        continue;

      uint8_t flag = response[1];
      uint16_t value = response[2] | (response[3] << 8);

      switch (flag) {
        case 0x81: // Rainfall status
          if (rainfall_status_sensor)
            rainfall_status_sensor->publish_state(value); // 0=no rain, 1=light, etc.
          break;
        case 0x82: // System status
          if (system_status_sensor)
            system_status_sensor->publish_state(value); // 0=OK, 1+=errors
          break;
        default:
          break;
      }
    }
  }

  void set_rainfall_sensor(Sensor *sensor) { rainfall_status_sensor = sensor; }
  void set_system_sensor(Sensor *sensor) { system_status_sensor = sensor; }

 private:
  uint8_t calculate_crc(uint8_t *data, uint8_t len) {
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
};

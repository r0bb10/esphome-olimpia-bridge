// --- MODBUS ASCII HANDLER HEADER ---
#pragma once

#include <vector>
#include <string>
#include "esphome/components/uart/uart.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace olimpia_bridge {

class OlimpiaBridge;  // Forward declaration

class ModbusAsciiHandler {
 public:
  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }  // Set UART instance
  void set_re_pin(GPIOPin *pin) { this->re_pin_ = pin; }  // Set receiver enable pin
  void set_de_pin(GPIOPin *pin) { this->de_pin_ = pin; }  // Set driver enable pin
  void set_direction(bool transmit);  // Set TX/RX line direction (true = TX, false = RX)

  bool send_and_receive(const std::vector<uint8_t> &request, std::vector<uint8_t> &response);  // Low-level send and receive (used by read/write ops)
  bool read_register(uint8_t address, uint16_t reg, uint8_t count, std::vector<uint16_t> &response);
  bool write_register(uint8_t address, uint16_t reg, uint16_t value);

 protected:
  std::string encode_ascii_frame(const std::vector<uint8_t> &data);  // Encode data payload into Modbus ASCII frame
  bool decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data);  // Decode ASCII frame into raw Modbus data (validates LRC)
  uint8_t compute_lrc(const std::vector<uint8_t> &data);  // Checksum calculation (LRC = 2's complement of sum)

  uart::UARTComponent *uart_{nullptr};  // Assigned UART component
  GPIOPin *re_pin_{nullptr};  // Receiver Enable pin
  GPIOPin *de_pin_{nullptr};  // Driver Enable pin
  uint32_t timeout_ms_{200}; // Default timeout (ms)
};

}  // namespace olimpia_bridge
}  // namespace esphome

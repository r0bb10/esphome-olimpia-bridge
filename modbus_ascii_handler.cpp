// --- MODBUS ASCII HANDLER IMPLEMENTATION ---
#include "modbus_ascii_handler.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "modbus_ascii";

// --- Direction control ---
void ModbusAsciiHandler::set_direction(bool transmit) {
  if (this->re_pin_ != nullptr)
    this->re_pin_->digital_write(transmit);  // RE low to enable RX, high to disable
  if (this->de_pin_ != nullptr)
    this->de_pin_->digital_write(transmit);   // DE high to enable TX, low to disable
}

// --- LRC Checksum ---
uint8_t ModbusAsciiHandler::compute_lrc(const std::vector<uint8_t> &data) {
  uint8_t sum = 0;
  for (uint8_t b : data) sum += b;
  return static_cast<uint8_t>(~sum + 1);
}

// --- ASCII Encoding ---
std::string ModbusAsciiHandler::encode_ascii_frame(const std::vector<uint8_t> &data) {
  static const char hex_chars[] = "0123456789ABCDEF";
  std::string result = ":";

  for (uint8_t byte : data) {
    result += hex_chars[(byte >> 4) & 0x0F];
    result += hex_chars[byte & 0x0F];
  }

  uint8_t lrc = compute_lrc(data);
  result += hex_chars[(lrc >> 4) & 0x0F];
  result += hex_chars[lrc & 0x0F];
  result += "\r\n";
  return result;
}

// --- ASCII Decoding ---
bool ModbusAsciiHandler::decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data) {
  if (frame.size() < 7 || frame[0] != ':' || frame.substr(frame.size() - 2) != "\r\n")
    return false;

  size_t length = frame.size() - 3;
  if (length % 2 != 0) return false;

  data.clear();
  uint8_t sum = 0;

  for (size_t i = 1; i < length - 2; i += 2) {
    auto hex = [](char c) -> int {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return -1;
    };
    int hi = hex(frame[i]);
    int lo = hex(frame[i + 1]);
    if (hi < 0 || lo < 0) return false;
    uint8_t byte = (hi << 4) | lo;
    data.push_back(byte);
    sum += byte;
  }

  uint8_t lrc = data.back();
  data.pop_back();  // remove LRC

  return compute_lrc(data) == lrc;
}

bool ModbusAsciiHandler::read_register(uint8_t address, uint16_t reg, uint8_t count, std::vector<uint16_t> &response) {
  std::vector<uint8_t> request = {
    address, 0x03,
    static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
    0x00, count
  };
  std::vector<uint8_t> raw_response;

  ESP_LOGD(TAG, "[Modbus] Sending read request: addr=0x%02X reg=0x%04X count=%d", address, reg, count);

  if (!send_and_receive(request, raw_response) || raw_response.size() < 3 + 2 * count) {
    ESP_LOGW(TAG, "[Modbus] No response received");
    delay(10);
    ESP_LOGD(TAG, "[Modbus] Read retry: addr=0x%02X reg=0x%04X", address, reg);
    if (!send_and_receive(request, raw_response) || raw_response.size() < 3 + 2 * count) {
      ESP_LOGW(TAG, "[Modbus] Read FAILED: addr=0x%02X reg=0x%04X", address, reg);
      return false;
    }
  }

  if (raw_response[0] != address || raw_response[1] != 0x03 || raw_response[2] != 2 * count) {
    ESP_LOGW(TAG, "[Modbus] Unexpected response header: %02X %02X %02X", raw_response[0], raw_response[1], raw_response[2]);
    return false;
  }

  response.clear();
  for (uint8_t i = 0; i < count; ++i) {
    uint16_t val = (raw_response[3 + i * 2] << 8) | raw_response[4 + i * 2];
    response.push_back(val);
  }

  return true;
}

bool ModbusAsciiHandler::write_register(uint8_t address, uint16_t reg, uint16_t value) {
  // TODO: Implement Modbus write request logic
  return false;
}

// --- TX-RX roundtrip ---
bool ModbusAsciiHandler::send_and_receive(const std::vector<uint8_t> &payload, std::vector<uint8_t> &response) {
  constexpr size_t MAX_FRAME_SIZE = 256;

  ESP_LOGD(TAG, "[Modbus] Preparing to send frame");

  if (!this->uart_)
    return false;

  // Flush RX buffer to clear stale data
  while (this->uart_->available()) {
    uint8_t dummy;
    this->uart_->read_byte(&dummy);
  }

  std::string frame = encode_ascii_frame(payload);

  ESP_LOGD(TAG, "[Modbus] Sending frame: %s", frame.c_str());

  // Enable TX mode
  set_direction(true);
  delay(2);

  this->uart_->write_str(frame.c_str());
  this->uart_->flush();
  delay(5);

  // Enable RX mode
  set_direction(false);
  delay(25);  // Quiet time before reading

  std::string buffer;
  uint32_t start = millis();

  while (millis() - start < 200) {  // 200ms timeout
    while (this->uart_->available()) {
      uint8_t c;
      this->uart_->read_byte(&c);
      if (c == 0x00)  // Skip null bytes
        continue;

      buffer += static_cast<char>(c);

      if (buffer.size() > MAX_FRAME_SIZE) {
        ESP_LOGW(TAG, "[Modbus] RX buffer overflow");
        break;
      }

      if (buffer.size() >= 2 && buffer[buffer.size()-2] == '\r' && buffer[buffer.size()-1] == '\n')
        goto decode;
    }
    delay(1);
  }

decode:
  ESP_LOGD(TAG, "[Modbus] Received frame: %s", buffer.c_str());

  if (buffer.empty()) {
    ESP_LOGW(TAG, "[Modbus] No response received");
    return false;
  }

  if (!decode_ascii_frame(buffer, response)) {
    ESP_LOGW(TAG, "[Modbus] Failed to decode response");
    return false;
  }

  return true;
}

}  // namespace olimpia_bridge
}  // namespace esphome

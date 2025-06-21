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
  for (uint8_t b : data)
    sum += b;
  return static_cast<uint8_t>(-sum);  // two's complement, same as ~sum + 1
}

// --- Hex helper functions for speed and clarity ---
static constexpr char HEX_CHARS[] = "0123456789ABCDEF";

inline uint8_t hex_char_to_val(char c) {
  // Fast hex conversion, assuming valid ASCII input
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;  // invalid
}

// --- ASCII Encoding ---
std::string ModbusAsciiHandler::encode_ascii_frame(const std::vector<uint8_t> &data) {
  // Reserve exact size upfront for performance: 
  // 1 start + 2 chars per byte + 2 chars LRC + 2 for CRLF
  std::string result;
  result.reserve(1 + 2 * (data.size() + 1) + 2);

  result.push_back(':');

  for (uint8_t byte : data) {
    result.push_back(HEX_CHARS[(byte >> 4) & 0x0F]);
    result.push_back(HEX_CHARS[byte & 0x0F]);
  }

  uint8_t lrc = compute_lrc(data);
  result.push_back(HEX_CHARS[(lrc >> 4) & 0x0F]);
  result.push_back(HEX_CHARS[lrc & 0x0F]);

  result += "\r\n";

  ESP_LOGD(TAG, "[Modbus] Encoded ASCII frame: %s", result.c_str());

  return result;
}

// --- ASCII Decoding ---
bool ModbusAsciiHandler::decode_ascii_frame(const std::string &frame, std::vector<uint8_t> &data) {
  if (frame.size() < 7 || frame[0] != ':' || frame.substr(frame.size() - 2) != "\r\n") {
    ESP_LOGW(TAG, "[Modbus] Invalid frame start/end");
    return false;
  }

  size_t payload_len = frame.size() - 3;  // exclude ':' and "\r\n"
  if (payload_len % 2 != 0) {
    ESP_LOGW(TAG, "[Modbus] Invalid frame length");
    return false;
  }

  data.clear();
  data.reserve(payload_len / 2 - 1);  // minus 1 for LRC byte

  uint8_t sum = 0;
  for (size_t i = 1; i < frame.size() - 4; i += 2) {
    uint8_t hi = hex_char_to_val(frame[i]);
    uint8_t lo = hex_char_to_val(frame[i + 1]);
    if (hi == 0xFF || lo == 0xFF) {
      ESP_LOGW(TAG, "[Modbus] Invalid hex chars '%c%c' at pos %zu", frame[i], frame[i + 1], i);
      return false;
    }
    uint8_t byte = (hi << 4) | lo;
    data.push_back(byte);
    sum += byte;
  }

  // Parse received LRC from last two hex chars before \r\n
  uint8_t hi_lrc = hex_char_to_val(frame[frame.size() - 4]);
  uint8_t lo_lrc = hex_char_to_val(frame[frame.size() - 3]);
  if (hi_lrc == 0xFF || lo_lrc == 0xFF) {
    ESP_LOGW(TAG, "[Modbus] Invalid LRC hex chars '%c%c'", frame[frame.size() - 4], frame[frame.size() - 3]);
    return false;
  }
  uint8_t received_lrc = (hi_lrc << 4) | lo_lrc;

  uint8_t computed_lrc = static_cast<uint8_t>(-sum);

  if (computed_lrc != received_lrc) {
    ESP_LOGW(TAG, "[Modbus] LRC mismatch: computed=0x%02X received=0x%02X", computed_lrc, received_lrc);
    return false;
  }

  ESP_LOGD(TAG, "[Modbus] Decoded ASCII frame successfully");

  return true;
}

// --- Read Holding Registers --
bool ModbusAsciiHandler::read_register(uint8_t address, uint16_t reg, uint8_t count, std::vector<uint16_t> &response) {
  std::vector<uint8_t> request = {
    address, 0x03,
    static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
    0x00, count
  };
  std::vector<uint8_t> raw_response;

  ESP_LOGD(TAG, "[Modbus] Sending read request: addr=0x%02X reg=0x%04X count=%d", address, reg, count);

  if (!send_and_receive(request, raw_response) || raw_response.size() < 3 + 2 * count) {
    ESP_LOGW(TAG, "[Modbus] Read failed for addr=0x%02X reg=0x%04X count=%d. Retrying once...", address, reg, count);
    delay(10);
    if (!send_and_receive(request, raw_response) || raw_response.size() < 3 + 2 * count)
      return false;
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

  ESP_LOGD(TAG, "[Modbus] Read success: addr=0x%02X reg=0x%04X count=%d", address, reg, count);
  return true;
}

// --- Public Modbus API (async) ---
void ModbusAsciiHandler::read_register(uint8_t address, uint16_t reg, uint16_t count,
                                       std::function<void(bool success, std::vector<uint16_t>)> callback) {
  ModbusRequest req;
  req.address = address;
  req.function = 0x03;  // Read Holding Registers
  req.start_register = reg;
  req.length_or_value = count;
  req.is_write = false;
  req.callback = std::move(callback);

  this->add_request(req);
}

void ModbusAsciiHandler::write_register(uint8_t address, uint16_t reg, uint16_t value,
                                        std::function<void(bool success, std::vector<uint16_t>)> callback) {
  ModbusRequest req;
  req.address = address;
  req.function = 0x06;  // Write Single Register
  req.start_register = reg;
  req.length_or_value = value;
  req.is_write = true;
  req.callback = std::move(callback);

  this->add_request(req);
}

// --- Write Single Holding Register ---
bool ModbusAsciiHandler::write_register(uint8_t address, uint16_t reg, uint16_t value) {
  std::vector<uint8_t> request = {
    address, 0x06,
    static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0xFF),
    static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)
  };
  std::vector<uint8_t> response;

  ESP_LOGD(TAG, "[Modbus] Sending write request: addr=0x%02X reg=0x%04X value=0x%04X", address, reg, value);

  if (!send_and_receive(request, response)) {
    ESP_LOGW(TAG, "[Modbus] Write failed: addr=0x%02X reg=0x%04X", address, reg);
    return false;
  }

  // Typical Modbus response for write is echo of request (6 bytes)
  if (response.size() < 6 ||
      response[0] != address || 
      response[1] != 0x06 || 
      response[2] != (reg >> 8) || 
      response[3] != (reg & 0xFF) || 
      response[4] != (value >> 8) || 
      response[5] != (value & 0xFF)) {
    ESP_LOGW(TAG, "[Modbus] Unexpected write response");
    return false;
  }

  ESP_LOGD(TAG, "[Modbus] Write success: addr=0x%02X reg=0x%04X value=0x%04X", address, reg, value);
  return true;
}

// --- FSM: Frame Builder ---
std::vector<uint8_t> ModbusAsciiHandler::build_request_frame_ascii_(const std::vector<uint8_t> &data) {
  std::string ascii = this->encode_ascii_frame(data);
  return std::vector<uint8_t>(ascii.begin(), ascii.end());
}

// --- FSM: Request Queue ---
void ModbusAsciiHandler::add_request(ModbusRequest request) {
  this->request_queue_.push(std::move(request));
  ESP_LOGD(TAG, "[FSM] Request enqueued (fn=0x%02X reg=0x%04X)", request.function, request.start_register);
}

// --- Full Modbus Transaction ---
bool ModbusAsciiHandler::send_and_receive(const std::vector<uint8_t> &request, std::vector<uint8_t> &response) {
  constexpr size_t MAX_FRAME_SIZE = 256;

  ESP_LOGW(TAG, "send_and_receive() entered");

  // Log request content and size
  ESP_LOGW(TAG, "request size: %zu bytes", request.size());
  for (size_t i = 0; i < request.size(); i++) {
    ESP_LOGW(TAG, "  - Byte[%zu] = 0x%02X", i, request[i]);
  }

  if (!this->uart_) {
    ESP_LOGW(TAG, "UART component not initialized!");
    return false;
  }

  // Flush leftover bytes from RX buffer before sending
  int flushed = 0;
  while (this->uart_->available()) {
    uint8_t dummy;
    this->uart_->read_byte(&dummy);
    flushed++;
  }
  if (flushed > 0) {
    ESP_LOGW(TAG, "Flushed %d stale bytes from UART RX buffer before TX", flushed);
  }

  std::string frame = encode_ascii_frame(request);

  // Prepare display string without trailing \r\n for cleaner logging
  std::string display_frame = frame;
  while (!display_frame.empty() && 
         (display_frame.back() == '\r' || display_frame.back() == '\n')) {
    display_frame.pop_back();
  }

  ESP_LOGD(TAG, "TX frame generated (%zu bytes): [%s]", frame.size(), display_frame.c_str());

  // Enable TX mode (set RE/DE pins)
  this->set_direction(true);
  delay(2);  // Allow pins to settle

  this->uart_->write_str(frame.c_str());
  ESP_LOGW(TAG, "Writing to UART: %s", frame.c_str());
  this->uart_->flush();
  delay(5);  // Allow UART hardware to finish sending

  // Enable RX mode
  this->set_direction(false);
  delay(25);  // Quiet line time before reading response

  std::string buffer;
  uint32_t timeout = std::max(this->timeout_ms_, 50U);  // Use configured timeout, min 50 ms
  ESP_LOGD(TAG, "Using Modbus response timeout: %u ms", timeout);

  uint32_t start_time = millis();

  while (millis() - start_time < timeout) {
    uint8_t byte;
    while (this->uart_->read_byte(&byte)) {
      if (byte == 0x00)  // Skip null bytes
        continue;

      buffer += static_cast<char>(byte);

      if (buffer.length() > MAX_FRAME_SIZE) {
        ESP_LOGW(TAG, "Received frame too long (%zu bytes). Aborting read.", buffer.length());
        break;
      }

      // Detect end of Modbus ASCII frame (CR LF)
      if (buffer.size() >= 2 && buffer[buffer.size() - 2] == '\r' && buffer[buffer.size() - 1] == '\n') {
        goto decode;
      }
    }
    delay(1);  // Prevent tight loop
  }

decode:
  // Clean trailing CR LF for logging
  std::string display_rx = buffer;
  if (display_rx.size() >= 2 &&
      display_rx[display_rx.size() - 2] == '\r' &&
      display_rx[display_rx.size() - 1] == '\n') {
    display_rx.resize(display_rx.size() - 2);
  }

  ESP_LOGD(TAG, "RX raw frame: '%s'", display_rx.c_str());

  if (buffer.empty()) {
    ESP_LOGW(TAG, "No response received");
    return false;
  }

  bool ok = decode_ascii_frame(buffer, response);
  if (!ok) {
    ESP_LOGW(TAG, "Failed to decode response: '%s'", buffer.c_str());
  }

  return ok;
}

// --- FSM: Loop ---
void ModbusAsciiHandler::loop() {
  switch (this->fsm_state_) {
    case ModbusState::IDLE:
      if (!this->request_queue_.empty()) {
        this->current_request_ = this->request_queue_.front();
        this->request_queue_.pop();
        ESP_LOGD(TAG, "[FSM] Transition: IDLE → SEND_REQUEST");
        this->fsm_state_ = ModbusState::SEND_REQUEST;
      }
      break;

    case ModbusState::SEND_REQUEST:
      {
        std::vector<uint8_t> request_pdu;

        request_pdu.push_back(this->current_request_.address);
        request_pdu.push_back(this->current_request_.function);
        request_pdu.push_back((this->current_request_.start_register >> 8) & 0xFF);
        request_pdu.push_back(this->current_request_.start_register & 0xFF);

        if (this->current_request_.is_write) {
          request_pdu.push_back((this->current_request_.length_or_value >> 8) & 0xFF);
          request_pdu.push_back(this->current_request_.length_or_value & 0xFF);
        } else {
          request_pdu.push_back((this->current_request_.length_or_value >> 8) & 0xFF);
          request_pdu.push_back(this->current_request_.length_or_value & 0xFF);
        }

        auto frame_ascii = this->build_request_frame_ascii_(request_pdu);
        ESP_LOGD(TAG, "[FSM] TX (ASCII): %s", std::string(frame_ascii.begin(), frame_ascii.end()).c_str());

        if (this->uart_) {
          for (auto b : frame_ascii) {
            this->uart_->write_byte(b);
          }
        }

        this->fsm_start_time_ = millis();
        this->fsm_state_ = ModbusState::WAIT_RESPONSE;
      }
      break;

    case ModbusState::WAIT_RESPONSE:
      if (this->read_available_()) {
        ESP_LOGD(TAG, "[FSM] Response ready, processing");
        this->fsm_state_ = ModbusState::PROCESS_RESPONSE;
      } else if (millis() - this->fsm_start_time_ > fsm_timeout_ms_) {
        ESP_LOGW(TAG, "[FSM] Timeout waiting for response");
        this->fsm_state_ = ModbusState::ERROR;
      }
      break;

    case ModbusState::PROCESS_RESPONSE:
      {
        std::vector<uint8_t> response;
        std::string raw(reinterpret_cast<char *>(this->rx_buffer_.data()), this->rx_buffer_.size());
        this->rx_buffer_.clear();

        if (!this->decode_ascii_frame(raw, response)) {
          ESP_LOGW(TAG, "[FSM] Invalid ASCII frame");
          this->fsm_state_ = ModbusState::ERROR;
          break;
        }

        if (response.size() < 2 || response[0] != this->current_request_.address) {
          ESP_LOGW(TAG, "[FSM] Address mismatch in response");
          this->fsm_state_ = ModbusState::ERROR;
          break;
        }

        std::vector<uint16_t> result;

        if (this->current_request_.function == 0x03 || this->current_request_.function == 0x04) {
          if (response.size() < 3) {
            ESP_LOGW(TAG, "[FSM] Response too short");
            this->fsm_state_ = ModbusState::ERROR;
            break;
          }

          uint8_t byte_count = response[2];
          for (int i = 0; i < byte_count; i += 2) {
            if (3 + i + 1 >= response.size()) break;
            uint16_t val = (response[3 + i] << 8) | response[3 + i + 1];
            result.push_back(val);
          }
        }

        if (this->current_request_.callback)
          this->current_request_.callback(true, result);

        this->fsm_state_ = ModbusState::IDLE;
      }
      break;

    case ModbusState::ERROR:
      ESP_LOGE(TAG, "[FSM] Error during request");
      if (this->current_request_.callback)
        this->current_request_.callback(false, {});
      this->fsm_state_ = ModbusState::IDLE;
      break;
  }
}

bool ModbusAsciiHandler::read_available_() {
  // Check if UART has a complete frame (ends in \n)
  if (!this->uart_ || !this->uart_->available())
    return false;

  while (this->uart_->available()) {
    uint8_t byte;
    if (!this->uart_->read_byte(&byte)) break;

    this->rx_buffer_.push_back(byte);

    if (byte == '\n') {
      return true;  // End of ASCII frame
    }

    // Protect against runaway buffer
    if (this->rx_buffer_.size() > 128) {
      this->rx_buffer_.clear();
      ESP_LOGW(TAG, "[FSM] RX overflow, buffer cleared");
      break;
    }
  }

  return false;
}

}  // namespace olimpia_bridge
}  // namespace esphome

// --- OLIMPIA BRIDGE COMPONENT IMPLEMENTATION ---
#include "olimpia_bridge.h"
#include "esphome/core/log.h"

namespace esphome {
namespace olimpia_bridge {

static const char *const TAG = "olimpia_bridge";

void OlimpiaBridge::setup() {
  ESP_LOGI(TAG, "Setting up Olimpia Bridge");

  // Set RE and DE pins (no direction pin check)
  if (this->re_pin_ != nullptr && this->de_pin_ != nullptr) {
    this->re_pin_->setup();
    this->de_pin_->setup();
    this->re_pin_->digital_write(false);  // RX mode
    this->de_pin_->digital_write(false);  // RX mode
  } else {
    ESP_LOGE(TAG, "No RE/DE pair configured.");
    this->mark_failed();
    return;
  }

  // Set UART for handler
  if (this->handler_) {
    this->handler_->set_uart(this->uart_);
    this->handler_->set_re_pin(this->re_pin_);
    this->handler_->set_de_pin(this->de_pin_);
  }

  // Register API services
  this->register_service(&OlimpiaBridge::read_register,
                         "olimpia_read_register",
                         {"address", "register"});
  this->register_service(&OlimpiaBridge::write_register,
                           "olimpia_write_config_register",
                           {"address", "register", "value"});

  ESP_LOGI(TAG, "OlimpiaBridge setup complete");
}

void OlimpiaBridge::update() {
  // Nothing to poll by default – could trigger automatic register reads here if needed
}

void OlimpiaBridge::read_register(int address, int reg) {
}

void OlimpiaBridge::write_register(int address, int reg, int value) {
}

}  // namespace olimpia_bridge
}  // namespace esphome

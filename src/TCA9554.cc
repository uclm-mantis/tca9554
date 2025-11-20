#include "TCA9554.hh"
#include "esp_log.h"

static const char *TAG = "TCA9554_CPP";

// ==========================================
// Constructor / Destructor
// ==========================================

TCA9554::TCA9554(i2c_port_t i2c_port, uint8_t dev_addr) {
    device = TCA9554_create(i2c_port, dev_addr);
    if (!device) {
        esp_err_t err = ESP_FAIL;
        TCA9554_ERROR_IF_NOT_OK(err);
    }
    ESP_LOGI(TAG, "TCA9554 C++ wrapper initialized");
}

TCA9554::~TCA9554() {
    if (device) {
        TCA9554_destroy(device);
        device = nullptr;
    }
}

// ==========================================
// Move Semantics
// ==========================================

TCA9554::TCA9554(TCA9554&& other) noexcept : device(other.device) {
    other.device = nullptr;
}

TCA9554& TCA9554::operator=(TCA9554&& other) noexcept {
    if (this != &other) {
        if (device) {
            TCA9554_destroy(device);
        }
        device = other.device;
        other.device = nullptr;
    }
    return *this;
}

// ==========================================
// Pin Mode Configuration
// ==========================================

void TCA9554::setPinMode(TCA9554_Pin_t pin, TCA9554_PinMode_t mode) {
    esp_err_t err = TCA9554_set_pin_mode(device, pin, mode);
    TCA9554_ERROR_IF_NOT_OK(err);
}

void TCA9554::setPortMode(uint8_t mode_mask) {
    esp_err_t err = TCA9554_set_port_mode(device, mode_mask);
    TCA9554_ERROR_IF_NOT_OK(err);
}

void TCA9554::pinMode(TCA9554_Pin_t pin, uint8_t mode) {
    // Arduino-style function name for compatibility
    if (mode == INPUT) {
        setPinMode(pin, TCA9554_MODE_INPUT);
    } else {
        setPinMode(pin, TCA9554_MODE_OUTPUT);
    }
}

// ==========================================
// Digital I/O Operations
// ==========================================

void TCA9554::digitalWrite(TCA9554_Pin_t pin, uint8_t value) {
    esp_err_t err = TCA9554_digital_write(device, pin, value);
    TCA9554_ERROR_IF_NOT_OK(err);
}

uint8_t TCA9554::digitalRead(TCA9554_Pin_t pin) {
    uint8_t value;
    esp_err_t err = TCA9554_digital_read(device, pin, &value);
    TCA9554_ERROR_IF_NOT_OK(err);
    return value;
}

void TCA9554::writePort(uint8_t value) {
    esp_err_t err = TCA9554_write_port(device, value);
    TCA9554_ERROR_IF_NOT_OK(err);
}

uint8_t TCA9554::readPort() {
    uint8_t value;
    esp_err_t err = TCA9554_read_port(device, &value);
    TCA9554_ERROR_IF_NOT_OK(err);
    return value;
}

// ==========================================
// Polarity Inversion
// ==========================================

void TCA9554::setPolarity(TCA9554_Pin_t pin, uint8_t inverted) {
    esp_err_t err = TCA9554_set_polarity(device, pin, inverted);
    TCA9554_ERROR_IF_NOT_OK(err);
}

void TCA9554::setPortPolarity(uint8_t polarity_mask) {
    esp_err_t err = TCA9554_set_port_polarity(device, polarity_mask);
    TCA9554_ERROR_IF_NOT_OK(err);
}

// ==========================================
// Register Access
// ==========================================

uint8_t TCA9554::readRegister(TCA9554_Reg_t reg) {
    uint8_t value;
    esp_err_t err = TCA9554_read_register(device, reg, &value);
    TCA9554_ERROR_IF_NOT_OK(err);
    return value;
}

void TCA9554::writeRegister(TCA9554_Reg_t reg, uint8_t value) {
    esp_err_t err = TCA9554_write_register(device, reg, value);
    TCA9554_ERROR_IF_NOT_OK(err);
}

// ==========================================
// Device Information
// ==========================================

i2c_port_t TCA9554::getI2CPort() const {
    return device ? device->i2c_port : static_cast<i2c_port_t>(-1);
}

uint8_t TCA9554::getDeviceAddress() const {
    return device ? device->dev_addr : 0;
}

bool TCA9554::isInitialized() const {
    return device != nullptr;
}

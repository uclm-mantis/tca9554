#pragma once

#include <stdint.h>
#include <memory>
#include "tca9554.h"

#ifndef ENABLE_TCA9554_EXCEPTIONS
#include "esp_debug_helpers.h"
#endif
#ifdef ENABLE_TCA9554_EXCEPTIONS
#   define TCA9554_ERROR_IF_NOT_OK(ec) if (ec != ESP_OK) throw ec
#else
#   define TCA9554_ERROR_IF_NOT_OK(ec) if (ec != ESP_OK) { esp_backtrace_print(32); abort(); }
#endif

/**
 * @brief C++ wrapper class for TCA9554 I/O Expander
 * 
 * This class provides an object-oriented interface to the TCA9554 8-bit
 * I2C I/O expander. It manages the device lifecycle and provides convenient
 * methods for pin and port operations.
 * 
 * Usage example:
 * @code
 * TCA9554 expander(I2C_NUM_0, 0x20);
 * expander.setPinMode(TCA9554_PIN0, TCA9554_MODE_OUTPUT);
 * expander.digitalWrite(TCA9554_PIN0, 1);
 * @endcode
 */
class TCA9554 {
public:
    /**
     * @brief Constructor - Creates and initializes a TCA9554 device instance.
     * 
     * This constructor creates a TCA9554 device instance on an existing I2C bus.
     * The I2C bus must be initialized separately using i2c_driver_install() before
     * creating a TCA9554 instance. Multiple TCA9554 instances can share the same
     * I2C bus with different device addresses.
     * 
     * By default, all pins are configured as inputs (power-on reset state).
     * 
     * @param i2c_port I2C port number (e.g., I2C_NUM_0, I2C_NUM_1)
     * @param dev_addr I2C device address (0x20-0x27, based on A0, A1, A2 pins)
     * 
     * @note Error handling: If ENABLE_TCA9554_EXCEPTIONS is defined, throws esp_err_t.
     *       Otherwise, prints backtrace and aborts on error.
     */
    TCA9554(i2c_port_t i2c_port, uint8_t dev_addr);

    /**
     * @brief Destructor - Automatically cleans up resources
     */
    ~TCA9554();

    /**
     * @brief Disallow copy constructor
     */
    TCA9554(const TCA9554&) = delete;

    /**
     * @brief Disallow copy assignment
     */
    TCA9554& operator=(const TCA9554&) = delete;

    /**
     * @brief Allow move constructor
     */
    TCA9554(TCA9554&& other) noexcept;

    /**
     * @brief Allow move assignment
     */
    TCA9554& operator=(TCA9554&& other) noexcept;

    // ==========================================
    // Pin Mode Configuration
    // ==========================================

    /**
     * @brief Sets the mode (input/output) for a single pin.
     * 
     * @param pin Pin number (0-7)
     * @param mode Pin mode (TCA9554_MODE_INPUT or TCA9554_MODE_OUTPUT)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void setPinMode(TCA9554_Pin_t pin, TCA9554_PinMode_t mode);

    /**
     * @brief Sets the mode (input/output) for all 8 pins at once.
     * 
     * @param mode_mask 8-bit mask where 1=input, 0=output (bit 0 = pin 0, bit 7 = pin 7)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void setPortMode(uint8_t mode_mask);

    /**
     * @brief Configures a pin as an output.
     * 
     * Convenience method equivalent to setPinMode(pin, TCA9554_MODE_OUTPUT).
     * 
     * @param pin Pin number (0-7)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void pinMode(TCA9554_Pin_t pin, uint8_t mode);

    // ==========================================
    // Digital I/O Operations
    // ==========================================

    /**
     * @brief Writes a digital value to a single output pin.
     * 
     * The pin should be configured as an output before calling this function.
     * 
     * @param pin Pin number (0-7)
     * @param value Output value (0 = LOW, non-zero = HIGH)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void digitalWrite(TCA9554_Pin_t pin, uint8_t value);

    /**
     * @brief Reads the digital value from a single input pin.
     * 
     * This reads the actual logic level on the pin, regardless of whether it's
     * configured as input or output.
     * 
     * @param pin Pin number (0-7)
     * @return uint8_t The pin value (0 = LOW, 1 = HIGH)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    uint8_t digitalRead(TCA9554_Pin_t pin);

    /**
     * @brief Writes values to all 8 output pins at once.
     * 
     * Only affects pins configured as outputs.
     * 
     * @param value 8-bit value to write (bit 0 = pin 0, bit 7 = pin 7)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void writePort(uint8_t value);

    /**
     * @brief Reads the values from all 8 pins at once.
     * 
     * Reads the actual logic levels on all pins.
     * 
     * @return uint8_t 8-bit value (bit 0 = pin 0, bit 7 = pin 7)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    uint8_t readPort();

    // ==========================================
    // Polarity Inversion
    // ==========================================

    /**
     * @brief Sets the polarity inversion for a single pin.
     * 
     * When polarity inversion is enabled, the value read from the Input Port Register
     * will be inverted from the actual pin state.
     * 
     * @param pin Pin number (0-7)
     * @param inverted 0 = normal polarity, 1 = inverted polarity
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void setPolarity(TCA9554_Pin_t pin, uint8_t inverted);

    /**
     * @brief Sets the polarity inversion for all 8 pins at once.
     * 
     * @param polarity_mask 8-bit mask where 1=inverted, 0=normal (bit 0 = pin 0, bit 7 = pin 7)
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void setPortPolarity(uint8_t polarity_mask);

    // ==========================================
    // Register Access
    // ==========================================

    /**
     * @brief Reads a single register from the TCA9554.
     * 
     * This provides direct access to device registers for advanced use cases.
     * 
     * @param reg Register address to read
     * @return uint8_t The register value
     * 
     * @throws std::runtime_error on I2C communication error
     */
    uint8_t readRegister(TCA9554_Reg_t reg);

    /**
     * @brief Writes a single register to the TCA9554.
     * 
     * This provides direct access to device registers for advanced use cases.
     * 
     * @param reg Register address to write
     * @param value Value to write
     * 
     * @throws std::runtime_error on I2C communication error
     */
    void writeRegister(TCA9554_Reg_t reg, uint8_t value);

    // ==========================================
    // Device Information
    // ==========================================

    /**
     * @brief Returns the I2C port used by this device.
     * 
     * @return i2c_port_t The I2C port number
     */
    i2c_port_t getI2CPort() const;

    /**
     * @brief Returns the I2C device address.
     * 
     * @return uint8_t The device address (0x20-0x27)
     */
    uint8_t getDeviceAddress() const;

    /**
     * @brief Checks if the device is initialized and accessible.
     * 
     * @return bool true if device is accessible, false otherwise
     */
    bool isInitialized() const;

private:
    TCA9554_t* device;  ///< Pointer to the underlying C device structure
};

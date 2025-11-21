#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 *  Register Definitions
 * =========================
 */

/**
 * @brief TCA9554 register addresses
 */
typedef enum {
    TCA9554_REG_INPUT    = 0x00,  /**< Input Port Register (read-only) */
    TCA9554_REG_OUTPUT   = 0x01,  /**< Output Port Register (read/write) */
    TCA9554_REG_POLARITY = 0x02,  /**< Polarity Inversion Register (read/write) */
    TCA9554_REG_CONFIG   = 0x03   /**< Configuration Register (read/write, 1=input, 0=output) */
} TCA9554_Reg_t;

/* =========================
 *  Pin Definitions
 * =========================
 */

/**
 * @brief TCA9554 pin numbers (0-7)
 */
typedef enum {
    TCA9554_PIN0 = 0,
    TCA9554_PIN1 = 1,
    TCA9554_PIN2 = 2,
    TCA9554_PIN3 = 3,
    TCA9554_PIN4 = 4,
    TCA9554_PIN5 = 5,
    TCA9554_PIN6 = 6,
    TCA9554_PIN7 = 7
} TCA9554_Pin_t;

/**
 * @brief Pin mode configuration
 */
typedef enum {
    TCA9554_MODE_OUTPUT = 0,  /**< Configure pin as output */
    TCA9554_MODE_INPUT  = 1   /**< Configure pin as input */
} TCA9554_PinMode_t;



/* =========================
 *  API Functions
 * =========================
 */

/**
 * @brief Opaque handle for TCA9554 device instance
 */
typedef struct TCA9554_t TCA9554_t;

/**
 * @brief Creates and initializes a TCA9554 device instance.
 * 
 * This function creates a TCA9554 device instance on an existing I2C bus. The I2C bus must be
 * initialized separately using i2c_driver_install() before calling this function.
 * Multiple TCA9554 instances can share the same I2C bus with different device addresses.
 * 
 * By default, all pins are configured as inputs (power-on reset state).
 * 
 * @param i2c_port I2C port number (e.g., I2C_NUM_0, I2C_NUM_1)
 * @param dev_addr I2C device address (0x20-0x27, based on A0, A1, A2 pins)
 * @return TCA9554_t* Pointer to device instance, or NULL on failure
 */
TCA9554_t* tca9554_create(i2c_port_t i2c_port, uint8_t dev_addr);

/**
 * @brief Destroys a TCA9554 device instance and frees resources.
 * 
 * @param dev Pointer to device instance
 */
void tca9554_destroy(TCA9554_t* dev);

/**
 * @brief Sets the mode (input/output) for a single pin.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param pin Pin number (0-7)
 * @param mode Pin mode (TCA9554_MODE_INPUT or TCA9554_MODE_OUTPUT)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_set_pin_mode(TCA9554_t* dev, TCA9554_Pin_t pin, TCA9554_PinMode_t mode);

/**
 * @brief Sets the mode (input/output) for all 8 pins at once.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param mode_mask 8-bit mask where 1=input, 0=output (bit 0 = pin 0, bit 7 = pin 7)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_set_port_mode(TCA9554_t* dev, uint8_t mode_mask);

/**
 * @brief Writes a digital value to a single output pin.
 * 
 * The pin must be configured as an output before calling this function.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param pin Pin number (0-7)
 * @param value Output value (0 = LOW, non-zero = HIGH)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_digital_write(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t value);

/**
 * @brief Reads the digital value from a single input pin.
 * 
 * This reads the actual logic level on the pin, regardless of whether it's configured
 * as input or output.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param pin Pin number (0-7)
 * @param[out] value Pointer to store the read value (0 or 1)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_digital_read(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t* value);

/**
 * @brief Writes values to all 8 output pins at once.
 * 
 * Only affects pins configured as outputs.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param value 8-bit value to write (bit 0 = pin 0, bit 7 = pin 7)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_write_port(TCA9554_t* dev, uint8_t value);

/**
 * @brief Writes values to all pins specified in a mask at once.
 * 
 * Only affects pins configured as outputs.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param value 8-bit value to write (bit 0 = pin 0, bit 7 = pin 7)
 * @param mask 8-bit value with bits which should be written to (bit 0 = pin 0, bit 7 = pin 7)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_write_port_masked(TCA9554_t* dev, uint8_t value, uint8_t mask);

/**
 * @brief Reads the values from all 8 pins at once.
 * 
 * Reads the actual logic levels on all pins.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param[out] value Pointer to store the 8-bit read value (bit 0 = pin 0, bit 7 = pin 7)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_read_port(TCA9554_t* dev, uint8_t* value);

/**
 * @brief Sets the polarity inversion for a single pin.
 * 
 * When polarity inversion is enabled, the value read from the Input Port Register
 * will be inverted from the actual pin state.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param pin Pin number (0-7)
 * @param inverted 0 = normal polarity, 1 = inverted polarity
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_set_polarity(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t inverted);

/**
 * @brief Sets the polarity inversion for all 8 pins at once.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param polarity_mask 8-bit mask where 1=inverted, 0=normal (bit 0 = pin 0, bit 7 = pin 7)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_set_port_polarity(TCA9554_t* dev, uint8_t polarity_mask);

/**
 * @brief Reads a single register from the TCA9554.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param reg Register address to read
 * @param[out] value Pointer to store the read value
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_read_register(TCA9554_t* dev, TCA9554_Reg_t reg, uint8_t* value);

/**
 * @brief Writes a single register to the TCA9554.
 * 
 * @param dev Pointer to TCA9554 device instance
 * @param reg Register address to write
 * @param value Value to write
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t tca9554_write_register(TCA9554_t* dev, TCA9554_Reg_t reg, uint8_t value);

#ifdef __cplusplus
} /* extern "C" */
#endif

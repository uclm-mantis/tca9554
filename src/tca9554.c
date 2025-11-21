#include "tca9554.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "TCA9554";

/* =========================
 *  Device Structure
 * =========================
 */

struct TCA9554_t {
    i2c_port_t i2c_port;
    uint8_t dev_addr;
    uint8_t value;
};

/* =========================
 *  I2C Helper Functions
 * =========================
 */

/**
 * @brief Reads a single register from the TCA9554 via I2C
 */
static esp_err_t tca9554_read_reg(TCA9554_t* dev, uint8_t reg, uint8_t* value) {
    if (!dev || !value) return ESP_ERR_INVALID_ARG;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);  // Repeated start
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Writes a single register to the TCA9554 via I2C
 */
static esp_err_t tca9554_write_reg(TCA9554_t* dev, uint8_t reg, uint8_t value) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev->dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
    }
    
    return ret;
}

/* =========================
 *  Public API Functions
 * =========================
 */

TCA9554_t* TCA9554_create(i2c_port_t i2c_port, uint8_t dev_addr) {
    if (dev_addr < 0x20 || dev_addr > 0x27) {
        ESP_LOGE(TAG, "Invalid device address 0x%02X (must be 0x20-0x27)", dev_addr);
        return NULL;
    }
    
    TCA9554_t* dev = (TCA9554_t*)calloc(1, sizeof(TCA9554_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate TCA9554 instance");
        return NULL;
    }
    
    dev->i2c_port = i2c_port;
    dev->dev_addr = dev_addr;
    dev->value = 0x00;
    
    // Verify device is accessible by reading the input register
    uint8_t test_val;
    esp_err_t ret = tca9554_read_reg(dev, TCA9554_REG_INPUT, &test_val);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to communicate with TCA9554 at address 0x%02X", dev_addr);
        free(dev);
        return NULL;
    }
    
    ESP_LOGI(TAG, "TCA9554 initialized at address 0x%02X", dev_addr);
    return dev;
}

void TCA9554_destroy(TCA9554_t* dev) {
    if (!dev) return;
    free(dev);
}

esp_err_t TCA9554_set_pin_mode(TCA9554_t* dev, TCA9554_Pin_t pin, TCA9554_PinMode_t mode) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (pin > TCA9554_PIN7) return ESP_ERR_INVALID_ARG;
    
    // Read current configuration
    uint8_t config;
    esp_err_t ret = tca9554_read_reg(dev, TCA9554_REG_CONFIG, &config);
    if (ret != ESP_OK) return ret;
    
    // Modify the specific pin bit
    if (mode == TCA9554_MODE_INPUT) {
        config |= (1 << pin);   // Set bit to 1 for input
    } else {
        config &= ~(1 << pin);  // Clear bit to 0 for output
    }
    
    // Write back the modified configuration
    return tca9554_write_reg(dev, TCA9554_REG_CONFIG, config);
}

esp_err_t TCA9554_set_port_mode(TCA9554_t* dev, uint8_t mode_mask) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return tca9554_write_reg(dev, TCA9554_REG_CONFIG, mode_mask);
}

esp_err_t TCA9554_digital_write(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t value) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (pin > TCA9554_PIN7) return ESP_ERR_INVALID_ARG;
    
    // Modify the specific pin bit
    if (value) {
        dev->value |= (1 << pin);   // Set bit to 1 for HIGH
    } else {
        dev->value &= ~(1 << pin);  // Clear bit to 0 for LOW
    }
    
    // Write back the modified output
    return tca9554_write_reg(dev, TCA9554_REG_OUTPUT, dev->value);
}

esp_err_t TCA9554_digital_read(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t* value) {
    if (!dev || !value) return ESP_ERR_INVALID_ARG;
    if (pin > TCA9554_PIN7) return ESP_ERR_INVALID_ARG;
    
    // Read input register
    uint8_t input;
    esp_err_t ret = tca9554_read_reg(dev, TCA9554_REG_INPUT, &input);
    if (ret != ESP_OK) return ret;
    
    // Extract the specific pin bit
    *value = (input >> pin) & 0x01;
    
    return ESP_OK;
}

esp_err_t TCA9554_write_port(TCA9554_t* dev, uint8_t value) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->value = value;
    return tca9554_write_reg(dev, TCA9554_REG_OUTPUT, value);
}

esp_err_t TCA9554_write_port_masked(TCA9554_t* dev, uint8_t value, uint8_t mask) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->value = (value & mask) | (dev->value & ~mask);
    return tca9554_write_reg(dev, TCA9554_REG_OUTPUT, dev->value);
}


esp_err_t TCA9554_read_port(TCA9554_t* dev, uint8_t* value) {
    if (!dev || !value) return ESP_ERR_INVALID_ARG;
    esp_err_t ret = tca9554_read_reg(dev, TCA9554_REG_INPUT, value);
    if (ret != ESP_OK) return ret;
    dev->value = *value;
    return ESP_OK;
}

esp_err_t TCA9554_set_polarity(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t inverted) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (pin > TCA9554_PIN7) return ESP_ERR_INVALID_ARG;
    
    // Read current polarity register
    uint8_t polarity;
    esp_err_t ret = tca9554_read_reg(dev, TCA9554_REG_POLARITY, &polarity);
    if (ret != ESP_OK) return ret;
    
    // Modify the specific pin bit
    if (inverted) {
        polarity |= (1 << pin);   // Set bit to 1 for inverted
    } else {
        polarity &= ~(1 << pin);  // Clear bit to 0 for normal
    }
    
    // Write back the modified polarity
    return tca9554_write_reg(dev, TCA9554_REG_POLARITY, polarity);
}

esp_err_t TCA9554_set_port_polarity(TCA9554_t* dev, uint8_t polarity_mask) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return tca9554_write_reg(dev, TCA9554_REG_POLARITY, polarity_mask);
}

esp_err_t TCA9554_read_register(TCA9554_t* dev, TCA9554_Reg_t reg, uint8_t* value) {
    if (!dev || !value) return ESP_ERR_INVALID_ARG;
    if (reg > TCA9554_REG_CONFIG) return ESP_ERR_INVALID_ARG;
    return tca9554_read_reg(dev, reg, value);
}

esp_err_t TCA9554_write_register(TCA9554_t* dev, TCA9554_Reg_t reg, uint8_t value) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (reg > TCA9554_REG_CONFIG) return ESP_ERR_INVALID_ARG;
    return tca9554_write_reg(dev, reg, value);
}

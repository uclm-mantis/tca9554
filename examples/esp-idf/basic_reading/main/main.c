#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "tca9554.h"

static const char *TAG = "TCA9554_EXAMPLE";

// I2C Pin Configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ_HZ 100000  // 100 kHz

// TCA9554 I2C Address (A0=A1=A2=GND)
#define TCA9554_ADDR 0x20

// Pin assignments on TCA9554
#define LED_PIN    TCA9554_PIN0
#define BUTTON_PIN TCA9554_PIN1
#define OUTPUT_PIN TCA9554_PIN2
#define INPUT_PIN  TCA9554_PIN3

void app_main(void) {
    ESP_LOGI(TAG, "TCA9554 Basic GPIO Example");
    
    // Initialize I2C bus
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    ESP_LOGI(TAG, "I2C bus initialized");
    
    // Create TCA9554 device instance
    TCA9554_t* gpio_expander = tca9554_create(I2C_NUM_0, TCA9554_ADDR);
    if (!gpio_expander) {
        ESP_LOGE(TAG, "Failed to create TCA9554 device");
        ESP_LOGE(TAG, "Check I2C connections and address!");
        return;
    }
    ESP_LOGI(TAG, "TCA9554 device created at address 0x%02X", TCA9554_ADDR);
    
    // Configure pins
    ESP_ERROR_CHECK(tca9554_set_pin_mode(gpio_expander, LED_PIN, TCA9554_MODE_OUTPUT));
    ESP_ERROR_CHECK(tca9554_set_pin_mode(gpio_expander, BUTTON_PIN, TCA9554_MODE_INPUT));
    ESP_ERROR_CHECK(tca9554_set_pin_mode(gpio_expander, OUTPUT_PIN, TCA9554_MODE_OUTPUT));
    ESP_ERROR_CHECK(tca9554_set_pin_mode(gpio_expander, INPUT_PIN, TCA9554_MODE_INPUT));
    ESP_LOGI(TAG, "Pins configured:");
    ESP_LOGI(TAG, "  Pin %d: OUTPUT (LED)", LED_PIN);
    ESP_LOGI(TAG, "  Pin %d: INPUT (Button)", BUTTON_PIN);
    ESP_LOGI(TAG, "  Pin %d: OUTPUT", OUTPUT_PIN);
    ESP_LOGI(TAG, "  Pin %d: INPUT", INPUT_PIN);
    
    // Set initial output states
    ESP_ERROR_CHECK(tca9554_digital_write(gpio_expander, LED_PIN, 0));
    ESP_ERROR_CHECK(tca9554_digital_write(gpio_expander, OUTPUT_PIN, 0));
    ESP_LOGI(TAG, "Initial outputs set to LOW");
    
    // Optional: Enable polarity inversion on button pin
    // This makes the button read as HIGH when pressed (if button pulls to GND)
    // tca9554_set_polarity(gpio_expander, BUTTON_PIN, 1);
    
    ESP_LOGI(TAG, "Setup complete!");
    ESP_LOGI(TAG, "LED will blink on pin %d", LED_PIN);
    ESP_LOGI(TAG, "Button state on pin %d will be displayed", BUTTON_PIN);
    
    // Main loop
    uint8_t led_state = 0;
    uint8_t last_button_state = 0;
    
    while (1) {
        // Toggle LED
        led_state = !led_state;
        ESP_ERROR_CHECK(tca9554_digital_write(gpio_expander, LED_PIN, led_state));
        ESP_LOGI(TAG, "LED: %s", led_state ? "ON" : "OFF");
        
        // Read button state
        uint8_t button_state;
        if (tca9554_digital_read(gpio_expander, BUTTON_PIN, &button_state) == ESP_OK) {
            if (button_state != last_button_state) {
                ESP_LOGI(TAG, "Button: %s", button_state ? "PRESSED" : "RELEASED");
                last_button_state = button_state;
            }
        }
        
        // Read all pins at once (port-level read)
        uint8_t port_value;
        if (tca9554_read_port(gpio_expander, &port_value) == ESP_OK) {
            ESP_LOGI(TAG, "Port value: 0x%02X", port_value);
        }
        
        // Wait 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Cleanup (never reached in this example)
    tca9554_destroy(gpio_expander);
    i2c_driver_delete(I2C_NUM_0);
}

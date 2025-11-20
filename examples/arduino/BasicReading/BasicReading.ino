/*
 * TCA9554 Basic GPIO Example for Arduino
 * 
 * This example demonstrates:
 * - Initializing the TCA9554 I2C GPIO expander
 * - Configuring pins as inputs and outputs
 * - Reading and writing GPIO pins
 * - Using polarity inversion
 * 
 * Hardware Connections:
 * - SDA   -> GPIO 21
 * - SCL   -> GPIO 22
 * - A0-A2 -> GND (address 0x20)
 * 
 * External connections:
 * - LED on TCA9554 pin 0 (with current-limiting resistor)
 * - Button on TCA9554 pin 1 (pull-up enabled on TCA9554)
 */

#include <Wire.h>
#include "tca9554.h"

// I2C Configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define TCA9554_ADDR 0x20  // A0=A1=A2=GND

// TCA9554 device handle
TCA9554_t* gpio_expander = NULL;

// Pin assignments
#define LED_PIN    TCA9554_PIN0
#define BUTTON_PIN TCA9554_PIN1
#define OUTPUT_PIN TCA9554_PIN2
#define INPUT_PIN  TCA9554_PIN3

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("TCA9554 Basic GPIO Example");
    Serial.println("===========================");
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("I2C initialized");
    
    // Initialize I2C driver (ESP-IDF style for compatibility)
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = (gpio_num_t)I2C_SDA,
        .scl_io_num = (gpio_num_t)I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000  // 100 kHz
        }
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    
    // Create TCA9554 device instance
    gpio_expander = TCA9554_create(I2C_NUM_0, TCA9554_ADDR);
    if (!gpio_expander) {
        Serial.println("ERROR: Failed to create TCA9554 device");
        Serial.println("Check I2C connections and address!");
        while(1) delay(1000);
    }
    Serial.printf("TCA9554 device created at address 0x%02X\n", TCA9554_ADDR);
    
    // Configure pins
    TCA9554_set_pin_mode(gpio_expander, LED_PIN, TCA9554_MODE_OUTPUT);
    TCA9554_set_pin_mode(gpio_expander, BUTTON_PIN, TCA9554_MODE_INPUT);
    TCA9554_set_pin_mode(gpio_expander, OUTPUT_PIN, TCA9554_MODE_OUTPUT);
    TCA9554_set_pin_mode(gpio_expander, INPUT_PIN, TCA9554_MODE_INPUT);
    Serial.println("Pins configured:");
    Serial.printf("  Pin %d: OUTPUT (LED)\n", LED_PIN);
    Serial.printf("  Pin %d: INPUT (Button)\n", BUTTON_PIN);
    Serial.printf("  Pin %d: OUTPUT\n", OUTPUT_PIN);
    Serial.printf("  Pin %d: INPUT\n", INPUT_PIN);
    
    // Set initial output states
    TCA9554_digital_write(gpio_expander, LED_PIN, 0);
    TCA9554_digital_write(gpio_expander, OUTPUT_PIN, 0);
    Serial.println("Initial outputs set to LOW");
    
    // Optional: Enable polarity inversion on button pin
    // This makes the button read as HIGH when pressed (if button pulls to GND)
    // TCA9554_set_polarity(gpio_expander, BUTTON_PIN, 1);
    
    Serial.println("===========================");
    Serial.println("Setup complete!");
    Serial.println("LED will blink on pin 0");
    Serial.println("Button state on pin 1 will be displayed");
    Serial.println("===========================\n");
}

void loop() {
    static uint8_t led_state = 0;
    static uint8_t last_button_state = 0;
    
    // Toggle LED
    led_state = !led_state;
    TCA9554_digital_write(gpio_expander, LED_PIN, led_state);
    Serial.printf("LED: %s\n", led_state ? "ON" : "OFF");
    
    // Read button state
    uint8_t button_state;
    if (TCA9554_digital_read(gpio_expander, BUTTON_PIN, &button_state) == ESP_OK) {
        if (button_state != last_button_state) {
            Serial.printf("Button: %s\n", button_state ? "PRESSED" : "RELEASED");
            last_button_state = button_state;
        }
    }
    
    // Read all pins at once (port-level read)
    uint8_t port_value;
    if (TCA9554_read_port(gpio_expander, &port_value) == ESP_OK) {
        Serial.printf("Port value: 0x%02X (binary: ", port_value);
        for (int i = 7; i >= 0; i--) {
            Serial.print((port_value >> i) & 1);
        }
        Serial.println(")");
    }
    
    Serial.println("---");
    delay(1000);
}

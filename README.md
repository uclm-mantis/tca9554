# TCA9554 Driver for ESP32

Multi-instance driver for the Texas Instruments TCA9554 8-bit I2C GPIO expander. Compatible with both ESP-IDF and Arduino IDE.

## Features

- ✅ **Multi-instance support** - Control multiple TCA9554 devices on the same I2C bus
- ✅ **Simple GPIO API** - Familiar digital read/write interface
- ✅ **Port-level operations** - Read/write all 8 pins at once
- ✅ **Polarity inversion** - Hardware-level input polarity control
- ✅ **Full register control** - Direct access to all TCA9554 registers
- ✅ **Dual platform support** - Works with ESP-IDF and Arduino IDE

## Hardware Connections

| TCA9554 Pin | ESP32 Pin | Description |
|-------------|-----------|-------------|
| SDA         | GPIO 21   | I2C Data (configurable) |
| SCL         | GPIO 22   | I2C Clock (configurable) |
| A0          | GND/VCC   | Address bit 0 |
| A1          | GND/VCC   | Address bit 1 |
| A2          | GND/VCC   | Address bit 2 |
| VCC         | 3.3V      | Supply voltage (1.65V - 5.5V) |
| GND         | GND       | Ground |
| P0-P7       | -         | GPIO pins (input/output) |

**I2C Address:** 0x20 + (A2<<2) + (A1<<1) + A0
- All address pins to GND: 0x20
- All address pins to VCC: 0x27

## Installation

### ESP-IDF

#### Method 1: Managed Component (Recommended)

Add the component as a dependency using the ESP-IDF Component Manager. This method automatically manages updates and dependencies.

1. Create or edit `main/idf_component.yml` in your project:
   ```yaml
   dependencies:
     tca9554:
       git: https://github.com/uclm-mantis/tca9554.git
       version: ">=1.0.0"
   ```

2. Build your project:
   ```bash
   idf.py build
   ```

3. The component will be automatically downloaded to `managed_components/tca9554/`

4. Include in your code:
   ```c
   #include "tca9554.h"
   ```

**Alternative: Using a specific branch or commit:**
```yaml
dependencies:
  tca9554:
    git: https://github.com/uclm-mantis/tca9554.git
    path: "."
    version: main  # or a specific commit hash
```

#### Method 2: Manual Installation

1. Copy this repository to your project's `components/` directory:
   ```
   your_project/
   ├── components/
   │   └── tca9554/
   ├── main/
   └── CMakeLists.txt
   ```

2. The component will be automatically detected and linked by ESP-IDF

### Arduino IDE

#### Method 1: Install from ZIP (Recommended)

1. Download the repository as a ZIP file:
   - Go to [https://github.com/uclm-mantis/tca9554](https://github.com/uclm-mantis/tca9554)
   - Click the green **Code** button
   - Select **Download ZIP**

2. In Arduino IDE, go to **Sketch → Include Library → Add .ZIP Library...**

3. Select the downloaded `tca9554-main.zip` file

4. The library will be installed and appear in **Sketch → Include Library → tca9554**

#### Method 2: Manual Installation

1. Copy this repository to your Arduino libraries directory:
   - **Windows**: `Documents\\Arduino\\libraries\\tca9554\\`
   - **macOS**: `~/Documents/Arduino/libraries/tca9554/`
   - **Linux**: `~/Arduino/libraries/tca9554/`

2. Restart Arduino IDE

3. The library will appear in **Sketch → Include Library → tca9554**

## Quick Start

### ESP-IDF Example

```c
#include "tca9554.h"

void app_main(void) {
    // Initialize I2C bus
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    // Create TCA9554 device (address 0x20)
    TCA9554_t* gpio_expander = TCA9554_create(I2C_NUM_0, 0x20);

    // Configure pin 0 as output, pin 1 as input
    TCA9554_set_pin_mode(gpio_expander, TCA9554_PIN0, TCA9554_MODE_OUTPUT);
    TCA9554_set_pin_mode(gpio_expander, TCA9554_PIN1, TCA9554_MODE_INPUT);

    // Write to output pin
    TCA9554_digital_write(gpio_expander, TCA9554_PIN0, 1);

    // Read from input pin
    uint8_t value;
    TCA9554_digital_read(gpio_expander, TCA9554_PIN1, &value);
    printf("Pin 1 value: %d\\n", value);
}
```

### Arduino Example

```cpp
#include <Wire.h>
#include "tca9554.h"

TCA9554_t* gpio_expander;

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C
    Wire.begin(21, 22);  // SDA, SCL
    
    // Create TCA9554 device (address 0x20)
    gpio_expander = TCA9554_create(I2C_NUM_0, 0x20);
    
    // Configure pin 0 as output, pin 1 as input
    TCA9554_set_pin_mode(gpio_expander, TCA9554_PIN0, TCA9554_MODE_OUTPUT);
    TCA9554_set_pin_mode(gpio_expander, TCA9554_PIN1, TCA9554_MODE_INPUT);
}

void loop() {
    // Toggle output pin
    static uint8_t state = 0;
    TCA9554_digital_write(gpio_expander, TCA9554_PIN0, state);
    state = !state;
    
    // Read input pin
    uint8_t value;
    TCA9554_digital_read(gpio_expander, TCA9554_PIN1, &value);
    Serial.print("Pin 1: ");
    Serial.println(value);
    
    delay(1000);
}
```

## API Reference

### Device Management

#### `TCA9554_create()`
Creates and initializes a TCA9554 device instance.

```c
TCA9554_t* TCA9554_create(i2c_port_t i2c_port, uint8_t dev_addr);
```

**Parameters:**
- `i2c_port` - I2C port number (e.g., I2C_NUM_0, I2C_NUM_1)
- `dev_addr` - I2C device address (0x20-0x27)

**Returns:** Device handle or `NULL` on failure

---

#### `TCA9554_destroy()`
Destroys a TCA9554 device instance and frees resources.

```c
void TCA9554_destroy(TCA9554_t* dev);
```

---

### Pin Configuration

#### `TCA9554_set_pin_mode()`
Sets the mode (input/output) for a single pin.

```c
esp_err_t TCA9554_set_pin_mode(TCA9554_t* dev, TCA9554_Pin_t pin, TCA9554_PinMode_t mode);
```

**Parameters:**
- `pin` - Pin number (TCA9554_PIN0 to TCA9554_PIN7)
- `mode` - TCA9554_MODE_INPUT or TCA9554_MODE_OUTPUT

---

#### `TCA9554_set_port_mode()`
Sets the mode for all 8 pins at once.

```c
esp_err_t TCA9554_set_port_mode(TCA9554_t* dev, uint8_t mode_mask);
```

**Parameters:**
- `mode_mask` - 8-bit mask where 1=input, 0=output

---

### Digital I/O

#### `TCA9554_digital_write()`
Writes a digital value to a single output pin.

```c
esp_err_t TCA9554_digital_write(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t value);
```

**Parameters:**
- `pin` - Pin number (TCA9554_PIN0 to TCA9554_PIN7)
- `value` - 0 for LOW, non-zero for HIGH

---

#### `TCA9554_digital_read()`
Reads the digital value from a single pin.

```c
esp_err_t TCA9554_digital_read(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t* value);
```

**Parameters:**
- `pin` - Pin number (TCA9554_PIN0 to TCA9554_PIN7)
- `value` - Pointer to store the read value (0 or 1)

---

#### `TCA9554_write_port()`
Writes values to all 8 pins at once.

```c
esp_err_t TCA9554_write_port(TCA9554_t* dev, uint8_t value);
```

**Parameters:**
- `value` - 8-bit value (bit 0 = pin 0, bit 7 = pin 7)

---

#### `TCA9554_read_port()`
Reads values from all 8 pins at once.

```c
esp_err_t TCA9554_read_port(TCA9554_t* dev, uint8_t* value);
```

**Parameters:**
- `value` - Pointer to store the 8-bit read value

---

### Polarity Inversion

#### `TCA9554_set_polarity()`
Sets the polarity inversion for a single pin.

```c
esp_err_t TCA9554_set_polarity(TCA9554_t* dev, TCA9554_Pin_t pin, uint8_t inverted);
```

**Parameters:**
- `pin` - Pin number (TCA9554_PIN0 to TCA9554_PIN7)
- `inverted` - 0 for normal polarity, 1 for inverted

**Note:** Polarity inversion only affects input readings. When enabled, HIGH is read as LOW and vice versa.

---

#### `TCA9554_set_port_polarity()`
Sets the polarity inversion for all 8 pins at once.

```c
esp_err_t TCA9554_set_port_polarity(TCA9554_t* dev, uint8_t polarity_mask);
```

**Parameters:**
- `polarity_mask` - 8-bit mask where 1=inverted, 0=normal

---

### Register Access

#### `TCA9554_read_register()`
Reads a single register from the TCA9554.

```c
esp_err_t TCA9554_read_register(TCA9554_t* dev, TCA9554_Reg_t reg, uint8_t* value);
```

---

#### `TCA9554_write_register()`
Writes a single register to the TCA9554.

```c
esp_err_t TCA9554_write_register(TCA9554_t* dev, TCA9554_Reg_t reg, uint8_t value);
```

---

## Advanced Usage

### Multiple Devices

```c
// Device 1 at address 0x20 (A0=A1=A2=GND)
TCA9554_t* expander1 = TCA9554_create(I2C_NUM_0, 0x20);

// Device 2 at address 0x21 (A0=VCC, A1=A2=GND)
TCA9554_t* expander2 = TCA9554_create(I2C_NUM_0, 0x21);

// Configure and use independently
TCA9554_set_pin_mode(expander1, TCA9554_PIN0, TCA9554_MODE_OUTPUT);
TCA9554_set_pin_mode(expander2, TCA9554_PIN0, TCA9554_MODE_OUTPUT);
```

### Port-Level Operations

```c
// Configure all pins at once: pins 0-3 as outputs, pins 4-7 as inputs
TCA9554_set_port_mode(expander, 0b11110000);

// Write to all output pins at once
TCA9554_write_port(expander, 0b00001111);

// Read all input pins at once
uint8_t port_value;
TCA9554_read_port(expander, &port_value);
```

### Polarity Inversion Example

```c
// Configure pin 0 as input
TCA9554_set_pin_mode(expander, TCA9554_PIN0, TCA9554_MODE_INPUT);

// Enable polarity inversion (useful for active-low signals)
TCA9554_set_polarity(expander, TCA9554_PIN0, 1);

// Now when pin is physically LOW, you'll read HIGH
uint8_t value;
TCA9554_digital_read(expander, TCA9554_PIN0, &value);
```

## Troubleshooting

**Problem:** Device not detected
- Check I2C connections (SDA, SCL)
- Verify I2C address matches hardware configuration (A0, A1, A2 pins)
- Use I2C scanner to detect devices on the bus
- Ensure pull-up resistors are present on SDA and SCL (typically 4.7kΩ)

**Problem:** Pins not responding
- Verify pin is configured as output before writing
- Check power supply voltage (1.65V - 5.5V)
- Ensure VCC and GND are properly connected

**Problem:** Build errors in Arduino IDE
- Ensure library is in correct folder
- Restart Arduino IDE after installation
- Check that ESP32 board support is installed

## License

MIT License - see [LICENSE](LICENSE) file for details

## Contributors

- Francisco Moya Fernández
- Fernando Castillo García
- Antonio González Rodríguez
- David Rodríguez Rosa
- Sergio Juárez Pérez
- Andrea Martín Parra
- Raúl Cuadros Tardío
- Juan Sánchez Medina

## References

- [TCA9554 Datasheet](https://www.ti.com/lit/ds/symlink/tca9554.pdf)
- [ESP-IDF I2C Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html)

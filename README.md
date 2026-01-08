# Nixie Clock

A retro-style digital clock built with nixie tubes and an ESP32 microcontroller. This project combines vintage vacuum tube display technology with modern embedded systems to create a unique timekeeping device.

## Features

- **Nixie Tube Display**: Classic vacuum tube display showing time with authentic retro aesthetic
- **Auto Brightness Adjustment**: Light Dependent Resistor (LDR) automatically adjusts display brightness based on ambient light
- **ESP32-based Control**: Powered by an ESP32 microcontroller with dual I2C buses for flexible expansion
- **Dual PCF8574 I/O Expanders**: Manages the BCD-to-decimal decoder logic for tube control
- **RGB LED Status Indicator**: NeoPixel LED for status and visual feedback
- **Cathode Protection**: Periodic nixie tube refresh at 2kHz to prevent cathode poisoning

## Block Diagram

![Nixie Clock Block Diagram](docs/images/Nixie clock block diagram.png)

See [docs/design/](docs/design/) for the detailed block diagram and architecture information.

## Hardware

### Main Components

- **Microcontroller**: ESP32 DevKit-C 32D
- **Nixie Tubes**: 4-digit display (hours and minutes)
  ![Nixie Tube](docs/images/tube image.png)
- **I/O Expansion**: 2x PCF8574 I2C I/O expanders
  ![PCF8574](docs/images/PCF8574 image.png)
- **Decoders**: K155ID1 BCD-to-decimal decoders
  ![K155ND1](docs/images/K155ND1 image.png)
- **Brightness Control**: Light Dependent Resistor (LDR) with analog input
- **Status LED**: Adafruit NeoPixel RGB LED
- **High Voltage Supply**: HV enable pin for nixie tube control

### PCB Boards

The project includes three main PCB designs:

1. **NixieClock_ControlDaughterBoard**: Main control board with ESP32 and I2C interfaces
2. **NixieClock_NixieTubePanel**: Nixie tube display panel with decoder logic
3. **NixieClock_Monoboard** (Archive): Earlier single-board design

### GPIO Pin Configuration

| Pin | Function |
|-----|----------|
| 4   | SDA1 (I2C Bus 1) |
| 15  | SCL1 (I2C Bus 1) |
| 22  | SCL2 (I2C Bus 2) |
| 19  | INT2 (Interrupt, unused) |
| 21  | RGB LED (NeoPixel) |
| 23  | HV Enable (High Voltage supply) |
| 33  | LDR (Analog input) |

## Software Architecture

### Directory Structure

```
src/
├── main.cpp              # Main application logic
├── clock_tick/           # Timer interrupt handling
├── config/               # Configuration management
├── nixie_tubes/          # Nixie tube display control
└── pcf8574/              # I2C I/O expander communication

include/                  # Header files

lib/                      # Local libraries and modules

test/                     # Unit tests
```

### Key Modules

- **clock_tick**: Handles the one-second timer interrupt using ESP32's high-resolution timer for accurate timekeeping
- **nixie_tubes**: Controls tube display with BCD encoding and brightness management
- **pcf8574**: I2C communication with PCF8574 I/O expanders for tube digit control
- **config**: Configuration and constants for pins, timing, and brightness levels

## Getting Started

### Prerequisites

- PlatformIO CLI or VS Code with PlatformIO extension
- Python 3.6+ (for PlatformIO)
- macOS, Linux, or Windows

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Terminal-Curiosity/NixieClock.git
   cd NixieClock
   ```

2. Install PlatformIO (if not already installed):
   ```bash
   pip install platformio
   ```

3. Build the project:
   ```bash
   pio run
   ```

4. Upload to ESP32:
   ```bash
   pio run -t upload
   ```

5. Monitor output (optional):
   ```bash
   pio device monitor
   ```

### Configuration

Edit the configuration constants in [src/main.cpp](src/main.cpp):

- **LDR_BRIGHT_ADC_MIN**: ADC reading threshold for bright light (default: 100)
- **LDR_DARK_ADC_MAX**: ADC reading threshold for dark light (default: 3200)
- **MIN_BRIGHTNESS_PERCENT**: Minimum display brightness (default: 10%)
- **MAX_BRIGHTNESS_PERCENT**: Maximum display brightness (default: 100%)
- **NIXIE_TUBE_REFRESH_TIME**: Refresh period in microseconds (default: 1000μs for 2kHz refresh)

## Technical Details

### Brightness Control

The LDR automatically adjusts display brightness between the configured minimum and maximum levels. The analog reading is mapped from the ADC range to a PWM duty cycle percentage.

### Nixie Tube Control

Digits are controlled via BCD encoding through the K155ID1 decoder:
- Each digit is represented as a 4-bit binary value (0-9)
- PCF8574 I/O expanders convert I2C signals to parallel outputs
- Periodic refresh cycles prevent cathode poisoning in the vacuum tubes

### Timer Interrupt

A high-resolution timer generates a one-second interrupt to increment the clock time without blocking the main loop.

## Dependencies

- **Adafruit NeoPixel**: RGB LED control library
- **Wire**: I2C communication library (built-in to Arduino framework)
- **esp_timer**: ESP32 high-resolution timer library (built-in)

## Project Status

Currently on the **project-restructure** branch, implementing an improved code organization and modularization.

## Documentation

- **[Design Documentation](docs/design/)** - Block diagrams and architectural information
- **[Images](docs/images/)** - Component images and visual references

## License

[Add license information here]

## Contributing

[Add contribution guidelines here]

## References

- [PlatformIO Documentation](https://docs.platformio.org/)
- [ESP32 Arduino Framework](https://github.com/espressif/arduino-esp32)
- [Adafruit NeoPixel Library](https://github.com/adafruit/Adafruit_NeoPixel)
- [K155ID1 Decoder Datasheet](https://en.wikipedia.org/wiki/K155ID1)

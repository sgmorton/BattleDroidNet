# BattleDroidNet

Communication and Control system for HADB BattleDroids.

## Firmware
The firmware is located in `firmware/BattleDroid/`. It is designed for ESP32 boards and uses ESP-NOW for peer-to-peer communication.

### Build Instructions
This project can be compiled and uploaded using `arduino-cli`.

**Dependencies:**
- Adafruit GFX Library
- Adafruit SSD1306

**Compile:**
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 firmware/BattleDroid
```

**Upload:**
```bash
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32 firmware/BattleDroid
```

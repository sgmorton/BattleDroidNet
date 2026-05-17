# BattleDroidNet

Communication and Control system for HADB BattleDroids.

Check out the [Sequence Syntax Guide](SEQUENCES.md) to learn how to create timed droid performances.

## Firmware
The firmware is located in `firmware/BattleDroid/`. It is designed for ESP32 boards and uses ESP-NOW for peer-to-peer communication.

### Build Instructions
This project is compiled and uploaded using `arduino-cli` or the Arduino IDE.

> [!IMPORTANT]
> **Critical Board Package & Hardware Constraints:**
> 1. **ESP32 Core Version `3.3.7` Required:** Core version `3.3.8` contains a regression in standard SPI mapping that breaks SD card operations. You must use version `3.3.7`.
> 2. **PSRAM Enabled:** On ESP32-WROVER boards (like the Makerfabs ESP32), PSRAM must be explicitly enabled during compile. If omitted, the internal cache conflicts with the SPI/SD lines, resulting in an `SD FAIL` on boot.

#### 1. Setup the Environment
To ensure compilation consistency, install and lock the core package version to `3.3.7` via `arduino-cli`:
```bash
# Downgrade/install core 3.3.7
arduino-cli core install esp32:esp32@3.3.7
```

**Required Libraries:**
Ensure the following libraries are installed in your sketchbook directory:
- `Adafruit GFX Library`
- `Adafruit BusIO`
- `Adafruit SSD1306`
- `ESP32-audioI2S-master`
- `ESP32Servo`
- `FastLED`

---

#### 2. Compile Firmware
Compile the BattleDroid sketch using the fully-configured WROVER FQBN:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32:PSRAM=enabled,PartitionScheme=huge_app firmware/BattleDroid
```

---

#### 3. Upload Firmware
Upload the compiled binary to the designated COM port using the same FQBN:
```bash
arduino-cli upload -p <PORT> --fqbn esp32:esp32:esp32:PSRAM=enabled,PartitionScheme=huge_app firmware/BattleDroid
```
*(For example, replacing `<PORT>` with `COM14` or `COM17` depending on the active USB-to-UART bridge).*

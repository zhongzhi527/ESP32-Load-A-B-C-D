# ESP32-Load-A-B-C-D

Firmware for the **load-cell nodes** of the wireless weighing system. Each node
is an ESP32 with a NAU7802 load-cell amplifier and a small SSD1306 OLED. It
reads the weight, auto-locks a stable reading above a threshold, and talks to
the handheld display over **ESP-NOW** — sending weight telemetry and obeying
tare / calibrate / settings commands.

Pairs with the handheld display project
[`Display-Load-A-B-C-D`](https://github.com/zhongzhi527/Display-Load-A-B-C-D).

## Two nodes from one source

Load Cell A and Load Cell B run the **same code**; the identity is chosen at
build time by the `LOAD_CELL_UNIT` flag in `platformio.ini`:

| Environment | `LOAD_CELL_UNIT` | Builds |
|-------------|------------------|--------|
| `load_cell_a` (default) | 1 | Load Cell A |
| `load_cell_b` | 2 | Load Cell B |

Flash `load_cell_a` to the first ESP32 and `load_cell_b` to the second.

## Hardware

| Part | Detail |
|------|--------|
| MCU | ESP32 (`esp32dev`) |
| Load-cell amplifier | SparkFun Qwiic Scale (NAU7802), I2C |
| Screen | SSD1306 128x64 OLED, I2C |
| Wireless | ESP-NOW (Wi-Fi STA) |

### Pin connections

| Signal | GPIO |
|--------|------|
| I2C SDA (OLED + NAU7802) | 23 |
| I2C SCL | 22 |
| LED 1 (lock indicator) | 4 |
| LED 2 (power) | 16 |
| Reset / clear-lock button | 33 |

## How it works

- Reads the NAU7802 every ~200 ms (averaged over several samples).
- When the weight is above `minLockWeightKg` and holds within a 1.0 kg window
  for 4 seconds, the reading **auto-locks** (if auto-lock is enabled).
- Sends a `WeightPacket` to the display each cycle; receives `CommandPacket`
  (tare / calibrate / clear-lock / settings) from the display.
- The physical button clears a lock and resumes measuring.
- Calibration factor and lock settings are saved to flash (`Preferences`).

The packet layouts are shared with the display in
[`include/EspNowProtocol.h`](include/EspNowProtocol.h).

## Pairing

Each node sends to the display's MAC address, set in `src/main.cpp`:

```cpp
uint8_t displayMacAddress[6] = {0x70, 0x4B, 0xCA, 0x76, 0xC4, 0x5C};
```

The display, in turn, listens for each node's MAC. On boot each node prints its
own MAC to the serial monitor (115200 baud), which is what the display must be
configured with.

## Build & flash (PlatformIO)

1. Open this folder in VS Code with the **PlatformIO IDE** extension.
2. Select the `load_cell_a` or `load_cell_b` environment.
3. **Build**, then **Upload** to the matching ESP32.
4. Open the **Serial Monitor** at `115200` baud to read the node's MAC and live
   weight.

### Platform version is pinned on purpose

`platformio.ini` pins `platform = espressif32@6.5.0` (Arduino-ESP32 2.0.x).
Leaving it unpinned pulls the newest platform, whose toolchain can fail to
build. If a build fails right after a clean checkout, confirm the pin is
present and delete the `.pio` folder before rebuilding.

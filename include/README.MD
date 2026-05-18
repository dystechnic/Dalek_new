# NSD-Dalek — ESP32 unified firmware

> **Note:** This firmware was vibe-coded with the assistance of [Claude](https://claude.ai) (Anthropic). The original four-Arduino design was rewritten, consolidated, and iteratively improved. Use it as a starting point, not gospel.

If something catches fire, a Dalek goes rogue, or your cat gets exterminated — that is entirely your problem. We have already moved on to other projects and accept absolutely no responsibility whatsoever.

None.

Good day.

Consolidates the original four-Arduino design (ESP-01, Mega, Pro Mini, Nano) into a single ESP32-WROOM-32U on a DevKitC V4 board.

## Hardware

| Component | Details |
|---|---|
| Microcontroller | ESP32-WROOM-32U on DevKitC V4 |
| Motors | 2× NEMA17 via Big Easy Driver |
| Sensors | 3× Maxbotix EZ1 ultrasonic (daisy-chained) |
| Sound | DFPlayer Mini + speaker |
| LED | WS2811 (eyestalk) |
| Antenna | External FPC 2.4GHz via U.FL connector |

See `include/config.h` for all pin assignments.

## Project structure

```
dalek-esp32/
  data/
    index.html        Web interface (served from LittleFS)
  include/
    config.h          Pin assignments, thresholds, settings
  src/
    main.cpp          Unified firmware
  platformio.ini      Build config and libraries
  secrets.ini         WiFi credentials — NOT committed to git
```

## Setup

### 1. Prerequisites

- [VS Code](https://code.visualstudio.com/) or [VSCodium](https://vscodium.com/)
- [PlatformIO IDE extension](https://platformio.org/install/ide?install=vscode)

### 2. Clone and open

```bash
git clone https://github.com/your-username/NSD-Dalek.git
```

Open the `dalek-esp32` folder in VS Code/VSCodium. PlatformIO will activate automatically.

### 3. WiFi credentials

Create a `secrets.ini` file in the project root (next to `platformio.ini`):

```ini
[env:esp32dev]
build_flags =
    ${env.build_flags}
    -DWIFI_SSID=\"YourNetworkName\"
    -DWIFI_PASSWORD=\"YourPassword\"
```

This file is excluded from git via `.gitignore` — never commit it.

### 4. Upload firmware

Connect the ESP32 via micro-USB, then in the PlatformIO toolbar:

1. Click **Build** (checkmark icon) — compiles the firmware
2. Click **Upload** (arrow icon) — flashes the firmware to the ESP32

### 5. Upload web interface

This only needs to be done once, and again whenever `data/index.html` changes:

- In the PlatformIO sidebar: expand `esp32dev → Platform → Upload Filesystem Image`
- Or via the command palette (`Ctrl+Shift+P`): `PlatformIO: Upload Filesystem Image`

### 6. Find the IP address

Open the Serial Monitor (plug icon in the toolbar) and press the **EN** button on the ESP32. The boot log prints the IP address within a few seconds:

```
WiFi status   : CONNECTED
IP address    : 192.168.x.x
```

Open that address in a browser on the same network to access the control interface.

## Normal workflow

| Changed | Action needed |
|---|---|
| `main.cpp` or `config.h` | Build + Upload firmware |
| `data/index.html` | Upload Filesystem Image |
| Both | Build + Upload firmware, then Upload Filesystem Image |

## Debugging

Debug output is controlled by `#define DEBUG` in `include/config.h`.
Comment it out for normal operation — this disables all serial output with zero runtime overhead.

## Libraries

Installed automatically by PlatformIO:

- [FastLED](https://github.com/FastLED/FastLED)
- [DFRobotDFPlayerMini](https://github.com/DFRobot/DFRobotDFPlayerMini)
- [FastAccelStepper](https://github.com/gin66/FastAccelStepper)
# C620 PlatformIO control

ESP32 firmware and macOS controller tools for a DJI C620/M3508 platform. The repository contains two separate firmware images; flash only the image for the hardware setup in use.

## Firmware projects

| Project | Purpose | Build |
|---|---|---|
| Repository root (`src/main.cpp`) | Chassis control for motor IDs 1–3; ID4 command bytes remain zero | `pio run` |
| `arm_control/` | Elbow control for motor ID4; commands for IDs 1–3 remain zero | `pio run -d arm_control` |

The two images use shared headers in `src/`. They are alternative programs for the ESP32, not processes to run at the same time.

To upload, first confirm the connected board and serial port, then use the matching build directory:

```sh
pio run -t upload
pio run -d arm_control -t upload
```

PlatformIO build and upload verify the firmware operation only. They do not verify CAN wiring, motor response, braking distance, or mechanical safety.

## Local Wi-Fi configuration

The chassis firmware starts a local access point. Copy `src/secrets.h.example` to `src/secrets.h` and set a private SSID and password before building. `src/secrets.h` is ignored by Git and must not be committed.

## macOS controller

The controller tools read a USB HID gamepad and provide the local web interfaces. Start the chassis controller with:

```sh
python3 tools/controller_input.py --drive
```

The elbow interface is served by `tools/arm_controller.py` and `tools/arm_controller.html`; it uses the serial port connected to the ESP32. See the status and setup checks in the page before enabling control.

## Safety and diagnostics

- The chassis image and elbow image have different motor-ID assignments. Verify which image is loaded before enabling motor output.
- Position and speed limits are software protections, not mechanical stops. A support, travel stop, and accessible power cutoff are still needed during hardware work.
- Logs, generated PlatformIO files, Python caches, and ROS 2 build artifacts are excluded from this source repository. Add a short, reviewed diagnostic excerpt only when it helps explain a specific issue.

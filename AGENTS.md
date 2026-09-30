# Laundry Room Controller — Agent Notes

## Build / Verify

Configure once:

```bash
. /home/honi/esp/esp-idf/export.sh
idf.py set-target esp32
```

Build:

```bash
. /home/honi/esp/esp-idf/export.sh
idf.py build
```

Flash:

```bash
. /home/honi/esp/esp-idf/export.sh
idf.py -p /dev/ttyUSB0 flash
```

Monitor (real terminal required):

```bash
. /home/honi/esp/esp-idf/export.sh
idf.py -p /dev/ttyUSB0 monitor
```

The board on this desk is connected through a CH340 adapter on `/dev/ttyUSB0`.
The relay board does not have a USB-serial bridge on its own USB connector; the
USB-to-TTL adapter must be wired to the ESP32 TX/RX pins for programming and logs.
If `idf.py flash` cannot enter the bootloader, press and hold the `BOOT` (GPIO0)
button, press `EN` (reset) briefly, then release `BOOT`, and retry the flash.

## Architecture

- **Core 0** (`wifi_task`, `tuya_worker_task`, `http_server`, `bt_spp`, `cli_serial`):
  Wi-Fi, Bluetooth, HTTP, Tuya, and command interfaces.
- **Core 1** (`sensor_task`, `ld2410_task`, `state_machine_task`): sensors,
  radar, and automation state machine.
- **NVS** stores all settings; `app_config_load_defaults` + `app_config_save`
  can be used for factory reset.

## Default board (green v1.0)

- Relay / fan #1: GPIO16, active-high
- Status LED: GPIO23, active-low
- I2C SDA/SCL: GPIO21/GPIO22, 100 kHz. Devices: VEML7700 0x10, AHT20 0x38,
  BMP280 0x77. **`trans_queue_depth` in `i2c_bus.c` must stay 0** — a non-zero
  value puts the ESP-IDF driver in asynchronous mode, whose ISR drops the
  completion event for `i2c_master_probe()` (no device handle) so every probe
  reports `I2C software timeout` and no sensor is ever detected. CLI diagnostics:
  `i2cscan` (driver), `i2cbb` (bit-bang, bypasses the peripheral),
  `pinpulse N`/`pindefault` (10 kHz on any GPIO for scope checks).
- LD2410S UART: GPIO19 (TX) / GPIO18 (RX), 115200 baud. Minimal report frames
  (6E | state | dist_lo | dist_hi | 62); config commands must be bracketed by
  enable-config (0x00FF) and end-config (0x00FE). `radarcfg`/`radarinfo`/`radarx`
  CLI commands exist for tuning and wire-level debugging.
- IR TX/RX: GPIO25/GPIO26. TX uses a salvaged remote front-end
  (IR LED + 1.5R series + DV4 NPN low-side, 82R base resistor) powered
  from 3.3V; keep a cap on its rail (~1A burst pulses). Fan protocol
  (verified working, fan responds to all commands): 38 kHz carrier,
  duty-cycle cells S=1240/440us, L=400/1240us, 12 cells per frame — the
  12th bit's mark doubles as the end-of-frame trailer the decoder
  requires, and its gap runs into the inter-frame space. Frames repeat
  x5 with ~8.4ms between bursts (varies 7.6-8.4ms with the last bit,
  same as the RC). Codes: ON=SSLSSLLLLLSL, OFF=SSLSSLLLLLLS,
  TIMER=SSLSSLLLSLLL, MODE=SSLSSLLLLSLL, SWING=SSLSSLLSLLLL. CLI:
  `irfan on|off|timer|mode|swing`. Two RMT quirks to preserve: the
  carrier free-runs while enabled, so it is applied only around each
  transmit (see ir_carrier_on/off); and the stream's tail items get
  clipped, so real data never sits last — three sacrificial low items
  follow the data (see ir_ctrl_send_pattern in ir_ctrl.c). Debug:
  `ircap` (print remote edges), `irloop` (TX->RX self-test),
  `irsend`/`irpwm`, `ircarrier` (live carrier freq).
- RF placeholder: GPIO27
- Button: GPIO0 (input only)

Other pin variants can be selected via `idf.py menuconfig`.

## Important runtime notes

- Bluetooth SPP device name is `LaundryCtrl`.
- On boot the status LED blinks fast (0.2 s) while starting, then slow (1 s)
  once Bluetooth/Wi-Fi is ready.
- The HTTP dashboard is on the IP given by Wi-Fi (port 80) and exposes
  status, control, and config endpoints.
- Manual relay/IR-fan/light commands set a 30-second override during which
  the state machine will not change that output.

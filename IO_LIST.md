# IO List — Laundry Room Controller

Board: ESP32-Relay-X1 — **Green v1.0** (`CONFIG_BOARD_GREEN_V10=y`, active in `sdkconfig`)
Source of truth: `main/Kconfig.projbuild` (all pins selectable via `idf.py menuconfig`)

## Active pinout (Green v1.0)

| GPIO | Function              | Dir    | Active level | Peripheral / notes                              |
|------|-----------------------|--------|--------------|-------------------------------------------------|
| 0    | User / BOOT button    | Input  | low          | Input-only pin; also used to enter bootloader   |
| 16   | Relay / fan #1        | Output | high         | On-board relay                                  |
| 19   | LD2410S TX (ESP → RX) | Output | —            | UART2 TX, 115200 baud (config commands, optional)|
| 18   | LD2410S RX (ESP ← TX) | Input  | —            | UART2 RX, minimal report frames 0x6E..0x62      |
| 21   | I2C SDA               | I/O    | —            | VEML7700, AHT20, BMP280 — 100 kHz               |
| 22   | I2C SCL               | Output | —            | I2C clock                                       |
| 23   | Status LED            | Output | low          | Fast blink (0.2 s) booting, slow (1 s) ready    |
| 25   | IR TX                 | Output | —            | RMT channel, IR LED                             |
| 26   | IR RX                 | Input  | —            | RMT channel, IR receiver                        |
| 27   | RF TX (SYN115)        | Output | —            | Placeholder for future RMT/ASK RF output        |

## Programming / console

| GPIO | Function   | Notes                                                        |
|------|------------|--------------------------------------------------------------|
| 1    | UART0 TX   | Console / flashing via external CH340 on `/dev/ttyUSB0`      |
| 3    | UART0 RX   | Board has no USB-serial bridge — wire adapter to TX/RX pins  |

## Blue v1.1 variant differences

| GPIO | Function   | Dir    | Active level |
|------|------------|--------|--------------|
| 19   | Relay      | Output | high         |
| 2    | Status LED | Output | low          |

All other pins identical to Green v1.0. Select via
`idf.py menuconfig` → *Laundry Room Controller* → *Relay board variant*.

## Config keys (for reference)

`CONFIG_RELAY_GPIO`, `CONFIG_RELAY_ACTIVE_LOW`, `CONFIG_STATUS_LED_GPIO`,
`CONFIG_STATUS_LED_ACTIVE_LOW`, `CONFIG_BUTTON_GPIO`, `CONFIG_I2C_SDA_GPIO`,
`CONFIG_I2C_SCL_GPIO`, `CONFIG_I2C_FREQ_HZ`, `CONFIG_LD2410_UART_PORT`,
`CONFIG_LD2410_TX_GPIO`, `CONFIG_LD2410_RX_GPIO`, `CONFIG_LD2410_BAUD_RATE`,
`CONFIG_IR_TX_GPIO`, `CONFIG_IR_RX_GPIO`, `CONFIG_RF_TX_GPIO`

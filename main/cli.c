#include "cli.h"

#include "board.h"
#include "bt_spp.h"
#include "config_manager.h"
#include "global_state.h"
#include "i2c_bus.h"
#include "esp_system.h"
#include "ir_ctrl.h"
#include "ld2410_hal.h"
#include "outputs.h"
#include "sensors.h"
#include "wifi_manager.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#define TAG "cli"
#define CLI_BUF_SIZE 256

static void cli_print_serial(const char *data, size_t len, void *user) {
    (void)user;
    if (len) write(STDOUT_FILENO, data, len);
}

static void cli_println(const char *line, cli_output_t out, void *user) {
    if (!out) {
        out = cli_print_serial;
    }
    out(line, strlen(line), user);
    out("\r\n", 2, user);
}

static void print_status(cli_output_t out, void *user) {
    /* Small snapshot only - a full global_state_t is ~2.3KB, too big for a task stack */
    status_snapshot_t st;
    global_state_status_snapshot(&st);

    char buf[128];
    snprintf(buf, sizeof(buf), "auto=%s fan_auto=%s light_auto=%s",
             st.auto_mode ? "on" : "off",
             st.fan_auto ? "on" : "off",
             st.light_auto ? "on" : "off");
    cli_println(buf, out, user);

    snprintf(buf, sizeof(buf), "relay=%s ir_fan=%s light=%s",
             st.fan_relay ? "on" : "off",
             st.fan_ir ? "on" : "off",
             st.light ? "on" : "off");
    cli_println(buf, out, user);

    snprintf(buf, sizeof(buf), "sensors: aht20=%s bmp280=%s veml7700=%s",
             st.aht_ok ? "ok" : "not detected",
             st.bmp_ok ? "ok" : "not detected",
             st.veml_ok ? "ok" : "not detected");
    cli_println(buf, out, user);

    snprintf(buf, sizeof(buf), "values: lux=%.1f temp=%.1f hum=%.1f press=%.1f",
             st.lux, st.temperature, st.humidity, st.pressure);
    cli_println(buf, out, user);

    snprintf(buf, sizeof(buf), "radar=%s mode=%s rx=%u state=%u dist=%u energy=%u",
             st.ld2410_ok ? "ok" : "not detected",
             st.ld2410_std_mode ? "standard" : "minimal",
             (unsigned)st.ld2410_rx_bytes,
             (unsigned)st.ld2410_state,
             (unsigned)st.moving_distance_cm,
             (unsigned)st.moving_energy);
    cli_println(buf, out, user);

    snprintf(buf, sizeof(buf), "wifi=%s ssid=%s rssi=%d ip=%s tuya=%s",
             st.wifi_connected ? "up" : (st.wifi_ssid[0] ? "connecting" : "down"),
             st.wifi_ssid[0] ? st.wifi_ssid : "-",
             st.wifi_rssi, st.ip_addr[0] ? st.ip_addr : "-",
             st.tuya_connected ? "up" : "down");
    cli_println(buf, out, user);

    snprintf(buf, sizeof(buf), "net: host=laundry_room.local bt=%s(%s)",
             BT_DEVICE_NAME, bt_spp_is_connected() ? "connected" : "advertising");
    cli_println(buf, out, user);
}

static void print_sensors(cli_output_t out, void *user) {
    /* Force a fresh read now rather than showing the last 1s poll. */
    sensor_health_t h;
    sensors_poll(&h);

    status_snapshot_t st;
    global_state_status_snapshot(&st);

    char buf[96];
    if (h.aht_ok) {
        snprintf(buf, sizeof(buf), "AHT20    (0x38): temp=%.1f C  hum=%.1f %%",
                 st.temperature, st.humidity);
    } else {
        snprintf(buf, sizeof(buf), "AHT20    (0x38): not detected");
    }
    cli_println(buf, out, user);

    if (h.bmp_ok) {
        int32_t adc_p, adc_t;
        float bmp_t;
        sensors_bmp_debug(&adc_p, &adc_t, &bmp_t);
        snprintf(buf, sizeof(buf),
                 "BMP280   (0x76/77): press=%.1f hPa  temp=%.1f C  raw p=%ld t=%ld",
                 st.pressure, bmp_t, (long)adc_p, (long)adc_t);
    } else {
        snprintf(buf, sizeof(buf), "BMP280   (0x76/77): not detected");
    }
    cli_println(buf, out, user);

    if (h.veml_ok) {
        snprintf(buf, sizeof(buf), "VEML7700 (0x10): lux=%.1f", st.lux);
    } else {
        snprintf(buf, sizeof(buf), "VEML7700 (0x10): not detected");
    }
    cli_println(buf, out, user);
}

static void print_help(cli_output_t out, void *user) {
    cli_println("Commands:", out, user);
    cli_println("  status            - show current state", out, user);
    cli_println("  sensors           - read I2C sensors now", out, user);
    cli_println("  relay on|off      - manual relay (fan #1)", out, user);
    cli_println("  irfan on|off|timer|mode|swing - manual IR fan", out, user);
    cli_println("  light on|off      - manual light", out, user);
    cli_println("  auto on|off       - enable/disable automatic control", out, user);
    cli_println("  fanauto on|off    - enable/disable automatic fan", out, user);
    cli_println("  lightauto on|off  - enable/disable automatic light", out, user);
    cli_println("  irsend PATTERN [REPEAT] [SMARK SGAP LMAR LGAP]", out, user);
    cli_println("  irloop [PATTERN]  - TX+RX loopback test (aim LED at VS1838B)", out, user);
    cli_println("  irpwm PATTERN [REPEAT] - software carrier TX (no RMT mod)", out, user);
    cli_println("  ircap             - print marks/gaps from any remote (8s listen)", out, user);
    cli_println("                    - send duty-cycle IR code (S/L), x5 @8.4ms", out, user);
    cli_println("  ircarrier HZ      - change IR carrier frequency live", out, user);
    cli_println("  learnir on|off    - learn IR code for fan on/off", out, user);
    cli_println("  threshold light|temp|hum VALUE", out, user);
    cli_println("  i2cscan           - scan I2C bus for devices", out, user);
    cli_println("  i2cwiggle [sda|scl] - pulse I2C pins for scope check (then reboot)", out, user);
    cli_println("  i2crecover        - clock SCL to unstick a slave holding SDA low", out, user);
    cli_println("  i2cpins SDA SCL   - move I2C bus to other GPIOs (dead-pin test)", out, user);
    cli_println("  i2cbb             - bit-bang I2C scan (bypasses I2C peripheral)", out, user);
    cli_println("  pintest [OUT IN]  - verify pin mapping via bridges", out, user);
    cli_println("  pinpulse GPIO     - 10kHz square wave on a pin (scope check)", out, user);
    cli_println("  pindefault        - stop pinpulse, restore pin to design config", out, user);
    cli_println("  radarcfg [SEC]    - set radar unmanned delay 10-120s (def 10)", out, user);
    cli_println("  radarparams       - read sensor params (unmanned delay, gates, rates)", out, user);
    cli_println("  radarread         - radar results: state, dist, per-gate energy", out, user);
    cli_println("  radarinfo         - read radar firmware version + serial", out, user);
    cli_println("  radarx            - raw hex exchange: enable+wrSN+rdSN+end", out, user);
    cli_println("  radarmode minimal|standard - switch radar report format", out, user);
    cli_println("  radardump [N]     - hex-dump next N radar frames (def 2)", out, user);
    cli_println("  radarraw CMD HEX  - raw cmd in cfg session, e.g. radarraw 007a 000000010000", out, user);
    cli_println("  tuya IP DEV KEY [DP] [DP2] - set Tuya switch settings", out, user);
    cli_println("  wifi SSID PASS    - set Wi-Fi credentials", out, user);
    cli_println("  save              - save config to NVS", out, user);
    cli_println("  factory           - reset to factory defaults and reboot", out, user);
    cli_println("  ?                 - this help", out, user);
}

static void pintest_pair(int o, int n, cli_output_t out, void *user) {
    gpio_set_direction((gpio_num_t)o, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)n, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)n, GPIO_PULLUP_ONLY);

    gpio_set_level((gpio_num_t)o, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    int lo = gpio_get_level((gpio_num_t)n);
    gpio_set_level((gpio_num_t)o, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    int hi = gpio_get_level((gpio_num_t)n);

    char buf[64];
    snprintf(buf, sizeof(buf), "  GPIO%d -> GPIO%d: %s (lo=%d hi=%d)",
             o, n, (lo == 0 && hi == 1) ? "OK" : "FAIL", lo, hi);
    cli_println(buf, out, user);
}

static void cmd_pintest(char **tok, int argc, cli_output_t out, void *user) {
    /* Each pair: drive OUT pin, read IN pin through the user's bridge.
       Reconfigures pins away from I2C/UART - reboot after this test. */
    cli_println("pin mapping test - reboot afterwards:", out, user);
    if (argc > 2) {
        pintest_pair(atoi(tok[1]), atoi(tok[2]), out, user);
        return;
    }
    /* GPIO12 avoided - it is a boot-strap pin (flash voltage select);
       pulling it high at reset hangs the boot. */
    static const struct { int out, in; } pairs[] = {
        {LD2410_TX_GPIO, 33}, {LD2410_RX_GPIO, 27}, {21, 25}, {22, 32},
    };
    for (size_t i = 0; i < sizeof(pairs) / sizeof(pairs[0]); i++) {
        pintest_pair(pairs[i].out, pairs[i].in, out, user);
    }
}

static void cmd_i2cscan(cli_output_t out, void *user) {
    /* Line levels: both should idle HIGH via pull-ups. A stuck-LOW line
       means a short, bad solder joint, or a dead sensor pulling it down. */
    int sda = (i2c_sda_gpio >= 0) ? i2c_sda_gpio : I2C_SDA_GPIO;
    int scl = (i2c_scl_gpio >= 0) ? i2c_scl_gpio : I2C_SCL_GPIO;
    char buf[80];
    snprintf(buf, sizeof(buf), "lines: SDA(GPIO%d)=%d SCL(GPIO%d)=%d",
             sda, gpio_get_level(sda), scl, gpio_get_level(scl));
    cli_println(buf, out, user);

    cli_println("scanning I2C bus 0x08-0x77...", out, user);
    int found = 0, nack = 0, tmo = 0, other = 0;
    esp_err_t first_err = ESP_OK;
    xSemaphoreTakeRecursive(i2c_mutex, portMAX_DELAY);
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        esp_err_t r = i2c_master_probe(i2c_bus, addr, 30);
        if (r == ESP_OK) {
            char buf[40];
            snprintf(buf, sizeof(buf), "  device at 0x%02X", addr);
            cli_println(buf, out, user);
            found++;
        } else if (r == ESP_ERR_TIMEOUT) {
            tmo++;
        } else if (r == ESP_FAIL || r == ESP_ERR_INVALID_RESPONSE) {
            nack++;
        } else {
            if (first_err == ESP_OK) first_err = r;
            other++;
        }
    }
    xSemaphoreGiveRecursive(i2c_mutex);
    snprintf(buf, sizeof(buf), "done: %d found, %d nack, %d tmo, %d err%s",
             found, nack, tmo, other,
             other ? esp_err_to_name(first_err) : "");
    cli_println(buf, out, user);
}

static void cmd_i2cwiggle(char **tok, int argc, cli_output_t out, void *user) {
    /* Pull the pins out of the I2C driver and toggle them as plain GPIO so
       the scope can confirm the physical pads unambiguously.
       i2cwiggle       - alternate: 8 pulses on SDA, then 8 on SCL, repeat
       i2cwiggle sda   - only SDA (GPIO21)
       i2cwiggle scl   - only SCL (GPIO22)
       REBOOT to restore the I2C bus afterwards. */
    int sda = (i2c_sda_gpio >= 0) ? i2c_sda_gpio : I2C_SDA_GPIO;
    int scl = (i2c_scl_gpio >= 0) ? i2c_scl_gpio : I2C_SCL_GPIO;
    int which = 0;   /* 0 = alternate, 1 = sda only, 2 = scl only */
    if (argc > 1) {
        if (strcasecmp(tok[1], "sda") == 0) which = 1;
        else if (strcasecmp(tok[1], "scl") == 0) which = 2;
    }
    char buf[96];
    snprintf(buf, sizeof(buf),
             "toggling SDA(GPIO%d)/SCL(GPIO%d) ~4Hz for 10s - then REBOOT",
             sda, scl);
    cli_println(buf, out, user);
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    gpio_set_direction(sda, GPIO_MODE_OUTPUT);
    gpio_set_direction(scl, GPIO_MODE_OUTPUT);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);

    for (int cycle = 0; cycle < 5; cycle++) {
        int pin = (which == 2) ? scl : sda;
        int other = (pin == sda) ? scl : sda;
        for (int i = 0; i < 8; i++) {          /* 8 pulses on this pin */
            gpio_set_level(pin, 0);
            vTaskDelay(pdMS_TO_TICKS(125));
            gpio_set_level(pin, 1);
            vTaskDelay(pdMS_TO_TICKS(125));
        }
        if (which == 0) {                       /* alternate between pins */
            gpio_set_level(pin, 1);
            pin = other;
            for (int i = 0; i < 8; i++) {
                gpio_set_level(pin, 0);
                vTaskDelay(pdMS_TO_TICKS(125));
                gpio_set_level(pin, 1);
                vTaskDelay(pdMS_TO_TICKS(125));
            }
        }
    }
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    cli_println("wiggle done - reboot to restore I2C", out, user);
}

static void cmd_i2crecover(cli_output_t out, void *user) {
    /* A stuck slave holding SDA low wedges the whole bus (all probes time
       out, SCL may free-run). i2c_master_bus_reset clocks SCL until the
       slave releases SDA, then issues a STOP. */
    char buf[48];
    esp_err_t r = i2c_master_bus_reset(i2c_bus);
    snprintf(buf, sizeof(buf), "bus reset: %s  SDA=%d SCL=%d",
             esp_err_to_name(r), gpio_get_level(I2C_SDA_GPIO),
             gpio_get_level(I2C_SCL_GPIO));
    cli_println(buf, out, user);
}

static void cmd_i2cpins(char **tok, int argc, cli_output_t out, void *user) {
    /* Move the I2C bus to different pins at runtime — used to test whether a
       GPIO is physically dead. Sensors re-probe on the new pins. */
    if (argc < 3) {
        cli_println("usage: i2cpins SDA SCL   e.g. i2cpins 21 22", out, user);
        return;
    }
    char buf[64];
    esp_err_t r = i2c_bus_relocate(atoi(tok[1]), atoi(tok[2]));
    snprintf(buf, sizeof(buf), "i2c bus on SDA=%d SCL=%d: %s",
             atoi(tok[1]), atoi(tok[2]), esp_err_to_name(r));
    cli_println(buf, out, user);
}

/* Bit-banged I2C scan — bypasses the I2C peripheral and driver entirely so
   the wires + sensor can be judged independently of the hardware block. */
#define BB_HALF_US 5   /* ~100 kHz */

static void bb_delay(void) { esp_rom_delay_us(BB_HALF_US); }
static void bb_scl(int scl, int v) { gpio_set_level(scl, v); bb_delay(); }
static void bb_sda(int sda, int v) { gpio_set_level(sda, v); bb_delay(); }

/* Returns true if a device ACKed the address. */
static bool bb_probe(int sda, int scl, uint8_t addr7) {
    uint8_t byte = (uint8_t)(addr7 << 1);   /* write bit */
    /* START: SDA falls while SCL high */
    bb_sda(sda, 1); bb_scl(scl, 1);
    bb_sda(sda, 0); bb_scl(scl, 0);
    for (int i = 7; i >= 0; i--) {
        bb_sda(sda, (byte >> i) & 1);
        bb_scl(scl, 1);
        bb_scl(scl, 0);
    }
    /* ACK bit: release SDA, clock, sample */
    bb_sda(sda, 1);
    bb_scl(scl, 1);
    bool ack = gpio_get_level(sda) == 0;
    bb_scl(scl, 0);
    /* STOP: SDA rises while SCL high */
    bb_sda(sda, 0); bb_scl(scl, 1); bb_sda(sda, 1);
    return ack;
}

static void cmd_i2cbb(cli_output_t out, void *user) {
    int sda = (i2c_sda_gpio >= 0) ? i2c_sda_gpio : I2C_SDA_GPIO;
    int scl = (i2c_scl_gpio >= 0) ? i2c_scl_gpio : I2C_SCL_GPIO;
    char buf[80];
    snprintf(buf, sizeof(buf), "bit-bang scan on SDA=%d SCL=%d (no I2C peripheral)...",
             sda, scl);
    cli_println(buf, out, user);

    /* Hold the bus mutex so the sensor task can't grab the pins back. */
    xSemaphoreTakeRecursive(i2c_mutex, portMAX_DELAY);
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(scl, GPIO_PULLUP_ONLY);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(100);

    snprintf(buf, sizeof(buf), "idle: SDA=%d SCL=%d (both must be 1)",
             gpio_get_level(sda), gpio_get_level(scl));
    cli_println(buf, out, user);

    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (bb_probe(sda, scl, a)) {
            snprintf(buf, sizeof(buf), "  ACK at 0x%02X", a);
            cli_println(buf, out, user);
            found++;
        }
        esp_rom_delay_us(50);
    }
    snprintf(buf, sizeof(buf), "bit-bang done: %d device(s)", found);
    cli_println(buf, out, user);

    /* Hand pins back to the I2C peripheral. */
    xSemaphoreGiveRecursive(i2c_mutex);
    i2c_bus_relocate(sda, scl);
}

/* pinpulse: drive any GPIO as output with pull-up and toggle it at 10 kHz
   (LEDC hardware PWM, 50% duty) until pindefault is called. */
#define PINPULSE_HZ      10000
#define PINPULSE_CHANNEL LEDC_CHANNEL_7
#define PINPULSE_TIMER   LEDC_TIMER_3

static int s_pulse_pin = -1;

static void cmd_pinpulse(char **tok, int argc, cli_output_t out, void *user) {
    if (argc < 2) {
        cli_println("usage: pinpulse GPIO   (10kHz square wave until pindefault)", out, user);
        return;
    }
    int pin = atoi(tok[1]);
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        cli_println("not a valid output GPIO", out, user);
        return;
    }
    if (s_pulse_pin >= 0) {
        cli_println("already pulsing another pin - run pindefault first", out, user);
        return;
    }

    sensors_set_paused(true);   /* stop the I2C driver reclaiming its pins */
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);

    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = PINPULSE_TIMER,
        .freq_hz = PINPULSE_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_channel_config_t ccfg = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PINPULSE_CHANNEL,
        .timer_sel = PINPULSE_TIMER,
        .duty = 512,   /* 50% of 10-bit */
        .hpoint = 0,
    };
    esp_err_t r = ledc_timer_config(&tcfg);
    if (r == ESP_OK) r = ledc_channel_config(&ccfg);

    char buf[80];
    if (r == ESP_OK) {
        s_pulse_pin = pin;
        snprintf(buf, sizeof(buf), "GPIO%d pulsing at %d Hz - 'pindefault' to stop",
                 pin, PINPULSE_HZ);
    } else {
        snprintf(buf, sizeof(buf), "pinpulse failed: %s", esp_err_to_name(r));
    }
    cli_println(buf, out, user);
}

static void cmd_pindefault(cli_output_t out, void *user) {
    if (s_pulse_pin < 0) {
        cli_println("no pin is pulsing", out, user);
        return;
    }
    int pin = s_pulse_pin;
    s_pulse_pin = -1;
    ledc_stop(LEDC_LOW_SPEED_MODE, PINPULSE_CHANNEL, 1);
    gpio_reset_pin(pin);
    sensors_set_paused(false);

    /* Hand the pin back to whichever peripheral owns it in the design. */
    const char *owner;
    int sda = (i2c_sda_gpio >= 0) ? i2c_sda_gpio : I2C_SDA_GPIO;
    int scl = (i2c_scl_gpio >= 0) ? i2c_scl_gpio : I2C_SCL_GPIO;
    if (pin == sda || pin == scl) {
        i2c_bus_relocate(sda, scl);
        owner = "I2C bus";
    } else if (pin == RELAY_GPIO) {
        outputs_init();
        owner = "relay (off)";
    } else if (pin == STATUS_LED_GPIO) {
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        owner = "status LED";
    } else if (pin == LD2410_TX_GPIO || pin == LD2410_RX_GPIO) {
        uart_set_pin(LD2410_UART_PORT, LD2410_TX_GPIO, LD2410_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (pin == LD2410_RX_GPIO) gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        owner = "LD2410 UART";
    } else if (pin == IR_TX_GPIO || pin == IR_RX_GPIO) {
        owner = "IR RMT - reboot to fully restore";
    } else if (pin == BUTTON_GPIO) {
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
        owner = "button input";
    } else {
        owner = "unused (reset to input)";
    }

    char buf[80];
    snprintf(buf, sizeof(buf), "GPIO%d restored: %s", pin, owner);
    cli_println(buf, out, user);
}

static bool parse_on_off(const char *s) {
    return (strcasecmp(s, "on") == 0 || strcmp(s, "1") == 0);
}

static control_cmd_t make_cmd(control_cmd_type_t type, int32_t val) {
    control_cmd_t c = {.type = type, .value = val};
    return c;
}

static void enqueue_cmd(const control_cmd_t *cmd) {
    xQueueSend(cmd_queue, cmd, pdMS_TO_TICKS(50));
}

static void cmd_set_mode(global_state_t *st, bool *field, bool val) {
    *field = val;
    control_cmd_t cmd = make_cmd(CMD_SET_MODE, 0);
    cmd.value = (st->config.auto_mode ? 1 : 0) |
                (st->config.fan_auto ? 2 : 0) |
                (st->config.light_auto ? 4 : 0);
    enqueue_cmd(&cmd);
}

esp_err_t cli_process_line(const char *line, cli_output_t out, void *user) {
    if (!line) return ESP_ERR_INVALID_ARG;
    if (!out) out = cli_print_serial;

    /* Strip CR/LF */
    char buf[CLI_BUF_SIZE];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *p = buf;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    size_t l = strlen(p);
    while (l > 0 && (p[l-1] == '\r' || p[l-1] == '\n' || p[l-1] == ' ')) p[--l] = '\0';

    if (l == 0) return ESP_OK;

    /* Tokenize */
    char *tok[8];
    int argc = 0;
    char *save;
    char *token = strtok_r(p, " \t", &save);
    while (token && argc < 8) {
        tok[argc++] = token;
        token = strtok_r(NULL, " \t", &save);
    }
    if (argc == 0) return ESP_OK;

    global_state_t *st = global_state_get();

    if (strcasecmp(tok[0], "?") == 0 || strcasecmp(tok[0], "help") == 0) {
        print_help(out, user);
    } else if (strcasecmp(tok[0], "status") == 0) {
        print_status(out, user);
    } else if (strcasecmp(tok[0], "sensors") == 0) {
        print_sensors(out, user);
    } else if (strcasecmp(tok[0], "relay") == 0 && argc > 1) {
        control_cmd_t cmd = make_cmd(CMD_SET_RELAY, parse_on_off(tok[1]));
        enqueue_cmd(&cmd);
        cli_println("ok relay", out, user);
    } else if (strcasecmp(tok[0], "irfan") == 0 && argc > 1) {
        if (!strcasecmp(tok[1], "on") || !strcasecmp(tok[1], "off")) {
            control_cmd_t cmd = make_cmd(CMD_SET_IR_FAN, parse_on_off(tok[1]));
            enqueue_cmd(&cmd);
        } else {
            ir_ctrl_send_fan(tok[1]);
        }
        cli_println("ok irfan", out, user);
    } else if (strcasecmp(tok[0], "light") == 0 && argc > 1) {
        control_cmd_t cmd = make_cmd(CMD_SET_LIGHT, parse_on_off(tok[1]));
        enqueue_cmd(&cmd);
        cli_println("ok light", out, user);
    } else if (strcasecmp(tok[0], "auto") == 0 && argc > 1) {
        global_state_lock();
        bool val = parse_on_off(tok[1]);
        cmd_set_mode(st, &st->config.auto_mode, val);
        global_state_unlock();
        cli_println("ok auto", out, user);
    } else if (strcasecmp(tok[0], "fanauto") == 0 && argc > 1) {
        global_state_lock();
        bool val = parse_on_off(tok[1]);
        cmd_set_mode(st, &st->config.fan_auto, val);
        global_state_unlock();
        cli_println("ok fanauto", out, user);
    } else if (strcasecmp(tok[0], "lightauto") == 0 && argc > 1) {
        global_state_lock();
        bool val = parse_on_off(tok[1]);
        cmd_set_mode(st, &st->config.light_auto, val);
        global_state_unlock();
        cli_println("ok lightauto", out, user);
    } else if (strcasecmp(tok[0], "irsend") == 0 && argc > 1) {
        int repeats = (argc > 2) ? atoi(tok[2]) : 5;
        uint32_t t[4];
        const uint32_t *tp = NULL;
        if (argc >= 7) {
            for (int i = 0; i < 4; i++) t[i] = (uint32_t)atoi(tok[3 + i]);
            tp = t;
        }
        ir_ctrl_send_pattern(tok[1], tp, repeats);
        cli_println("ok irsend", out, user);
    } else if (strcasecmp(tok[0], "ircap") == 0) {
        ir_ctrl_capture();
        cli_println("ok ircap", out, user);
    } else if (strcasecmp(tok[0], "irloop") == 0) {
        ir_ctrl_loopback(argc > 1 ? tok[1] : "SSLSSLLLLLS");
        cli_println("ok irloop", out, user);
    } else if (strcasecmp(tok[0], "irpwm") == 0 && argc > 1) {
        int repeats = (argc > 2) ? atoi(tok[2]) : 5;
        ir_ctrl_send_pattern_pwm(tok[1], NULL, repeats);
        cli_println("ok irpwm", out, user);
    } else if (strcasecmp(tok[0], "ircarrier") == 0 && argc > 1) {
        uint32_t hz = (uint32_t)atoi(tok[1]);
        esp_err_t ret = ir_ctrl_set_carrier(hz);
        snprintf(buf, sizeof(buf), "ok ircarrier=%lu (%d)",
                 (unsigned long)hz, (int)ret);
        cli_println(buf, out, user);
    } else if (strcasecmp(tok[0], "learnir") == 0 && argc > 1) {
        bool on = parse_on_off(tok[1]);
        ir_ctrl_learn(on);
        cli_println(on ? "learning IR on..." : "learning IR off...", out, user);
    } else if (strcasecmp(tok[0], "tuya") == 0 && argc > 3) {
        global_state_lock();
        app_config_t *cfg = &global_state_get()->config;
        strncpy(cfg->tuya_ip, tok[1], sizeof(cfg->tuya_ip) - 1);
        cfg->tuya_ip[sizeof(cfg->tuya_ip) - 1] = '\0';
        strncpy(cfg->tuya_dev_id, tok[2], sizeof(cfg->tuya_dev_id) - 1);
        cfg->tuya_dev_id[sizeof(cfg->tuya_dev_id) - 1] = '\0';
        strncpy(cfg->tuya_local_key, tok[3], sizeof(cfg->tuya_local_key) - 1);
        cfg->tuya_local_key[sizeof(cfg->tuya_local_key) - 1] = '\0';
        if (argc > 4) cfg->tuya_dpid = (uint8_t)atoi(tok[4]);
        cfg->tuya_dpid2 = (argc > 5) ? (uint8_t)atoi(tok[5]) : 0;
        global_state_unlock();
        cli_println("ok tuya", out, user);
    } else if (strcasecmp(tok[0], "threshold") == 0 && argc > 2) {
        float val = strtof(tok[2], NULL);
        global_state_lock();
        if (strcasecmp(tok[1], "light") == 0) st->config.light_thr_lux = val;
        else if (strcasecmp(tok[1], "temp") == 0) st->config.temp_thr_c = val;
        else if (strcasecmp(tok[1], "hum") == 0) st->config.hum_thr_pct = val;
        global_state_unlock();
        cli_println("ok threshold", out, user);
    } else if (strcasecmp(tok[0], "wifi") == 0 && argc > 2) {
        global_state_lock();
        strncpy(st->config.wifi_ssid, tok[1], sizeof(st->config.wifi_ssid) - 1);
        st->config.wifi_ssid[sizeof(st->config.wifi_ssid) - 1] = '\0';
        strncpy(st->config.wifi_pass, tok[2], sizeof(st->config.wifi_pass) - 1);
        st->config.wifi_pass[sizeof(st->config.wifi_pass) - 1] = '\0';
        global_state_unlock();
        wifi_manager_reconnect();
        cli_println("ok wifi", out, user);
    } else if (strcasecmp(tok[0], "i2cscan") == 0) {
        cmd_i2cscan(out, user);
    } else if (strcasecmp(tok[0], "i2cwiggle") == 0) {
        cmd_i2cwiggle(tok, argc, out, user);
    } else if (strcasecmp(tok[0], "i2crecover") == 0) {
        cmd_i2crecover(out, user);
    } else if (strcasecmp(tok[0], "i2cpins") == 0) {
        cmd_i2cpins(tok, argc, out, user);
    } else if (strcasecmp(tok[0], "i2cbb") == 0) {
        cmd_i2cbb(out, user);
    } else if (strcasecmp(tok[0], "pinpulse") == 0) {
        cmd_pinpulse(tok, argc, out, user);
    } else if (strcasecmp(tok[0], "pindefault") == 0) {
        cmd_pindefault(out, user);
    } else if (strcasecmp(tok[0], "pintest") == 0) {
        cmd_pintest(tok, argc, out, user);
    } else if (strcasecmp(tok[0], "radarcfg") == 0) {
        uint32_t sec = (argc > 1) ? (uint32_t)atoi(tok[1]) : 10;
        esp_err_t ret = ld2410_set_unmanned_delay(sec);
        if (ret == ESP_OK) {
            cli_println("ok radarcfg - delay + fast response applied", out, user);
        } else if (ret == ESP_ERR_TIMEOUT) {
            cli_println("radarcfg failed: no ACK (check ESP TX -> sensor RX wire)", out, user);
        } else {
            cli_println("radarcfg failed", out, user);
        }
    } else if (strcasecmp(tok[0], "radarx") == 0) {
        cli_println("radar exchange:", out, user);
        ld2410_exchange_test();
        cli_println("done", out, user);
    } else if (strcasecmp(tok[0], "radarmode") == 0) {
        if (argc < 2) {
            cli_println("usage: radarmode minimal|standard", out, user);
        } else {
            bool standard = (strcasecmp(tok[1], "standard") == 0);
            esp_err_t ret = ld2410_set_output_mode(standard);
            cli_println(ret == ESP_OK ? "ok radarmode" : "radarmode failed: no ACK",
                        out, user);
        }
    } else if (strcasecmp(tok[0], "radardump") == 0) {
        int cnt = (argc > 1) ? atoi(tok[1]) : 2;
        if (cnt < 1) cnt = 1;
        if (cnt > 20) cnt = 20;
        ld2410_dump_request(cnt);
        cli_println("dumping next frames to log...", out, user);
    } else if (strcasecmp(tok[0], "radarraw") == 0) {
        if (argc < 2) {
            cli_println("usage: radarraw CMD [HEXPARAMS]  e.g. radarraw 007a 000000010000", out, user);
        } else {
            uint16_t cmd = (uint16_t)strtoul(tok[1], NULL, 16);
            uint8_t params[24];
            uint16_t plen = 0;
            if (argc > 2) {
                const char *h = tok[2];
                size_t hl = strlen(h);
                while (hl >= 2 && plen < sizeof(params)) {
                    char b[3] = {h[0], h[1], 0};
                    params[plen++] = (uint8_t)strtoul(b, NULL, 16);
                    h += 2; hl -= 2;
                }
            }
            cli_println("raw exchange:", out, user);
            ld2410_raw_cmd(cmd, params, plen);
            cli_println("done", out, user);
        }
    } else if (strcasecmp(tok[0], "radarparams") == 0) {
        uint32_t v[6];
        uint8_t frame[128];
        uint16_t flen = sizeof(frame);
        esp_err_t ret = ld2410_read_params(v, frame, &flen);
        if (ret == ESP_OK) {
            char hex[3 * sizeof(frame) + 1];
            char *p = hex;
            for (uint16_t i = 0; i < flen; i++)
                p += sprintf(p, "%02x ", frame[i]);
            cli_println("rx frame:", out, user);
            cli_println(hex, out, user);
            snprintf(buf, sizeof(buf),
                     "radar: far_gate=%lu near_gate=%lu unmanned_delay=%lus",
                     (unsigned long)v[0], (unsigned long)v[1],
                     (unsigned long)v[2]);
            cli_println(buf, out, user);
            snprintf(buf, sizeof(buf),
                     "  status_rate=%.1fHz dist_rate=%.1fHz resp=%s",
                     v[3] / 10.0f, v[4] / 10.0f,
                     v[5] == 10 ? "fast" : "normal");
            cli_println(buf, out, user);
        } else {
            cli_println("radarparams failed: no ACK", out, user);
        }
    } else if (strcasecmp(tok[0], "radarread") == 0) {
        uint32_t gates[16];
        global_state_lock();
        bool ok = st->ld2410_ok;
        bool std = st->ld2410_std_mode;
        uint8_t rstate = st->ld2410_state;
        int presence = st->presence;
        uint8_t dist = st->ld2410_moving_distance_cm;
        uint16_t rxb = st->ld2410_rx_bytes;
        memcpy(gates, st->ld2410_gate_energy, sizeof(gates));
        global_state_unlock();
        snprintf(buf, sizeof(buf),
                 "radar: %s mode=%s state=%u presence=%s",
                 ok ? "ok" : "no-data", std ? "standard" : "minimal",
                 rstate,
                 presence == PRESENCE_MOVING ? "moving" :
                 presence == PRESENCE_STATIC ? "static" : "none");
        cli_println(buf, out, user);
        snprintf(buf, sizeof(buf), "  dist=%ucm rx=%u", dist, rxb);
        cli_println(buf, out, user);
        if (std) {
            int peak_g = 0;
            for (int g = 1; g < 16; g++)
                if (gates[g] > gates[peak_g]) peak_g = g;
            snprintf(buf, sizeof(buf), "  max gate: g%d energy=%lu (~%.2f-%.2fm)",
                     peak_g, (unsigned long)gates[peak_g],
                     peak_g * 0.75f, (peak_g + 1) * 0.75f);
            cli_println(buf, out, user);
            cli_println("  gate energies (~0.75m per gate):", out, user);
            for (int g = 0; g < 16; g += 4) {
                snprintf(buf, sizeof(buf),
                         "  g%-2d %6lu  g%-2d %6lu  g%-2d %6lu  g%-2d %6lu",
                         g,   (unsigned long)gates[g],
                         g+1, (unsigned long)gates[g+1],
                         g+2, (unsigned long)gates[g+2],
                         g+3, (unsigned long)gates[g+3]);
                cli_println(buf, out, user);
            }
        } else {
            cli_println("  (gate energies need: radarmode standard)", out, user);
        }
    } else if (strcasecmp(tok[0], "radarinfo") == 0) {
        char ver[24] = {0}, sn[24] = {0};
        esp_err_t ret = ld2410_read_info(ver, sizeof(ver), sn, sizeof(sn));
        if (ret == ESP_OK) {
            snprintf(buf, sizeof(buf), "radar: fw=%s sn=%s", ver, sn);
            cli_println(buf, out, user);
        } else if (ret == ESP_ERR_TIMEOUT) {
            cli_println("radarinfo failed: no ACK", out, user);
        } else {
            cli_println("radarinfo failed", out, user);
        }
    } else if (strcasecmp(tok[0], "save") == 0) {
        global_state_lock();
        esp_err_t ret = app_config_save(&st->config);
        global_state_unlock();
        cli_println(ret == ESP_OK ? "saved" : "save failed", out, user);
    } else if (strcasecmp(tok[0], "factory") == 0) {
        cli_println("resetting to factory defaults...", out, user);
        vTaskDelay(pdMS_TO_TICKS(100));
        global_state_lock();
        app_config_load_defaults(&st->config);
        app_config_save(&st->config);
        global_state_unlock();
        esp_restart();
    } else {
        cli_println("unknown command. Type '?' for help.", out, user);
    }

    return ESP_OK;
}

static void cli_serial_task(void *arg) {
    (void)arg;
    char buf[CLI_BUF_SIZE];
    char last[CLI_BUF_SIZE] = {0};   /* previous command, recalled by Up arrow */
    size_t pos = 0;
    int esc = 0;                     /* ANSI escape state: 0 none, 1 got ESC, 2 got ESC [ */

    /* Make stdin non-blocking */
    fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);

    for (;;) {
        uint8_t c;
        int n = read(STDIN_FILENO, &c, 1);
        if (n == 1) {
            if (esc == 1) {
                esc = (c == '[') ? 2 : 0;
                continue;
            }
            if (esc == 2) {
                esc = 0;
                if (c == 'A' && last[0]) {       /* Up arrow: recall last command */
                    /* erase current line contents on the terminal */
                    for (size_t i = 0; i < pos; i++) write(STDOUT_FILENO, "\b \b", 3);
                    strlcpy(buf, last, sizeof(buf));
                    pos = strlen(buf);
                    write(STDOUT_FILENO, buf, pos);
                }
                continue;                         /* swallow other cursor keys */
            }
            if (c == 27) {
                esc = 1;
            } else if (c == '\r' || c == '\n') {
                write(STDOUT_FILENO, "\r\n", 2);
                if (pos > 0) {
                    buf[pos] = '\0';
                    strlcpy(last, buf, sizeof(last));
                    cli_process_line(buf, NULL, NULL);
                    pos = 0;
                }
            } else if (pos < sizeof(buf) - 1 && c >= 32) {
                buf[pos++] = c;
                /* local echo */
                write(STDOUT_FILENO, &c, 1);
            } else if (c == '\b' || c == 127) {
                if (pos > 0) {
                    pos--;
                    write(STDOUT_FILENO, "\b \b", 3);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t cli_init(void) {
    ESP_LOGI(TAG, "cli init");
    xTaskCreatePinnedToCore(cli_serial_task, "cli_serial", 4096, NULL, 3, NULL, 0);
    return ESP_OK;
}

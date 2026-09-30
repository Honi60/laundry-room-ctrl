#pragma once

#include "config_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>
#include <stdint.h>

/* Command types from CLI/HTTP/BT */
typedef enum {
    CMD_SET_RELAY,
    CMD_SET_IR_FAN,
    CMD_SET_LIGHT,
    CMD_SET_MODE,
    CMD_SET_THRESHOLD,
    CMD_LEARN_IR_ON,
    CMD_LEARN_IR_OFF,
    CMD_FACTORY_RESET,
} control_cmd_type_t;

typedef struct {
    control_cmd_type_t type;
    int32_t            value;   /* on/off or numeric value */
    char               str[64]; /* e.g. Wi-Fi SSID, Tuya IP etc. */
} control_cmd_t;

/* Sensor snapshot */
typedef struct {
    bool valid;
    bool aht_ok;
    bool bmp_ok;
    bool veml_ok;
    float lux;
    float temperature;
    float humidity;
    float pressure;
} sensor_values_t;

typedef enum {
    PRESENCE_NONE,
    PRESENCE_MOVING,
    PRESENCE_STATIC,
} presence_state_t;

/* Shared runtime state. Access must be protected by global_state_lock(). */
typedef struct {
    app_config_t    config;

    /* Inputs */
    sensor_values_t sensors;
    presence_state_t presence;
    bool            ld2410_ok;
    bool            ld2410_std_mode;          /* true = standard frames, false = minimal */
    uint16_t        ld2410_rx_bytes;
    uint8_t         ld2410_state;             /* raw target state: 0/1 none, 2/3 target */
    uint8_t         ld2410_moving_distance_cm;
    uint8_t         ld2410_static_distance_cm;
    uint8_t         ld2410_moving_energy;
    uint8_t         ld2410_static_energy;
    uint32_t        ld2410_gate_energy[16];   /* standard mode: per-gate energy */
    bool            wifi_connected;
    bool            tuya_connected;
    int8_t          wifi_rssi;
    char            ip_addr[16];

    /* Outputs */
    bool fan_relay;
    bool fan_ir;
    bool light;

    /* LED/connection state */
    bool boot_done;
} global_state_t;

/* Small snapshot for status reporting - safe to keep on a task stack
   (global_state_t itself is ~2.3KB because of the embedded IR arrays). */
typedef struct {
    bool    auto_mode;
    bool    fan_auto;
    bool    light_auto;
    bool    fan_relay;
    bool    fan_ir;
    bool    light;
    bool    wifi_connected;
    bool    tuya_connected;
    int8_t  wifi_rssi;
    char    ip_addr[16];
    char    wifi_ssid[32];
    bool    sensors_valid;
    bool    aht_ok;
    bool    bmp_ok;
    bool    veml_ok;
    float   lux;
    float   temperature;
    float   humidity;
    float   pressure;
    int     presence;
    bool    ld2410_ok;
    bool    ld2410_std_mode;
    uint16_t ld2410_rx_bytes;
    uint8_t ld2410_state;
    uint8_t moving_distance_cm;
    uint8_t moving_energy;
    uint8_t static_distance_cm;
    uint8_t static_energy;
} status_snapshot_t;

extern QueueHandle_t cmd_queue;

/* Returns the global state pointer. */
global_state_t *global_state_get(void);

/* Mutex helpers. */
void global_state_lock(void);
void global_state_unlock(void);

/* Get a copy of the whole state for read-only consumers. */
void global_state_copy(global_state_t *out);

/* Fill a small status snapshot under the state lock. */
void global_state_status_snapshot(status_snapshot_t *out);

esp_err_t global_state_init(void);

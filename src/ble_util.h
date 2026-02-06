#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>

#define NODE_NAME_LENGTH 8
#define BT_UUID_LYS_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x8f3a6c21, 0x4b72, 0x4fd1, 0x9e38, 0x3c2b7d9a51f4)

#define BT_UUID_LYS_CHAR_VAL \
    BT_UUID_128_ENCODE(0x8f3a6c22, 0x4b72, 0x4fd1, 0x9e38, 0x3c2b7d9a51f4)

#define BT_UUID_TIME_SYNC \
    BT_UUID_128_ENCODE(0x12345679,0x1234,0x1234,0x1234,0x1234567890ab)

extern struct bt_uuid_128 bt_uuid_lys_service;
extern struct bt_uuid_128 bt_uuid_lys_char;
extern struct bt_uuid_128 bt_uuid_lys_time;

struct __attribute__((packed)) sensor_payload_v1 {

    uint8_t ver;
    uint32_t uptime_ms;   // k_uptime_get()
    uint32_t epoch_s;     
    uint16_t epoch_ms;   
    char     node_name[NODE_NAME_LENGTH];

    // MLX90614
    int16_t mlx_obj_c;
    int16_t mlx_amb_c;

    // SEN0546 
    int16_t sen_temp_c;
    int16_t sen_rh;

    // Soil 
    int16_t soil_temp_c;

    // Wind 
    int16_t wind_mph;

    // PAR 
    int32_t par_ppfd;

    // SP-610 pyranometer 
    int16_t shortwave_w_m2;

    // SL-610/160 pyrgeometer 
    int16_t pyr_temp_k;
    int16_t longwave_w_m2;

    // hx711
    int32_t weight_integer;
    int32_t weight_fractional;

};


struct time_values {
    int64_t epoch0_ms;    // epoch time at sync (ms)
    int64_t uptime0_ms;   // k_uptime_get() at sync (ms)
};

bool tx_subscribed(void);
int lys_ble_notify(struct bt_conn *g_conn, const struct sensor_payload_v1 *p);
bool time_now_epoch(uint32_t *out_epoch_s, uint16_t *out_epoch_ms);
void time_sync_from_pi(uint32_t epoch_s, uint16_t epoch_ms);
void pump_init(void);
void pump_schedule_init(void);
uint16_t get_node_period(void);
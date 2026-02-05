#pragma once

#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
/* 128-bit UUIDs for the custom Lysimeter service and characteristic */
#define BT_UUID_LYS_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x8f3a6c21, 0x4b72, 0x4fd1, 0x9e38, 0x3c2b7d9a51f4)

#define BT_UUID_LYS_CHAR_VAL \
    BT_UUID_128_ENCODE(0x8f3a6c22, 0x4b72, 0x4fd1, 0x9e38, 0x3c2b7d9a51f4)
    
/* Forward declarations of UUID objects */
extern struct bt_uuid_128 bt_uuid_lys_service;
extern struct bt_uuid_128 bt_uuid_lys_char;


struct __attribute__((packed)) sensor_payload_v1 {

    /* MLX90614 */
    int16_t mlx_obj_c;
    int16_t mlx_amb_c;

    /* SEN0546 */
    int16_t sen_temp_c;
    int16_t sen_rh;

    /* Soil */
    int16_t soil_temp_c;

     /* Wind */
    int16_t wind_mph;

    /* PAR */
    int32_t par_ppfd;

    /* SP-610 pyranometer */
    int16_t shortwave_w_m2;

    /* SL-610/160 pyrgeometer */
    int16_t pyr_temp_k;
    int16_t longwave_w_m2;

    // hx711
    int32_t weight_integer;
    int32_t weight_fractional;

};

bool tx_subscribed(void);
/* Update the GATT value and send notification (if connected) */
int lys_ble_notify(struct bt_conn *g_conn, const struct sensor_payload_v1 *p);

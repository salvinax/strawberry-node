#include "ble_util.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

// ble_uti.c - handles bluetooth + time logic + pump scheduling 
LOG_MODULE_REGISTER(ble_lys, LOG_LEVEL_INF);

// BLE notification flag
// true when the pi subscribes to notifications
static bool notify_enabled;

// Set to true after valid time is received from the pi
static bool time_synced = false;

// Next scheduled pump time, sent by the  pi as Unix epoch seconds
static uint32_t next_pump_epoch_s;

// Delayed work items used to turn the pump ON/OFF at scheduled times
static struct k_work_delayable pump_on_work;
static struct k_work_delayable pump_off_work;

// Pump ON duration and node sampling period - both sent by the Raspberry Pi.
static uint16_t pump_duration_s;
static uint16_t node_period_s;

// BLE UUIDs
struct bt_uuid_128 bt_uuid_lys_service = BT_UUID_INIT_128(BT_UUID_LYS_SERVICE_VAL);
struct bt_uuid_128 bt_uuid_lys_char    = BT_UUID_INIT_128(BT_UUID_LYS_CHAR_VAL);
struct bt_uuid_128 bt_uuid_lys_time   = BT_UUID_INIT_128(BT_UUID_TIME_SYNC); 

// Current sensor payload stored before notification
static struct sensor_payload_v1 g_cur_payload;

// Stores synchronized Pi time + nRF52 uptime
static struct time_values g_time;

// Called when the Raspberry Pi enables or disables BLE notifications.
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Lysimeter notifications enabled");
    } else {
        LOG_INF("Lysimeter notifications disabled");
    }
}

//Helper function used by other code to check if BLE notifications are enabled.
bool tx_subscribed(void)
{
    return notify_enabled;
}

// Save the Raspberry Pi's current Unix time.
// After this the nRF52 can estimate current epoch time using its uptime.
void time_sync_from_pi(uint32_t epoch_s, uint16_t epoch_ms)
{
    // store pi epoch time in milliseconds
    g_time.epoch0_ms  = (int64_t)epoch_s * 1000 + (int64_t)epoch_ms;
    
    // store local uptime 
    g_time.uptime0_ms = (int64_t)k_uptime_get();
    time_synced = true;
}

// Estimate current Unix time using last pi time sync plus nRF52 uptime.
bool time_now_epoch(uint32_t *out_epoch_s, uint16_t *out_epoch_ms)
{
    // only works after Pi time sync has happened
    if (!time_synced) {
        return false;
    }

    int64_t now_ms = g_time.epoch0_ms + ((int64_t)k_uptime_get() - g_time.uptime0_ms);
    if (now_ms < 0) return false;

    *out_epoch_s  = (uint32_t)(now_ms / 1000);
    *out_epoch_ms = (uint16_t)(now_ms % 1000);
    return true;
}

// schedule the pump to turn ON 
static void pump_set_next_epoch(uint32_t epoch_s)
{
    next_pump_epoch_s = epoch_s;

    uint32_t now_s; 
    uint16_t now_ms;

    if (!time_now_epoch(&now_s, &now_ms)) {
        LOG_WRN("No valid time yet; cannot arm pump");
        return;
    }

    int64_t delay_ms = ((int64_t)next_pump_epoch_s - (int64_t)now_s) * 1000 - now_ms;
    if (delay_ms < 0) delay_ms = 0;

    k_work_schedule(&pump_on_work, K_MSEC(delay_ms));
    LOG_INF("Pump armed for epoch=%u (in %lld ms)", epoch_s, delay_ms);
}


// Pi writes a 14-byte payload:
// bytes 0-3   = current epoch seconds
// bytes 4-5   = current epoch milliseconds
// bytes 6-9   = next pump start epoch seconds
// bytes 10-11 = pump duration in seconds
// bytes 12-13 = node sampling period in seconds

static ssize_t time_sync_write(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr,
                               const void *buf, uint16_t len,
                               uint16_t offset, uint8_t flags)
{

    ARG_UNUSED(conn);
    ARG_UNUSED(attr);
    ARG_UNUSED(flags);

    //payload must be 14 bytes
    if (len != 14) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    // get pi payload
    const uint8_t *b = buf;
    uint32_t epoch_s     = sys_get_le32(&b[0]);
    uint16_t epoch_ms    = sys_get_le16(&b[4]);
    uint32_t next_pump_s = sys_get_le32(&b[6]);
    uint16_t duration_s  = sys_get_le16(&b[10]);
    uint16_t period_s    = sys_get_le16(&b[12]);

    next_pump_epoch_s = next_pump_s;
    pump_duration_s   = duration_s;
    node_period_s     = period_s;

    LOG_INF("Time sync received");
    LOG_INF("epoch_s=%u, epoch_ms=%u, next_pump=%u, duration=%u, period=%u",
        epoch_s,
        epoch_ms,
        next_pump_epoch_s,
        pump_duration_s,
        node_period_s);

    // save time from pi
    time_sync_from_pi(epoch_s, epoch_ms);

    // schedule next pump run
    pump_set_next_epoch(next_pump_epoch_s);

    return len;
}

//  Helper Function - Return node sampling period - sent by the pi during time sync
uint16_t get_node_period(void){
    return node_period_s;
}

//  INTIALIZE PUMP CTRL PIN
static const struct gpio_dt_spec pump =GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pump_relay_gpios);


void pump_init(void)
{
    int err;

    if (!gpio_is_ready_dt(&pump)) {
        LOG_ERR("Pump GPIO is not ready");
        return -ENODEV;
    }

    // configure pump relay as output and start OFF
    err = gpio_pin_configure_dt(&pump, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Failed to configure pump GPIO: %d", err);
        return err;
    }

    LOG_INF("Pump GPIO initialized");

    return 0;
}

static void pump_on(void)  { 
    gpio_pin_set_dt(&pump, 1); 
    LOG_INF("PUMP ON");
}

static void pump_off(void) { 
    gpio_pin_set_dt(&pump, 0);
    LOG_INF("PUMP OFF"); 
}


static void pump_off_fn(struct k_work *work)
{
    pump_off();
}

// turns on pump and off and reschedules 
static void pump_on_fn(struct k_work *work)
{
    pump_on();
    // turn pump off after set duration
    k_work_schedule(&pump_off_work, K_SECONDS(pump_duration_s));
     
    // Schedule next pump in 24 hours
    next_pump_epoch_s += 86400; // sec in 24 hours 
    // next_pump_epoch_s += 60; // for debug every minute
    // add to payload; so user can set this from global hub

    uint32_t now_s; 
    uint16_t now_ms;

    if (time_now_epoch(&now_s, &now_ms)) {
        int64_t delay_ms = ((int64_t)next_pump_epoch_s - (int64_t)now_s) * 1000 - now_ms;
        if (delay_ms < 0) delay_ms = 0;
        k_work_schedule(&pump_on_work, K_MSEC(delay_ms));
        LOG_INF("Next pump scheduled for epoch=%u, in %lld ms",
                next_pump_epoch_s,
                delay_ms);
    } else {
        LOG_WRN("Time not synced; cannot reschedule next pump");
    }
}

// initialize delayed work objects for pump scheduling
void pump_schedule_init(void)
{
     k_work_init_delayable(&pump_on_work, pump_on_fn);
    k_work_init_delayable(&pump_off_work, pump_off_fn);
}

/*
 * GATT service definition.
 *
 * Includes:
 * 1. Sensor payload characteristic:
 *    - NOTIFY: Node can notify Pi with new payload
 *
 * 2. Time sync characteristic:
 *    - WRITE: Pi sends time and pump scheduling information
 */

BT_GATT_SERVICE_DEFINE(lys_svc,
    BT_GATT_PRIMARY_SERVICE(&bt_uuid_lys_service),


    // notify characteristic for sensor payload
    BT_GATT_CHARACTERISTIC(&bt_uuid_lys_char.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, &g_cur_payload),

    BT_GATT_CCC(ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CUD("Sensor Payload from Node", BT_GATT_PERM_READ),

    // characteristic for pi writing time to nordic for time sync
    BT_GATT_CHARACTERISTIC(&bt_uuid_lys_time.uuid,
        BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
        BT_GATT_PERM_WRITE,
        NULL,               
        time_sync_write,     
        NULL),

    BT_GATT_CUD("Time Sync", BT_GATT_PERM_READ),

  
);

// Send current sensor payload using BLE notification
int lys_ble_notify(struct bt_conn *g_conn, const struct sensor_payload_v1 *p)
{
    int err;

    if (!g_conn) {
        return -ENOTCONN;
    }

     // Do not send notification if Pi has not subscribed
    if (!notify_enabled) {
        return 0;
    }

    g_cur_payload = *p;

    err = bt_gatt_notify(g_conn, 
                        &lys_svc.attrs[2],
                        &g_cur_payload, 
                        sizeof(g_cur_payload));
                    

    if (err == -ENOTCONN || err == -EPIPE) {
        return 0;
    }

    if (err) {
        LOG_WRN("Notify failed (err %d)", err);
    }

    return err;
}


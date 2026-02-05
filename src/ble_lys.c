#include "ble_lys.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

LOG_MODULE_REGISTER(ble_lys, LOG_LEVEL_INF);
static bool notify_enabled;
/* UUID objects */
struct bt_uuid_128 bt_uuid_lys_service = BT_UUID_INIT_128(BT_UUID_LYS_SERVICE_VAL);
struct bt_uuid_128 bt_uuid_lys_char    = BT_UUID_INIT_128(BT_UUID_LYS_CHAR_VAL);


/* Current payload value stored in the GATT database */
static struct sensor_payload_v1 g_cur_payload;

// static ssize_t lys_read(struct bt_conn *conn,
//                         const struct bt_gatt_attr *attr,
//                         void *buf, uint16_t len, uint16_t offset)
// {
//     const struct lys_payload *p = attr->user_data;
//     return bt_gatt_attr_read(conn, attr, buf, len, offset, p, sizeof(*p));
// }

/* CCC (Client Characteristic Configuration) descriptor */
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

//tx_subscribed()
bool tx_subscribed(void)
{
    return notify_enabled;
}

/* GATT service definition */
BT_GATT_SERVICE_DEFINE(lys_svc,
    BT_GATT_PRIMARY_SERVICE(&bt_uuid_lys_service),
    BT_GATT_CHARACTERISTIC(&bt_uuid_lys_char.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           NULL, NULL, &g_cur_payload),
    BT_GATT_CCC(ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

/* Connection callbacks */

int lys_ble_notify(struct bt_conn *g_conn, const struct sensor_payload_v1 *p)
{
    int err;

    g_cur_payload = *p;

    err = bt_gatt_notify(g_conn, &lys_svc.attrs[2],
                         &g_cur_payload, sizeof(g_cur_payload));
    if (err == -ENOTCONN || err == -EPIPE) {
        return 0;
    }

    if (err) {
        LOG_WRN("Notify failed (err %d)", err);
    }

    return err;
}

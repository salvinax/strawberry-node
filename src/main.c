#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include "drv_mlx90614.h"
#include "drv_analog.h"
#include "ble_util.h"
#include "drv_sen0546.h"
#include "drv_ads1015.h"
#include "drv_load_point.h"
#include <math.h>


LOG_MODULE_REGISTER(main_local, LOG_LEVEL_INF);


static struct mlx90614_sample g_mlx;
static struct sen0546_sample g_sen;
static struct pyrgeometer_ADS_sample g_pyrgeo;
static struct pyranometer_ADS_sample g_pyrano;
static struct soil_sample    g_soil;
static struct wind_sample    g_wind;
static struct par_sample   g_par;
static struct load_sample g_load;

static const char NODE_NAME[NODE_NAME_LENGTH] = "nodeA";


static const struct device *const i2c0 = DEVICE_DT_GET(DT_NODELABEL(i2c0));

static int i2c_scan_log(const struct device *i2c)
{
    if (!device_is_ready(i2c)) {
        LOG_INF("I2C device not ready");
        return -ENODEV;
    }

    int found = 0;

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        /* probe: 0-byte write */
        int err = i2c_write(i2c, NULL, 0, addr);
        if (err == 0) {
            LOG_INF("I2C ACK at 0x%02X", addr);
            found++;
        }
        k_sleep(K_MSEC(2));
    }

    LOG_INF("I2C scan done, found %d device(s)", found);
    return found;
}

// BLE FUNCTIONS/HELPERS
#define S16_INVALID ((int16_t)INT16_MIN)

static inline int16_t s16_q(float x, float scale)
{
    if (!isfinite(x)){
        return S16_INVALID;
    }

    float v = x * scale;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)(v >= 0 ? (v + 0.5f) : (v - 0.5f));
}


static inline void fill_node_name(char out[NODE_NAME_LENGTH])
{
    memset(out, 0, NODE_NAME_LENGTH);
    memcpy(out, NODE_NAME, strnlen(NODE_NAME, NODE_NAME_LENGTH));
}


static void build_payload(struct sensor_payload_v1 *pl)
{
    memset(pl, 0, sizeof(*pl));

    // HEADER GEN INFOsazzzz
    uint32_t epoch_s = 0;
    uint16_t epoch_ms = 0;
    time_now_epoch(&epoch_s, &epoch_ms);

    pl->ver = 1; // payload version is 1 incase we change it down the line - for parsing
    pl->uptime_ms= (uint32_t)k_uptime_get();
    pl->epoch_s = epoch_s;
    pl->epoch_ms = epoch_ms;
    fill_node_name(pl->node_name);

    // data
    pl->mlx_obj_c = s16_q(g_mlx.obj_c, 100.0f);
    pl->mlx_amb_c = s16_q(g_mlx.amb_c, 100.0f);

    pl->sen_temp_c = s16_q(g_sen.temp_c, 100.0f);
    pl->sen_rh     = s16_q(g_sen.rh, 100.0f);

    //pl->soil_v      = g_soil.voltage;
    //pl->soil_r_ohm  = g_soil.resistance;
    pl->soil_temp_c = s16_q(g_soil.temp_c, 100.0f);

    //pl->wind_v      = g_wind.voltage;
    //pl->wind_temp_c = s16_q(g_wind.temp_in_c, 100.0f);
    pl->wind_mph    = s16_q(g_wind.mph, 100.0f);

    //pl->par_shunt_v   = g_par.voltage;
   // pl->par_current_mA= g_par.current_mA;
    pl->par_ppfd      = (int32_t)lroundf(g_par.ppfd * 100.0f);

    pl->shortwave_w_m2 = s16_q(g_pyrano.shortwave_radiation, 100.0f);

    //pl->thermopile_mv_diff = g_pyrgeo.thermopile_signal_mv_diff;
    pl->pyr_temp_k         = s16_q(g_pyrgeo.temperature_in_K, 100.0f);
    pl->longwave_w_m2      = s16_q(g_pyrgeo.longwave_radiation, 100.0f);

  
    pl->weight_integer    = g_load.integer;
    pl-> weight_fractional = g_load.fractional;
    
}

struct bt_conn *my_conn = NULL;
static struct k_work adv_work;

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONN |
     BT_LE_ADV_OPT_USE_IDENTITY),
    BT_GAP_ADV_FAST_INT_MIN_1,
    BT_GAP_ADV_FAST_INT_MAX_1,
    NULL);


/* STEP 11.2 - Create variable that holds callback for MTU negotiation */
static struct bt_gatt_exchange_params exchange_params;

/* STEP 13.4 - Forward declaration of exchange_func(): */
static void exchange_func(struct bt_conn *conn, uint8_t att_err, struct bt_gatt_exchange_params *params);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_LYS_SERVICE_VAL),
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};


static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

static void advertising_start(void)
{
    k_work_submit(&adv_work);
}

/* STEP 7.1 - Define the function to update the connection's PHY */
static void update_phy(struct bt_conn *conn)
{
    int err;
    const struct bt_conn_le_phy_param preferred_phy = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
    };
    err = bt_conn_le_phy_update(conn, &preferred_phy);
    if (err) {
        LOG_ERR("bt_conn_le_phy_update() returned %d", err);
    }
}

/* STEP 10 - Define the function to update the connection's data length */
static void update_data_length(struct bt_conn *conn)
{
    int err;
    struct bt_conn_le_data_len_param my_data_len = {
        .tx_max_len = BT_GAP_DATA_LEN_MAX,
        .tx_max_time = BT_GAP_DATA_TIME_MAX,
    };
    err = bt_conn_le_data_len_update(my_conn, &my_data_len);
    if (err) {
        LOG_ERR("data_len_update failed (err %d)", err);
    }
}

/* STEP 11.1 - Define the function to update the connection's MTU */
static void update_mtu(struct bt_conn *conn)
{
    int err;
    exchange_params.func = exchange_func;

    err = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (err) {
        LOG_ERR("bt_gatt_exchange_mtu failed (err %d)", err);
    }
}

K_THREAD_STACK_DEFINE(sensor_wq_stack, 4096);
static struct k_work_q sensor_wq;

static struct k_work_delayable sensor_work;
static struct k_work sensor_start_work;
static struct k_work sensor_stop_work;

static void sensor_start_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_INF("Sensor loop START");
    k_work_schedule_for_queue(&sensor_wq, &sensor_work, K_NO_WAIT);
}

static void sensor_stop_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_INF("Sensor loop STOP");
    k_work_cancel_delayable(&sensor_work);
}

// get coefficient from certificates
// taken from SL-610-SS-1437
#define K1 8.658f // w/m2 per mV
#define K2 1.210f // unitless

#define STEPHAN_CONSTANT 5.6704e-8f

static struct sensor_payload_v1 pl;


static void debug_sensors(void)
{
    LOG_INF("MLX90614: obj=%.2f C, amb=%.2f C",
            (double)g_mlx.obj_c,
        (double)g_mlx.amb_c);

    LOG_INF("SEN0546: T=%.2f C, RH=%.2f%%",
            (double)g_sen.temp_c,
            (double)g_sen.rh);

    LOG_INF("Soil: V=%.3f V, R=%.0f ohm, T=%.2f C",
        (double)g_soil.voltage,
        (double)g_soil.resistance,
        (double)g_soil.temp_c);

    LOG_INF("Wind: V=%.3f V (RV), Temp (C)=%.2f, MPH=%.2f",
        (double)g_wind.voltage,
        (double)g_wind.temp_in_c,
        (double)g_wind.mph);
    
    LOG_INF("PAR: V=%.3f V (shunt), I=%.2f mA, PPFD=%.2f µmol m^-2 s^-1",
            (double)g_par.voltage,  (double)g_par.current_mA,  (double)g_par.ppfd);

    LOG_INF("ADS1O15 CH0&1->SP-610: SW=%.6f W/m^2",
            (double)g_pyrano.shortwave_radiation);

    LOG_INF("SL-160 PYR: Temp in K=%.2f", (double)g_pyrgeo.temperature_in_K);

    LOG_INF("ADS1O15 CH2&3->SL-610 Thermopile Signal: %.6f mV",
            (double)g_pyrgeo.thermopile_signal_mv_diff);

    LOG_INF("SL-160 PYR: Longwave radiation W/m2=%.2f", (double)g_pyrgeo.longwave_radiation);

    LOG_INF("Weight: %d.%02d grams",g_load.integer, g_load.fractional);

}

/* Main periodic work: read sensors and send BLE packet */
static void sensor_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    int err;
    if (tx_subscribed()) {
        //Read MLX90614 (IR temp) - WORKS
        err = mlx90614_read(&g_mlx);
        if (err && err != -EAGAIN) {
            LOG_WRN("MLX90614 read err %d", err);
        }
       
        // Read SEN0546 TEMP/HUMIDITY -WORKS
        err = sen0546_read(&g_sen);
        if (err && err != -EAGAIN) {
            LOG_WRN("SEN0546 read err %d", err);
        }

        //Read soil thermistor (root temp) - WORKS
        err = soil_read(&g_soil);
        if (err && err != -EAGAIN) {
            LOG_WRN("Soil read err %d", err);
        }

        //Read wind sensor - TEMP A BIT OFF BUT MPH GOOD NEED 2X VOLTAGE DIVIDER
        err = wind_read(&g_wind);
        if (err && err != -EAGAIN) {
            LOG_WRN("Wind read err %d", err);
        }

        //  Read PAR (SQ-214) - SHUNT RESISTOR + POWER 7V
        err = par_read(&g_par);
        if (err && err != -EAGAIN) {
            LOG_WRN("PAR read err %d", err);
        }
        
         // ADS1015 MEASUREMETNS
        //GET SP-610 DATA (SHORTWAVE RADIATION) - works
        get_sp610_pyranometer(&g_pyrano);
        

        // sl-610 THERMISTOR READING WORKS - RATIOMETRIC MEASUREMENT WITH EXCITATION
        err = sl160_thermistor_read(&g_pyrgeo);
        if (err && err != -EAGAIN) {
            LOG_WRN("SL-610 read err %d", err);
        }

        // GET SL-610 THERMOPILE DATA NEEDED FOR EQUATION LATER ON - works
        get_sl610_thermopile(&g_pyrgeo);
       

        float calculate_longwave = K1 * g_pyrgeo.thermopile_signal_mv_diff + (K2 *  STEPHAN_CONSTANT * (float)pow(g_pyrgeo.temperature_in_K, 4));
        if isfinite(calculate_longwave)
        {
             g_pyrgeo.longwave_radiation = calculate_longwave;
        } else {
            g_pyrgeo.longwave_radiation = NAN; // OR MAYBE NAN OR 0.0f??
        }

        // Get data from load point
        measure(&g_load);

        debug_sensors();
       
        // if subscribed send with ble
        // map payload
        build_payload(&pl);

        // //send ble notification
        lys_ble_notify(my_conn, &pl);
    }

   //reschedule
    k_work_schedule(&sensor_work, K_SECONDS(get_node_period()));
}




static void sensor_wq_init(void)
{
    k_work_queue_start(&sensor_wq,
                       sensor_wq_stack,
                       K_THREAD_STACK_SIZEOF(sensor_wq_stack),
                       2,
                       NULL);

    k_work_init_delayable(&sensor_work, sensor_work_handler);
    k_work_init(&sensor_start_work, sensor_start_handler);
    k_work_init(&sensor_stop_work, sensor_stop_handler);
}



/* Callbacks */
void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection error %d", err);
		return;
	}
	LOG_INF("Connected");
	my_conn = bt_conn_ref(conn);
    k_sleep(K_MSEC(100));

	/* STEP 1.1 - Declare a structure to store the connection parameters */
	struct bt_conn_info info;
	err = bt_conn_get_info(conn, &info);
	if (err) {
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}
	/* STEP 1.2 - Add the connection parameters to your log */
	double connection_interval = BT_GAP_US_TO_CONN_INTERVAL(info.le.interval_us) *1.25; // in ms
	uint16_t supervision_timeout = info.le.timeout*10; // in ms
	LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, info.le.latency, supervision_timeout);
	/* STEP 7.2 - Update the PHY mode */
	update_phy(my_conn);
	/* STEP 13.5 - Update the data length and MTU */
	update_data_length(my_conn);
	update_mtu(my_conn);

    // start collecting data
    k_work_submit_to_queue(&sensor_wq, &sensor_start_work);
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    k_work_cancel_delayable(&sensor_work);
    LOG_INF("Disconnected. Reason %d", reason);
    bt_conn_unref(my_conn);
    my_conn = NULL;

    //stop collecting data
    k_work_submit_to_queue(&sensor_wq, &sensor_stop_work);
}

void on_recycled(void)
{
    advertising_start();
}

/* STEP 4.2 - Add the callback for connection parameter updates */
void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    double connection_interval = interval*1.25;         // in ms
    uint16_t supervision_timeout = timeout*10;          // in ms
    LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}
/* STEP 8.1 - Write a callback function to inform about updates in the PHY */
void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    // PHY Updated
    if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M) {
        LOG_INF("PHY updated. New PHY: 1M");
    }
    else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M) {
        LOG_INF("PHY updated. New PHY: 2M");
    }
    else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8) {
        LOG_INF("PHY updated. New PHY: Long Range");
    }
}

/* STEP 13.1 - Write a callback function to inform about updates in data length */
void on_le_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    uint16_t tx_len     = info->tx_max_len;
    uint16_t tx_time    = info->tx_max_time;
    uint16_t rx_len     = info->rx_max_len;
    uint16_t rx_time    = info->rx_max_time;
    LOG_INF("Data length updated. Length %d/%d bytes, time %d/%d us", tx_len, rx_len, tx_time, rx_time);
}

struct bt_conn_cb connection_callbacks = {
    .connected          = on_connected,
    .disconnected       = on_disconnected,
    .recycled           = on_recycled,
    /* STEP 4.1 - Add the callback for connection parameter updates */
    .le_param_updated   = on_le_param_updated,
    /* STEP 8.3 - Add the callback for PHY mode updates */
    .le_phy_updated     = on_le_phy_updated,
    /* STEP 13.2 - Add the callback for data length updates */
    .le_data_len_updated    = on_le_data_len_updated,
};

/* STEP 13.3 - Implement callback function for MTU exchange */
static void exchange_func(struct bt_conn *conn, uint8_t att_err,
              struct bt_gatt_exchange_params *params)
{
    LOG_INF("MTU exchange %s", att_err == 0 ? "successful" : "failed");
    if (!att_err) {
        uint16_t payload_mtu = bt_gatt_get_mtu(conn) - 3;   // 3 bytes used for Attribute headers.
        LOG_INF("New MTU: %d bytes", payload_mtu);
    }
}


int main(void)

{
    LOG_INF("Sensor node starting (with BLE)");

    /* Init sensors */
    int err;

    sensor_wq_init();

    (void)i2c_scan_log(i2c0);

    err = sen0546_init();
    if (err) {
        LOG_ERR("SEN0566 init failed (%d)", err);
    }

    err = mlx90614_init();
    if (err) {
        LOG_ERR("MLX90614 init failed (%d)", err);
    }

    err = ADS1015_init();
    if (err) {
        LOG_ERR("ADS1015 init failed (%d)", err);
    }

    err = analog_init();
    if (err) {
        LOG_ERR("Analog init failed (%d)", err);
    }

    // load point
    err = hx711_probe_init();
    if (err) {
        LOG_ERR("HX711 Load point init failed (%d)", err);
    }

    // initialize relay gpio
    pump_init();
    pump_schedule_init();

    //calibrate scale
    calibrate();

    //init ble
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return err;
    }

    bt_conn_cb_register(&connection_callbacks);

    LOG_INF("Bluetooth initialized\n");
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

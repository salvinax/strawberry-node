#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#include <sensor/hx711/hx711.h>
#include <stddef.h>
#include "drv_load_point.h"

LOG_MODULE_REGISTER(load_point, LOG_LEVEL_INF);

const struct device *hx711_dev = DEVICE_DT_GET_ANY(avia_hx711);


static void set_rate(void)
{
	static struct sensor_value rate_val;

	rate_val.val1 = HX711_RATE_10HZ;
	sensor_attr_set(hx711_dev,
			HX711_SENSOR_CHAN_WEIGHT,
			SENSOR_ATTR_SAMPLING_FREQUENCY,
			&rate_val);
}


int hx711_probe_init(void)
{
    if (hx711_dev == NULL) {
        LOG_ERR("No HX711 devicetree node found (status=okay)");
        return -ENODEV;
    }

    if (!device_is_ready(hx711_dev)) {
        LOG_ERR("HX711 device not ready (driver init failed?)");
        return -ENODEV;
    }

	set_rate();
    LOG_INF("HX711 dev OK: %s", hx711_dev->name);
    return 0;
}


void measure(struct load_sample *l) {
	static struct sensor_value weight;
	int ret;

	ret = sensor_sample_fetch(hx711_dev);
	if (ret != 0) {
		LOG_ERR("Cannot take measurement: %d", ret);
	} else {
		sensor_channel_get(hx711_dev, HX711_SENSOR_CHAN_WEIGHT, &weight);
		l->integer = weight.val1;
		l->fractional = weight.val2;
	}
}

void calibrate(void) {
	float calibration_weight = 55.99f; //known weight of object
	
	LOG_INF("Device is %p, name is %s", hx711_dev, hx711_dev->name);
	LOG_INF("Calculating offset...");
	avia_hx711_tare(hx711_dev, 5);

	LOG_INF("Waiting for known weight of %.2f grams...", (double)calibration_weight);

	for (int i = 5; i >= 0; i--) {
		LOG_INF(" %d..", i);
		k_msleep(1000);
	}

	LOG_INF("Calculating slope...");
	avia_hx711_calibrate(hx711_dev, calibration_weight, 5);
}
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

#include "drv_sen0546.h"

// https://dfimg.dfrobot.com/nobody/wiki/4c8e1057e1c118e5c72f8ff6147575db.pdf
LOG_MODULE_REGISTER(sen0546, LOG_LEVEL_INF);

#define I2C_NODE DT_NODELABEL(sen0546)
static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(I2C_NODE);

//msb first
static int probe_cht8305(void)
{   
    
    uint8_t addr = MANUFACTURE_ID_CHT8305_REG;
    uint8_t rx[2] = {0};
    int err = i2c_write_read_dt(&i2c_dev, &addr, 1, rx, sizeof(rx));
    if (err) return err;

    if (!(rx[0] == 0x59 && rx[1] == 0x59)) {
        LOG_INF("NOT EXPECTED MANUFACTURER ID");
        return -EIO; /* Not a CHT8305 */
    }

    return 0;
   
}


int sen0546_init(void)
{
    if (!device_is_ready(i2c_dev.bus)) return -ENODEV;

    k_sleep(K_MSEC(20));

    int err = probe_cht8305();
    if (err == 0) {
        LOG_INF("SEN0546 detected: CHT8305 @0x40");
        return 0;
    }

    LOG_WRN("SEN0546 not detected on 0x40 (last err=%d)", err);
    return -ENODEV;
}

int sen0546_read(struct sen0546_sample *out)
{

    // get temperature and humidity 
    uint8_t cmd =  TEMP_CHT8305_REG; 
    int err = i2c_write_dt(&i2c_dev,&cmd,1);
    if (err) return err;

    // wait conversion time
    k_sleep(K_MSEC(20)); 

    uint8_t buf[4] = {0};
    err = i2c_read_dt(&i2c_dev, buf, sizeof(buf));
    if (err) return err;

    uint16_t t_raw  = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t rh_raw = ((uint16_t)buf[2] << 8) | buf[3];

    out->temp_c = (float)((((float)t_raw * 165.0f) / 65535.0f) - 40.0f);
    out->rh    = (float)((100.0f * ((float)rh_raw / 65535.0f)));
    return 0;
}

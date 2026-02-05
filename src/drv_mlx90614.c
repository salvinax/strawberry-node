#include "drv_mlx90614.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
//https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/hat/MLX90614-Datasheet-Melexis_en.pdf
//https://github.com/m5stack/M5Stack/blob/master/examples/Unit/NCIR_MLX90614/NCIR_MLX90614.ino
// https://docs.m5stack.com/en/unit/ncir 3.3v logic pins

//infrared temperature sensor
LOG_MODULE_REGISTER(mlx, LOG_LEVEL_INF);

#define REG_TA   0x06   // ambient temperature
#define REG_TO1  0x07   // IR1 in 0x07 and IR2 0x08

#define I2C_NODE DT_NODELABEL(mlx90614)
static const struct i2c_dt_spec i2c_dev = I2C_DT_SPEC_GET(I2C_NODE);

// skip pec for now but implement later 
// skip sleep mode for now but implement later

static int mlx_read_word(uint8_t reg, uint16_t *out)
{
    uint8_t rx[3] = {0};
    int err = i2c_write_read_dt(&i2c_dev, &reg, 1, rx, sizeof(rx));
    if (err) {
        return err;
    }

    uint16_t raw = ((uint16_t)rx[1] << 8) | rx[0];

    // MSB can be an error flag on MLX90614 readings sometimes
    if (raw & 0x8000u) {
        return -EIO;
    }

    *out = raw;
    return 0;
}


int mlx90614_init(void)
{
    if (!device_is_ready(i2c_dev.bus)) {
        LOG_INF("MLX I2C bus not ready\n");
        return -1;
    }
    // probe - read temp once
    uint16_t raw = 0;
    int err = mlx_read_word(REG_TO1, &raw);
    if (err) return err;

    LOG_INF("MLX90614 probe OK (raw=0x%04X)", raw);
    return 0;
}

int mlx90614_read(struct mlx90614_sample *out)
{
    
    uint16_t amb_raw = 0, obj_raw = 0;
    int err = mlx_read_word(REG_TA, &amb_raw);
    if (err) return err;
    err = mlx_read_word(REG_TO1, &obj_raw);
    if (err) return err;

    // or scale it by 100 and store in int32
    float amb_c = ((float)amb_raw * 0.02f) - 273.15f;
    float obj_c = ((float)obj_raw * 0.02f) - 273.15f;

    out->amb_c = amb_c;
    out->obj_c = obj_c;
    return 0;
}

#include <zephyr/device.h>
#include <stdint.h>


//https://wiki.dfrobot.com/SKU_SEN0546_I2C_Temperature_and_Humidity_Sensor_Stainless_Steel_Shell

// we have the cht8305
#define SEN0546_ADDR_CHT8305  0x40

#define TEMP_CHT8305_REG 0x00
#define HUMIDITY_CHT8305_REG 0x01
#define CONFIG_CHT8305_REG 0x02
#define ALERT_CHT8305_REG 0x02
#define MANUFACTURE_ID_CHT8305_REG 0xFE
#define VERSION_ID_CHT8305_REG 0xFF

struct sen0546_sample {
    float temp_c;  // temperature in centi-degC
    float rh;      // relative humidity in %RH * 100 
};


int sen0546_init(void);
int sen0546_read(struct sen0546_sample *out);

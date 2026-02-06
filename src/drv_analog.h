#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include "drv_ads1015.h"

struct soil_sample {
    float voltage;      // measured divider voltage
    float resistance;   // computed thermistor resistance
    float temp_c;       // estimated temperature
};

struct wind_sample {
    float voltage;      // raw analog voltage
    float temp_in_c;
    float mph;
};

struct par_sample {
    float voltage;     // shunt voltage (V)
    float current_mA;  // loop current (mA)
    float ppfd;      // µmol m^-2 s^-1
};

// struct pyr_sample {
//     int16_t mv;       /* sensor output in millivolts */
//     int16_t w_m2;     /* computed shortwave irradiance (W/m^2) */
// };

// struct sl610_sample {
//     float v_tp_mV;     /* thermopile differential (mV); + => target hotter than detector */
//     float t_det_c;     /* detector temperature (°C) from thermistor */
//     float lw_w_m2;     /* longwave irradiance (W/m^2) using k1/k2 */
//     float v_th_mV;     /* thermistor node Vout (mV), debug */
// };

int analog_init(void);
int soil_read(struct soil_sample *s);
int wind_read(struct wind_sample *w);
int par_read(struct par_sample *p);
int sl160_thermistor_read(struct pyrgeometer_ADS_sample *p);

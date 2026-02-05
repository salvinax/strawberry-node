#include "drv_analog.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h> 
LOG_MODULE_REGISTER(analog, LOG_LEVEL_INF);

// connected to PAR sensor, wind, thermistor, root temperature
static const struct adc_dt_spec adc_chan0 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_chan1 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);
static const struct adc_dt_spec adc_chan2 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2);
static const struct adc_dt_spec adc_chan3 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3);
static const struct adc_dt_spec adc_chan4 = ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4);



static int adc_setup_one(const struct adc_dt_spec *ch)
{
	if (!adc_is_ready_dt(ch)) {
		LOG_ERR("ADC dev not ready: %s", ch->dev->name);
		return -ENODEV;
	}
	return adc_channel_setup_dt(ch);
}


int analog_init(void)
{
    int err;
    err = adc_setup_one(&adc_chan0); if (err) return err;
    err = adc_setup_one(&adc_chan1); if (err) return err;
    err = adc_setup_one(&adc_chan2); if (err) return err;
    err = adc_setup_one(&adc_chan3); if (err) return err;
    err = adc_setup_one(&adc_chan4); if (err) return err;
    return 0;
}


static int adc_read_one_mv(const struct adc_dt_spec *ch, int16_t *raw, int32_t *mv)
{
	int err;

	struct adc_sequence seq = {
		.buffer = raw,
		.buffer_size = sizeof(*raw),
	};

	err = adc_sequence_init_dt(ch, &seq);
	if (err) return err;

	err = adc_read(ch->dev, &seq);
	if (err) return err;

	*mv = (int32_t)(*raw);
	(void)adc_raw_to_millivolts_dt(ch, mv);
	return 0;
}


#define JS8746_R0_OHMS     10000.0f
#define JS8746_T0_K        (25.0f + 273.15f)
#define JS8746_BETA_K      3977.0f

#define VDD_MV             3300.0f   
#define R_PULLUP_OHMS      12000.0f

// https://amphenol-sensors.com/hubfs/Documents/AAS-920-747B-Thermometrics%20JS8746-Temp-Sensor-053119-web.pdf
int soil_read(struct soil_sample *s)
{
    int err;
    int16_t raw;
    int32_t mv;

    err = adc_read_one_mv(&adc_chan3, &raw, &mv);
    if (err) return err;

    float v_out = (float)mv / 1000.0f;
    float vdd   = (float)VDD_MV / 1000.0f;

    float denom = (vdd - v_out);
    float r_ntc = R_PULLUP_OHMS * (v_out / denom);
    //https://www.giangrandi.org/electronics/ntc/ntc.shtml
    float inv_T = (1.0f / JS8746_T0_K) + (1.0f / JS8746_BETA_K) * logf(r_ntc / JS8746_R0_OHMS);
    float temp_k = 1.0f / inv_T;
    float temp_c = temp_k - 273.15f; // convert to c

    s->voltage =  v_out;
    s->resistance =  r_ntc;
    s->temp_c =  temp_c;

    return 0;
}

#define WIND_SUPPLY 5.0f
#define R_TOP_OHMS        1000.0f       
#define R_BOT_OHMS        2000.0f

// Divider gain: Vsig = Vadc * (Rtop+Rbot)/Rbot 
#define DIV_GAIN ((R_TOP_OHMS + R_BOT_OHMS)/R_BOT_OHMS)
#define WIND_ADJUST 0.0f

int wind_read(struct wind_sample *w)
{
    int16_t raw_temp, raw_rv;
	int32_t mv_temp, mv_rv;
    int err = adc_read_one_mv(&adc_chan1, &raw_temp, &mv_temp);
    if (err) return err;
    err = adc_read_one_mv(&adc_chan2, &raw_rv, &mv_rv);
    if (err) return err;

    // convert to 5v (remove reistor divideer) + convert to volts
    float v_tmp_sig = (mv_temp/1000.0f) * DIV_GAIN;
    float v_rv_sig = (mv_rv/1000.0f) * DIV_GAIN;

    // Convert TMP volts -> counts/steps
    // following arduino regression sketch
    float TMP_Therm_ADunits = (v_tmp_sig) * (1024.0f / WIND_SUPPLY);

    float TempCtimes100 = (0.005f * (TMP_Therm_ADunits * TMP_Therm_ADunits)) - (16.862f * TMP_Therm_ADunits)+ 9075.4f;

	float zeroWind_ADunits = -0.0006f * (TMP_Therm_ADunits * TMP_Therm_ADunits) + (1.0727f * TMP_Therm_ADunits) + 47.172f;

	float zeroWind_volts = (zeroWind_ADunits * (WIND_SUPPLY/ 1024.0f)) - WIND_ADJUST;
    
    float base = (v_rv_sig - zeroWind_volts)/0.2300f;
    float WindSpeed_MPH;

    if (!isfinite(base)|| base <= 0.0f){
        WindSpeed_MPH = 0.0f;
    } else {
        WindSpeed_MPH = powf(base, 2.7265f);
    }

// LOG_INF("mv_tmp_adc=%.3f mV  mv_rv_adc=%.3f mV", (double)mv_temp, (double)mv_rv);
// LOG_INF("v_tmp_sig=%.3f V  v_rv_sig=%.3f V", (double)v_tmp_sig, (double)v_rv_sig);
// LOG_INF("TMP_ADunits=%.1f  TempC=%.2f",
//         (double)TMP_Therm_ADunits, (double)(TempCtimes100/100.0f));
    
    w->voltage = v_rv_sig;
    w->mph = WindSpeed_MPH;
    w->temp_in_c = TempCtimes100/100.0f;

    return 0;

}


#define PAR_SHUNT_OHMS 120.0f

int par_read(struct par_sample *p)
{
    int16_t raw;
	int32_t mv;
    int err = adc_read_one_mv(&adc_chan0, &raw, &mv);
    if (err) return err;

    float v = (float)mv / 1000.0f;  
	float i_mA = (v / PAR_SHUNT_OHMS) * 1000.0f;      

	// sq-214 scaling
	float ppfd = 250.0f * (i_mA - 4.0f);
	if (ppfd < 0.0f) ppfd = 0.0f;

	p->voltage = v;
	p->current_mA = i_mA;
	p->ppfd = (int32_t)(ppfd + 0.5f);    

    return 0;
}

#define RESISTOR_THERMISTOR 22000.0f
#define EXCITATION_VOLTAGE 33f

#define SH_A_NEG  9.32960e-4f
#define SH_B_NEG  2.21424e-4f
#define SH_C_NEG  1.26329e-7f
#define SH_A_POS  9.32794e-4f
#define SH_B_POS  2.21451e-4f
#define SH_C_POS  1.26233e-7f

static float steinhart_kelvin(float rt_ohm)
{
    const float lnR = logf(rt_ohm);
    // take care of negative and positive cases 
    float invT = SH_A_POS + SH_B_POS*lnR + SH_C_POS*lnR*lnR*lnR;
    float Tc   = (1.0f/invT) - 273.15f;
    if (Tc < -5.0f) {
        invT = SH_A_NEG + SH_B_NEG*lnR + SH_C_NEG*lnR*lnR*lnR;
    }
    return 1.0f / invT;
}

int sl160_thermistor_read(struct pyrgeometer_ADS_sample *s) 
{
    int16_t raw;
	int32_t vout_mv;
    int err = adc_read_one_mv(&adc_chan4, &raw, &vout_mv);
    if (err) return err;

   float thermistor_resistance = RESISTOR_THERMISTOR * ((float)vout_mv / (float)(VDD_MV - vout_mv));

   // get temperature 
    s->temperature_in_K = steinhart_kelvin(thermistor_resistance);
    return 0;
}
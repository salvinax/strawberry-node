#include "drv_ads1015.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#define ADS_NODE DT_NODELABEL(ads1015)
static const struct i2c_dt_spec ads_i2c = I2C_DT_SPEC_GET(ADS_NODE);
// ADC with pyrgeometer and pyranometer connected
LOG_MODULE_REGISTER(ads1015, LOG_LEVEL_INF);
// holds all settings
static ADS1015 ads1015_module;

uint8_t ads1015_txBuff[3];
uint8_t ads1015_rxBuff[2];

static void ADS1015_WriteRegister(uint8_t reg, uint16_t value);
static int16_t ADS1015_ReadRegister(uint8_t reg);
static float get_lsb_multiplier(adsGain_t gain);
//static uint32_t get_delay_usec(adsSPS_t sps);
static uint32_t get_delay_msec(adsSPS_t sps);
static uint16_t get_adc_config();

void ADS1015_WriteRegister (uint8_t reg, uint16_t value){
//	uint8_t data[3];
	ads1015_txBuff[0] = reg;
	ads1015_txBuff[1] = (value >> 8);
	ads1015_txBuff[2] = (value & 0xFF);

  i2c_write_dt(&ads_i2c, ads1015_txBuff,3);
}

int16_t ADS1015_ReadRegister(uint8_t reg){
//	uint8_t data[2];
	ads1015_txBuff[0] = reg;

  i2c_write_dt(&ads_i2c,ads1015_txBuff,1);
  i2c_read_dt(&ads_i2c,ads1015_rxBuff,2);

	return (ads1015_rxBuff[0] << 8) | ads1015_rxBuff[1];
}

// for ADS1015 DIFFERENT FOR ADS115 which we will be using in the pcb
float get_lsb_multiplier(adsGain_t gain){
	float lsb_multiplier=0;
  switch (gain) {
  case (GAIN_TWOTHIRDS):
    lsb_multiplier = 3;
    break;
  case (GAIN_ONE):
    lsb_multiplier = 2;
    break;
  case (GAIN_TWO):
    lsb_multiplier = 1;
    break;
  case (GAIN_FOUR):
    lsb_multiplier = 0.5;
    break;
  case (GAIN_EIGHT):
    lsb_multiplier = 0.25;
    break;
  case (GAIN_SIXTEEN):
    lsb_multiplier = 0.125;
    break;
	default:
    lsb_multiplier = 0.0625;
    break;
  }
	return lsb_multiplier;
}

/*
uint32_t get_delay_usec(adsSPS_t sps){
	uint32_t delay = 0;
  switch (sps) {
  case (SPS_8):
    delay = 125000;
    break;
  case (SPS_16):
    delay = 62500;
    break;
  case (SPS_32):
    delay = 31250;
    break;
  case (SPS_64):
    delay = 15625;
    break;
  case (SPS_128):
    delay = 7825;
    break;
  case (SPS_250):
    delay = 4000;
    break;
  case (SPS_475):
    delay = 2150;
    break;
  case (SPS_860):
    delay = 1200;
    break;
	default:
    delay = 7825;
    break;
  }
	return delay;
}
*/
uint32_t get_delay_msec(adsSPS_t sps){
	uint32_t delay = 0;
  switch (sps) {
  case (SPS_8):
    delay = 125;
    break;
  case (SPS_16):
    delay = 63;
    break;
  case (SPS_32):
    delay = 32;
    break;
  case (SPS_64):
    delay = 16;
    break;
  case (SPS_128):
    delay = 8;
    break;
  case (SPS_250):
    delay = 4;
    break;
  case (SPS_475):
    delay = 3;
    break;
  case (SPS_860):
    delay = 2;
    break;
	default:
    delay = 8;
    break;
  }
	return delay;
}

uint16_t get_adc_config(){
	ads1015_module.m_conversionDelay = get_delay_msec(ads1015_module.m_samplingRate);
	ads1015_module.m_lsbMultiplier = get_lsb_multiplier(ads1015_module.m_gain)/1000;

  return
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ads1015_module.m_compMode |							// Comparator Mode
      ads1015_module.m_samplingRate |					// Sampling Rate
      ads1015_module.m_gain |									// PGA Gain
      ads1015_module.m_mode;									// ADC mode
}

/**************************************************************************/
/*!
    @brief  Reset a ADS1015
    @param  Device struct
*/
/**************************************************************************/
int ADS1015_reset(void){
//	uint8_t cmd = 0x06;
	ads1015_txBuff[0] = 0x06;

  return i2c_write_dt(&ads_i2c,ads1015_txBuff,1);
}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range
    @param gain gain setting to use
*/
/**************************************************************************/
void ADS1015_setGain(adsGain_t gain){
	ads1015_module.m_gain = gain;
}

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range
    @return the gain setting
*/
/**************************************************************************/
adsGain_t ADS1015_getGain(){
	return ads1015_module.m_gain;
}

/**************************************************************************/
/*!
    @brief  Sets the Sampling rate
		@param  sps: sampling rate to use
*/
/**************************************************************************/
void ADS1015_setSPS(adsSPS_t sps){
	ads1015_module.m_samplingRate = sps;
}

/**************************************************************************/
/*!
    @brief  Gets the Sampling rate
    @return the sampling rate
*/
/**************************************************************************/
adsSPS_t ADS1015_getSPS(){
	return ads1015_module.m_samplingRate;
}

/**************************************************************************/
/*!
    @brief  Sets a ADC conversion mode setting
    @param  mode
*/
/**************************************************************************/
void ADS1015_setCONV(adsCONV_t mode){
	ads1015_module.m_mode = mode;
}

/**************************************************************************/
/*!
    @brief  Gets a ADC conversion mode setting
    @return the conversion setting
*/
/**************************************************************************/
adsCONV_t ADS1015_getCONV(){
	return ads1015_module.m_mode;
}

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1015 struct w/appropriate properties
    @param  Device struct
*/
/**************************************************************************/
int ADS1015_init(void)
{

  if (!device_is_ready(ads_i2c.bus)) {
        return -1;
  }

  int err = ADS1015_reset();
  if (err) return -1;

	// Start with default values
	ads1015_module.m_samplingRate = SPS_128;							// 128 samples per second (default)
	ads1015_module.m_mode 		= ADS1015_REG_CONFIG_MODE_SINGLE;	// Single-shot mode (default)
	ads1015_module.m_gain 		= GAIN_SIXTEEN;					// +/- 6.144V range (limited to VDD +0.3V max!)
	ads1015_module.m_compMode 	= CMODE_TRAD;						// Traditional comparator (default val)

  ads1015_module.config =
      ADS1015_REG_CONFIG_CQUE_NONE |    // Disable the comparator (default val)
      ADS1015_REG_CONFIG_CLAT_NONLAT |  // Non-latching (default val)
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ads1015_module.m_compMode |				// Comparator Mode
      ads1015_module.m_samplingRate |			// Sampling Rate
      ads1015_module.m_gain |					// PGA Gain
      ads1015_module.m_mode;					// ADC mode

	ads1015_module.m_conversionDelay = (uint8_t)ADS1015_CONVERSIONDELAY;

  return 0;
}


/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel
    @param channel ADC channel to read
    @return the ADC reading
*/
/**************************************************************************/
float ADS1015_readADC(adc_Ch_t channel){
  if ((channel & ~ADS1015_REG_CONFIG_MUX_MASK) != 0) {
    return 0;
  }

  uint16_t config = get_adc_config();;
	config |= channel;
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

	ads1015_module.config = config;

  // check ready flag
	uint16_t ret_val;
	do{
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}
	while( (ret_val & ADS1015_REG_CONFIG_OS_MASK) == ADS1015_REG_CONFIG_OS_BUSY);

  //write config... make sure its correct -> do readback
	do{
		ADS1015_WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}while( (ret_val & ADS1015_REG_CONFIG_MUX_MASK) != (config & ADS1015_REG_CONFIG_MUX_MASK) );

	k_sleep(K_MSEC(ads1015_module.m_conversionDelay));

  // poll to see if conversion if finished 
	do{
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}
	while( (ret_val & ADS1015_REG_CONFIG_OS_MASK) == ADS1015_REG_CONFIG_OS_BUSY);


  // if sinigle ended takes abs() force positive else signed values
	if(channel>ADS1015_REG_CONFIG_MUX_DIFF_2_3)
		return (float)abs(ADS1015_ReadRegister(ADS1015_REG_POINTER_CONVERT)) * ads1015_module.m_lsbMultiplier;
	else
		return (float)ADS1015_ReadRegister(ADS1015_REG_POINTER_CONVERT) * ads1015_module.m_lsbMultiplier;
}

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel
    @param channel ADC channel to read
    @return the ADC reading
*/
/**************************************************************************/
int16_t ADS1015_readADC_raw( adc_Ch_t channel){


  // make sure nothing is set outside of MUX fields!!!
  if ((channel & ~ADS1015_REG_CONFIG_MUX_MASK) != 0) {
    return 0;
  }

  uint16_t config = get_adc_config();
	config |= channel;
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

	ads1015_module.config = config;

  // check busy or ready flag
  // make sure its ready
	uint16_t ret_val = ADS1015_REG_CONFIG_OS_BUSY;
	do{
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}
	while( (ret_val & ADS1015_REG_CONFIG_OS_MASK) == ADS1015_REG_CONFIG_OS_BUSY);


	do{
		ADS1015_WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}while( (ret_val & ADS1015_REG_CONFIG_MUX_MASK) != (config & ADS1015_REG_CONFIG_MUX_MASK) );


  k_sleep(K_MSEC(ads1015_module.m_conversionDelay));


	do{
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}
	while( (ret_val & ADS1015_REG_CONFIG_OS_MASK) == ADS1015_REG_CONFIG_OS_BUSY);

	return ADS1015_ReadRegister(ADS1015_REG_POINTER_CONVERT);
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.
            This will also set the ADC in continuous conversion mode.
    @param channel ADC channel to use
    @param threshold comparator threshold
*/
/**************************************************************************/
void ADS1015_startComparator_SingleEnded(uint8_t channel, int16_t threshold){
	// Start with default values
  uint16_t config =
      ADS1015_REG_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1 match
      ADS1015_REG_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ads1015_module.m_compMode |							// Comparator Mode
      ads1015_module.m_samplingRate |					// Sampling Rate
			ads1015_module.m_gain |									// ADC gain setting
      ADS1015_REG_CONFIG_MODE_CONTIN;   // Continuous conversion mode

  // Set single-ended input channel
  switch (channel) {
  case (0):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
    break;
  case (1):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
    break;
  case (2):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
    break;
  case (3):
    config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
    break;
  }

	ads1015_module.config = config;

  // Set the high threshold register
	ADS1015_WriteRegister(ADS1015_REG_POINTER_HITHRESH, threshold);

  // Write config register to the ADC
  ADS1015_WriteRegister(ADS1015_REG_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.
    @return the last ADC reading
*/
/**************************************************************************/
float ADS1015_getLastConversionResults(void){
	uint16_t ret_val = ADS1015_REG_CONFIG_OS_BUSY;
  // Wait for the conversion to complete
  k_sleep(K_MSEC(ads1015_module.m_conversionDelay));
	do{
		ret_val = ADS1015_ReadRegister(ADS1015_REG_POINTER_CONFIG);
	}
	while( (ret_val & ADS1015_REG_CONFIG_OS_MASK) == ADS1015_REG_CONFIG_OS_BUSY);

  // Read the conversion results
	return (float)ADS1015_ReadRegister(ADS1015_REG_POINTER_CONVERT) * ads1015_module.m_lsbMultiplier;
}

/************************************************************************/
// Pyranometer (SP-610-SS)
// 

// 
void get_sp610_pyranometer(struct pyranometer_ADS_sample *p)
{
  //sensor output is 0 to 70mv
  // set gain to 16
  //AIN0 (pos) /AIN1 (neg)
  int16_t counts;

  ADS1015_setGain(GAIN_SIXTEEN);

  // read register then shift right 4 bits - ads1015 only
  // ads1015 stores conversion as two bit complement
  counts = ADS1015_readADC_raw(ADS1015_REG_CONFIG_MUX_DIFF_0_1) >> 4;
  if (counts > 0x07FF) {
    // if we have negative number extend the sign to the 16th bit. 
    counts |= 0xF000;
  } 
  float v_signal = (float)counts * ads1015_module.m_lsbMultiplier;
  float mv_signal = v_signal * 1000.0f; 

  float calibrated_radiation = (float)mv_signal  * 26.72f;//CALIBRATION CONSTANT FROM CERTIFICATE sn 1665

  if (calibrated_radiation < 0) {
    calibrated_radiation = 0;
  }

  // convert sensor output to shortwave radiation in w/m^2
  p->shortwave_radiation = calibrated_radiation;
  
}


// Pyrgeometer (SL-610-SS)
//output is +/- 23mV
//set gain to 16x gain  +/- 0.256V  1 bit = 0.0078125mV
void get_sl610_thermopile(struct pyrgeometer_ADS_sample *p) {
  //AIN2 (pos) /AIN3 (neg)
  int16_t counts;
  ADS1015_setGain(GAIN_SIXTEEN);

  counts = ADS1015_readADC_raw(ADS1015_REG_CONFIG_MUX_DIFF_2_3) >> 4;
  if (counts > 0x07FF) {
    // if we have negative number extend the sign to the 16th bit. 
    counts |= 0xF000;
  }

  
  float signal_thermopile = (float)counts * ads1015_module.m_lsbMultiplier; // convert to volts
  float mv_signal = signal_thermopile * 1000.0f;  // convert to mV

  p->thermopile_signal_mv_diff = mv_signal;
}
#include "application/measurement.h"
#include "application/driver_init.h"
#include <string.h>
#include "application/i2cbus.h"
#include "application/time.h"

struct micromini_measurement measurement;
static struct micromini_measurement next_measurement;
volatile int measurement_queued = 0;
static int measurement_in_progress = 0;
static int measurement_done = 0;
static int last_scheduled;
volatile uint8_t nmeasurements = 0;

volatile uint8_t ain[AIN_SIZE+1];
volatile int ain_ready;
volatile uint8_t ain_nread_div_8_m1 = 63;
volatile uint16_t ain_hist[256];
volatile uint8_t ain_hist_mode_bin;
volatile uint8_t ain_hist_max;
volatile uint8_t ain_hist_min;


static enum ain_source source = DEFAULT_AIN_SOURCE;
static enum ain_source new_source = DEFAULT_AIN_SOURCE;
static const uint8_t source_map[] = { [SOURCE_AIN1] = AIN1, [SOURCE_AIN12] = AIN12, [SOURCE_AIN13] = AIN13, [SOURCE_BATMON] = BAT_MON , [SOURCE_TEMP] = 0x18};

volatile uint8_t thresh_rising = 128;
volatile uint8_t thresh_falling = 128;
volatile uint8_t N_rising = 0;
volatile uint8_t N_falling = 0;

static uint8_t gain_cfg = 0;
static uint8_t rate_cfg = 0;
static uint8_t new_gain_cfg = 0;
static uint8_t new_rate_cfg = 0;

static void ain_cb(const struct adc_dma_descriptor * const d)
{
  (void) d;
  adc_dma_disable_channel(&ANALOGIN,0);
  //compute hist

  ain_hist_max = 0;
  ain_hist_min = 0xff;
  uint16_t biggest = 0;
  //fill histogram, skip first sample
  for (int i = 1; i <= 8*(1+ain_nread_div_8_m1); i++)
  {
    uint8_t val = ain[i];
    ain_hist[val]++;
    if (ain_hist[val] > biggest)
    {
      biggest = ain_hist[val];
      ain_hist_mode_bin = val;
    }
    if (val > ain_hist_max) ain_hist_max = val;
    if (val < ain_hist_min) ain_hist_min = val;

  }
  recalculate_crossings();
  ain_ready = 1;
}

void recalculate_crossings()
{
  ain_ready =0;
  N_rising =0;
  N_falling = 0;
  for (int i = 1; i <= 8*(1+ain_nread_div_8_m1); i++)
  {
    if (i > 1 && ain[i] >= thresh_rising && ain[i-1] < thresh_rising) N_rising++;
    if (i > 1 && ain[i] <= thresh_falling && ain[i-1] > thresh_falling) N_falling++;
  }
  ain_ready =1;
}


#define LTC2992_ADDR 0x6f
#define LTC2992_REG_CTRL_A 0x00  //control  register 1
#define LTC2992_REG_CTRL_B 0x01  // control regiser 2
#define LTC2992_REG_NADC 0x04  // set ADC resolution
#define LTC2992_REG_I1_MSB 0x14 //deltaSense1
#define LTC2992_REG_I1_LSB 0x15
#define LTC2992_REG_S1_MSB 0x1e //sense1 V 
#define LTC2992_REG_S1_LSB 0x1f
#define LTC2992_REG_I2_MSB 0x46 //deltaSense2
#define LTC2992_REG_I2_LSB 0x47
#define LTC2992_REG_S2_MSB 0x50 //sense2 V 
#define LTC2992_REG_S2_LSB 0x51
#define LTC2992_REG_GPIO_MSB 0x29 
#define LTC2992_REG_GPIO_LSB 0x29
#define LTC2992_MASK_LTC_ADC_RESOLUTION  0x80
#define LTC2992_MASK_LTC_SNAPSHOT_MEASURE_S12  0x06
#define LTC2992_MASK_LTC_MODE_SHUTDOWN  0x60
#define LTC2992_MASK_LTC_MODE_SINGLE_CYCLE  0x40
#define LTC2992_MASK_LTC_MODE_SNAPSHOT  0x20
#define LTC2992_MASK_LTC_MODE_CONTINUOUS  0x0
#define LTC2992_ADC_STATUS 0x32  //status register
#define LTC2992_MASK_READY  0xc0  // I think... the 2 MSB if i did this right


#define TMP432_ADDRESS 0x4c
#define TMP432_REG_CFG1 0x09  // write-only
#define TMP432_REG_STATUS 0x02  // read only
#define TMP432_REG_ONESHOT 0x0f  // write only
#define TMP432_REG_LOCAL_TEMP_HIGH 0x0
#define TMP432_REG_REMOTE1_TEMP_HIGH 0x1
#define TMP432_REG_REMOTE2_TEMP_HIGH 0x23
#define TMP432_REG_LOCAL_TEMP_LOW 0x29
#define TMP432_REG_REMOTE1_TEMP_LOW 0x10
#define TMP432_REG_REMOTE2_TEMP_LOW 0x24
#define TMP432_EXTENDED_TEMPERATURE_MASK 0x04
#define TMP432_SD_MASK 0x40
#define TMP432_BUSY_MASK 0x80


struct ltc2992_ctx
{
  uint8_t addr;
  int last_read;
  uint16_t sense1;
  uint16_t sense2;
  uint16_t delta_sense1;
  uint16_t delta_sense2;
};






static struct ltc2992_ctx ltc = { .addr = LTC2992_ADDR };

void measurement_init()
{
	adc_dma_init(&ANALOGIN, ADC);
  adc_dma_set_inputs(&ANALOGIN, source_map[DEFAULT_AIN_SOURCE], 0x19,0);
  adc_dma_set_conversion_mode(&ANALOGIN, ADC_CONVERSION_MODE_FREERUN);
  adc_dma_register_callback(&ANALOGIN, ADC_DMA_COMPLETE_CB, ain_cb);

  //set temperature range, shutdown mode
  i2c_task_t tmp432_task = {.addr = TMP432_ADDRESS, .reg=TMP432_REG_CFG1, .write=1, .data = TMP432_EXTENDED_TEMPERATURE_MASK | TMP432_SD_MASK }; 
  i2c_enqueue(&tmp432_task);


}

static inline uint16_t read12bitADC(uint16_t msb, uint16_t lsb)
{
  return (( msb << 8) | lsb) >> 4; 

}


static uint16_t get_adc(uint8_t address, uint8_t reg_msb, uint8_t reg_lsb) 
{
  i2c_task_t msb = { .addr = address, .write = 0, .reg = reg_msb } ; 
  i2c_task_t lsb = { .addr = address, .write = 0, .reg = reg_lsb }; ; 
  i2c_enqueue(&msb);
  i2c_enqueue(&lsb);
  while (!lsb.done);
  return read12bitADC(msb.data,lsb.data); 
}


void update_ltc_if_ready(struct ltc2992_ctx * c)
{

  //check status on power system
  i2c_task_t check= {.addr= c->addr, .reg=LTC2992_ADC_STATUS, .write = 0, .data = 0}; 
  i2c_enqueue(&check);
  while (!check.done);
  if ( (check.data & LTC2992_MASK_READY) == LTC2992_MASK_READY) //both ADC and IADC ready 
  {
    c->sense1 = get_adc(c->addr, LTC2992_REG_S1_MSB, LTC2992_REG_S1_LSB);
    c->sense2 = get_adc(c->addr, LTC2992_REG_S2_MSB, LTC2992_REG_S2_LSB);
    c->delta_sense1 = get_adc(c->addr, LTC2992_REG_I1_MSB, LTC2992_REG_I1_LSB);
    c->delta_sense2 = get_adc(c->addr, LTC2992_REG_I2_MSB, LTC2992_REG_I2_LSB); 
    c->last_read = last_scheduled;
  }


  //check temperature status
}
void get_temp(uint8_t high_reg, uint8_t low_reg, uint8_t * dest)
{
  i2c_task_t task_high = {.addr=TMP432_ADDRESS, .reg = high_reg} ;
  i2c_enqueue(&task_high);
  i2c_task_t task_low = {.addr=TMP432_ADDRESS, .reg = low_reg} ;
  i2c_enqueue(&task_low);
  while (!task_high.done);
  dest[0] = task_low.data;
  dest[1] = task_high.data;
}

static int last_temp_read;
static uint8_t last_local_t[2];
static uint8_t last_remote1_t[2];
static uint8_t last_remote2_t[2];
void update_temps_if_ready()
{
  i2c_task_t check = {.addr=TMP432_ADDRESS, .reg=TMP432_REG_STATUS }; 
  i2c_enqueue(&check);
  while (!check.done);
  if (check.done < 0) return;
  if ((check.data & TMP432_BUSY_MASK) == 0)
  {
    get_temp(TMP432_REG_LOCAL_TEMP_HIGH, TMP432_REG_LOCAL_TEMP_LOW, last_local_t);
    get_temp(TMP432_REG_REMOTE1_TEMP_HIGH, TMP432_REG_REMOTE1_TEMP_LOW, last_remote1_t);
    get_temp(TMP432_REG_REMOTE2_TEMP_HIGH, TMP432_REG_REMOTE2_TEMP_LOW, last_remote2_t);
    last_temp_read = last_scheduled;
  }
}



int measurement_process()
{

  //handle potential ADC changes here
  if (!measurement_queued && !measurement_in_progress)
  {
    if (new_source != source) 
    {
      //do we need to restart ? 
      adc_dma_set_inputs(&ANALOGIN, source_map[new_source], 0x19,0);
      source = new_source;
    }

    if (new_gain_cfg != gain_cfg)
    {
      adc_dma_set_channel_gain(&ANALOGIN,0, gain_cfg & 0xf);
      adc_dma_set_reference(&ANALOGIN, (gain_cfg >> 4) & 0xf);
    }

    if (new_rate_cfg != rate_cfg)
    {
      const void * const hw = ANALOGIN.device.hw;
      hri_adc_clear_CTRLA_ENABLE_bit(hw);
      hri_adc_write_CTRLB_PRESCALER_bf(hw,rate_cfg & 0x7);
      hri_adc_write_SAMPCTRL_SAMPLEN_bf(hw,rate_cfg >> 4);
      hri_adc_set_CTRLA_ENABLE_bit(hw);
    }
  }

  if (measurement_queued && !measurement_in_progress)
  {
    last_scheduled = get_time();

    //schedule a read of the power
    uint8_t ctrl_a_data = LTC2992_MASK_LTC_MODE_SNAPSHOT | LTC2992_MASK_LTC_SNAPSHOT_MEASURE_S12;
    i2c_task_t ltc_task = {.addr = LTC2992_ADDR, .write=1, .reg =LTC2992_REG_CTRL_A, .data = ctrl_a_data };
    i2c_enqueue(&ltc_task);

    //schedule a read of the temperatures
    i2c_task_t tmp432_task = {.addr = TMP432_ADDRESS, .write=1, .reg = TMP432_REG_ONESHOT };
    i2c_enqueue(&tmp432_task);

    next_measurement.when = last_scheduled;
    ain_ready = 0;
    adc_dma_enable_channel(&ANALOGIN, 0);
    adc_dma_read(&ANALOGIN, ain, 8*ain_nread_div_8_m1+2); //first is ignored
    adc_dma_start_conversion(&ANALOGIN);
    memset(ain_hist,0,sizeof(ain_hist));


    while(!tmp432_task.done);


    measurement_in_progress = 1;
  }

  if (measurement_in_progress)
  {
    if (ltc.last_read != last_scheduled)
    {
      update_ltc_if_ready(&ltc);
    }

    if (last_temp_read != last_scheduled)
    {
      update_temps_if_ready();
    }

    if (ltc.last_read == last_scheduled && last_temp_read == last_scheduled)
    {

      next_measurement.T_local = (last_local_t[1]-64) << 8 | last_local_t[0];
      next_measurement.T1 = (last_remote1_t[1]-64) << 8 | last_remote1_t[0];
      next_measurement.T2 = (last_remote2_t[1]-64) << 8 | last_remote2_t[0];
      next_measurement.pv = ltc.sense1;
      next_measurement.turbine = ltc.sense2;
      next_measurement.delta_pv = ltc.delta_sense1;
      next_measurement.delta_turbine = ltc.delta_sense2;
      measurement_done = 1;
    }
  }

  if (measurement_done)
  {
    measurement_queued = 0;
    measurement_in_progress = 0;
    measurement_done = 0;
    nmeasurements++;
    memcpy(&measurement,&next_measurement, sizeof(measurement));
  }

  return measurement_in_progress || measurement_queued;

}

void ain_set_source(enum ain_source src)
{
  new_source = src;
}

void ain_set_rate(uint8_t update_rate_cfg)
{
  new_rate_cfg = update_rate_cfg;
}

void ain_set_gain(uint8_t update_gain_cfg)
{
  new_gain_cfg = update_gain_cfg;
}

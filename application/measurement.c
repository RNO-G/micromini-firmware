#include "application/measurement.h"
#include "application/driver_init.h"
#include <string.h>
#include "application/i2cbus.h"
#include "application/time.h"

struct micromini_measurement measurement;
static struct micromini_measurement next_measurement;
int measurement_queued = 0;
static int measurement_in_progress = 0;
static int measurement_done = 0;
static int last_scheduled; 
uint8_t nmeasurements = 0;


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

#define NSKIP 2
static uint16_t read_adc(int chan, int navg)
{
  uint32_t sum = 0; 
  uint8_t buf[2] = {0,0}; 
  int i = 0; 
  if (navg < 1) navg = 1; 
  adc_sync_set_inputs(&ANALOGIN, chan, 0x18,0);
  adc_sync_enable_channel(&ANALOGIN, 0);
  while (i < navg+NSKIP)
  {
    adc_sync_read_channel(&ANALOGIN, 0, (uint8_t*) &buf, 2);
    if (i > NSKIP-1) //skip first NSKIP
      sum+= buf[0] + (buf[1] << 8);
    i++;
  }
  adc_sync_disable_channel(&ANALOGIN, 0); 
  return sum/=navg;
}


static struct ltc2992_ctx ltc = { .addr = LTC2992_ADDR }; 

void measurement_init()
{
	adc_sync_init(&ANALOGIN, ADC, (void *)NULL);

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
    next_measurement.bat_mon = read_adc(BAT_MON, 16);
    next_measurement.ain1 = read_adc(AIN1,16);
    next_measurement.ain12 = read_adc(AIN12,16);
    next_measurement.ain13 = read_adc(AIN13,16);

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

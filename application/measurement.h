#ifndef _MICROMINI_MEASUREMENT_H
#define _MICROMINI_MEASUREMENT_H

#include <stdint.h>



extern struct micromini_measurement
{
  uint32_t when;
  uint16_t pv;
  uint16_t turbine;
  uint16_t delta_pv;
  uint16_t delta_turbine;
  uint16_t T_local;
  uint16_t T1;
  uint16_t T2;
  uint8_t T_local_MSB_raw;
  uint8_t T1_MSB_raw;
  uint8_t T2_MSB_raw;
} measurement;

#define DEFAULT_AIN_SOURCE SOURCE_AIN1
#define AIN_SIZE 2048

extern volatile uint8_t ain[AIN_SIZE]; //extra due to first bad byte
extern volatile int ain_ready;
extern volatile uint8_t ain_nread_div_8_m1; //nread/8 - 1

extern volatile uint16_t ain_hist[256];
extern volatile uint8_t ain_hist_mode_bin;
extern volatile uint8_t ain_hist_max;
extern volatile uint8_t ain_hist_min;

extern volatile uint8_t thresh_rising;
extern volatile uint8_t thresh_falling;
extern volatile uint8_t N_rising;
extern volatile uint8_t N_falling;

void recalculate_crossings();

extern volatile int measurement_queued;
extern volatile uint8_t nmeasurements;

enum ain_source
{
  SOURCE_AIN1,
  SOURCE_AIN12,
  SOURCE_AIN13,
  SOURCE_BATMON,
  SOURCE_TEMP
};

extern volatile enum ain_source ain_source;
extern volatile uint8_t ain_rate_cfg;
extern volatile uint8_t ain_gain_cfg;

void measurement_init();
int measurement_process(); //returns 1 to inhibit standby


#endif

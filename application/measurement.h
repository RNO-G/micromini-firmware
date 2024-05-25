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
  uint16_t bat_mon;
  uint16_t ain1;
  uint16_t ain12;
  uint16_t ain13;
} measurement;


extern int measurement_queued;
extern uint8_t nmeasurements;

void measurement_init();
int measurement_process(); //returns 1 to inhibit standby


#endif

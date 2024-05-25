#ifndef _RNO_G_TIME_H
#define _RNO_G_TIME_H

#include <stdint.h>


void time_process();
int uptime();

// not implemented yet but needed for LORA
int get_time();
void set_time(int t);



#endif

#include "application/time.h"
#include "hal_calendar.h" 
#include "application/driver_init.h" 


static int the_uptime;
void time_process()
{
  the_uptime = _calendar_get_counter(&CALENDAR.device);
}
int uptime()
{
 return  the_uptime;
}

static int offset = 0;

void set_time(int t) { offset = t-uptime(); }
int get_time() { return uptime() + offset; }

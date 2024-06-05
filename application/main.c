#include "config/config.h"
#include "application/driver_init.h"
#include "application/time.h"
#include "application/i2c_client.h"
#include "hal_calendar.h" 
#include "hal_sleep.h" 
#include "application/io.h"
#include "application/printf.h"
#include "application/i2cbus.h"
#include "application/spi_flash.h"
#include "application/measurement.h"
#include "application/gpio.h"


static unsigned nhf = 0;
void HardFault_Handler()
{
  nhf++;
  _reset_mcu();
}

void WDT_Handler()
{
  static unsigned nwdt;
  hri_wdt_clear_INTFLAG_EW_bit(WDT);
  nwdt++;
}

static uint64_t nticks =0;
int last_feed;
static int woke = 0;

static void wakeup()
{
  woke = 1;
}



static int do_sleep(int how_long)
{
  int cmp = _calendar_get_counter(&CALENDAR.device)+how_long; 
  _calendar_register_callback(&CALENDAR.device,wakeup); 
  _calendar_set_comp(&CALENDAR.device,cmp); 
  woke=0;
  return sleep(3);
}


int main(void)
{

  system_init();

  if (ENABLE_WATCHDOG)
  {
    if (ENABLE_WATCHDOG_EW)
    {
      NVIC_EnableIRQ(WDT_IRQn);
      //attempt to set up the EW interrupt.
      hri_wdt_wait_for_sync(WDT);
      hri_wdt_write_EWCTRL_EWOFFSET_bf(WDT, 0xa);
      hri_wdt_write_INTEN_EW_bit(WDT,1);
    }

    wdt_enable(&INTERNAL_WATCHDOG);
  }

  /* Initialize SPI flash */
//  spi_flash_init();

  /** initialize io */
  io_init();

  // read in the config block
//  config_block_t * cfg = config_block();

  //enable the calendar
  calendar_enable(&CALENDAR);

  i2c_bus_init();
  i2c_client_init();
  measurement_init();
  gpio_init();
  int awake_ticks = 0;

  while (1)
  {
    time_process();
    int up = uptime();

    if (ENABLE_WATCHDOG)
    {
      if (up > last_feed)
      {
        wdt_feed(&INTERNAL_WATCHDOG);
        last_feed=up;
      }
    }

    int cant_sleep = measurement_process();
    gpio_process();

    if (!cant_sleep && awake_ticks++ > 50)
    {
      awake_ticks = 0;
      wdt_feed(&INTERNAL_WATCHDOG);
      last_feed=up;
      do_sleep(7);
    }
    delay_ms(10);
    nticks++;
  }
}


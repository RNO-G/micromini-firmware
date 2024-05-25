#include "application/gpio.h"
#include "application/driver_init.h"


uint32_t gpio_outputs = 0;
uint32_t gpio_outputs_set_mask = 1;
uint32_t gpio_outputs_set_vals = 1;

uint32_t gpio_inputs()
{
  return gpio_get_pin_level(NALERT);
}

void gpio_init()
{
 // nothing to do here yet
}


void gpio_process()
{

  if (gpio_outputs_set_mask)
  {
    if (gpio_outputs_set_mask & 1)
    {
      gpio_set_pin_level(AUX_ENABLE, !!(gpio_outputs_set_vals & 1));
    }

    gpio_outputs |= (gpio_outputs_set_mask & gpio_outputs_set_vals);
    gpio_outputs &= ~(gpio_outputs_set_mask & ~gpio_outputs_set_vals);

    gpio_outputs_set_mask = 0;
    gpio_outputs_set_vals = 0;
  }

}

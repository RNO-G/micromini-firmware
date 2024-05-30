#ifndef _RNO_G_GPIO_H
#define _RNO_G_GPIO_H
#include <stdint.h>

extern volatile uint32_t gpio_outputs;
extern volatile uint32_t gpio_outputs_set_mask;
extern volatile uint32_t gpio_outputs_set_vals;

uint32_t gpio_inputs();

void gpio_init();
void gpio_process();

#endif

/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMD21 has 8 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7

#define AIN1 GPIO(GPIO_PORTA, 3)
#define LORA_DIO2 GPIO(GPIO_PORTA, 5)
#define LORA_DIO1 GPIO(GPIO_PORTA, 6)
#define W1_GPIO GPIO(GPIO_PORTA, 7)
#define PA08 GPIO(GPIO_PORTA, 8)
#define PA09 GPIO(GPIO_PORTA, 9)
#define LORA_DIO0 GPIO(GPIO_PORTA, 10)
#define LORA_RESET GPIO(GPIO_PORTA, 11)
#define PA12 GPIO(GPIO_PORTA, 12)
#define SPIFLASH_CS GPIO(GPIO_PORTA, 13)
#define PA14 GPIO(GPIO_PORTA, 14)
#define PA15 GPIO(GPIO_PORTA, 15)
#define PA16 GPIO(GPIO_PORTA, 16)
#define PA17 GPIO(GPIO_PORTA, 17)
#define EXT_GPIO0 GPIO(GPIO_PORTA, 18)
#define EXT_GPIO1 GPIO(GPIO_PORTA, 19)
#define PA20 GPIO(GPIO_PORTA, 20)
#define PA21 GPIO(GPIO_PORTA, 21)
#define PA22 GPIO(GPIO_PORTA, 22)
#define SDCARD_CS GPIO(GPIO_PORTA, 23)
#define PA24 GPIO(GPIO_PORTA, 24)
#define PA25 GPIO(GPIO_PORTA, 25)
#define W1_SLEEP GPIO(GPIO_PORTB, 0)
#define AIN12 GPIO(GPIO_PORTB, 4)
#define AIN13 GPIO(GPIO_PORTB, 5)
#define BAT_MON GPIO(GPIO_PORTB, 6)
#define AUX_ENABLE GPIO(GPIO_PORTB, 10)
#define PB12 GPIO(GPIO_PORTB, 12)
#define PB14 GPIO(GPIO_PORTB, 14)
#define PB15 GPIO(GPIO_PORTB, 15)
#define NALERT GPIO(GPIO_PORTB, 16)
#define PB30 GPIO(GPIO_PORTB, 30)
#define PB31 GPIO(GPIO_PORTB, 31)

#endif // ATMEL_START_PINS_H_INCLUDED

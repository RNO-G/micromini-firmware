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
#define I2C_DEVICE_SDA GPIO(GPIO_PORTA, 8)
#define I2C_CLIENT_SCL GPIO(GPIO_PORTA, 9)
#define SPIFLASH_MISO GPIO(GPIO_PORTA, 12)
#define SPIFLASH_CS GPIO(GPIO_PORTA, 13)
#define SPIFLASH_MOSI GPIO(GPIO_PORTA, 14)
#define SPIFLASH_SCLK GPIO(GPIO_PORTA, 15)
#define EXTUART_TX GPIO(GPIO_PORTA, 16)
#define EXTUART_RX GPIO(GPIO_PORTA, 17)
#define EXT_GPIO0 GPIO(GPIO_PORTA, 18)
#define EXT_GPIO1 GPIO(GPIO_PORTA, 19)
#define SDCARD_MOSI GPIO(GPIO_PORTA, 20)
#define SDCARD_SCLK GPIO(GPIO_PORTA, 21)
#define SDCARD_MISO GPIO(GPIO_PORTA, 22)
#define SDCARD_CS GPIO(GPIO_PORTA, 23)
#define USB_DMINUS GPIO(GPIO_PORTA, 24)
#define USB_DPLUS GPIO(GPIO_PORTA, 25)
#define AUX_ENABLE GPIO(GPIO_PORTB, 3)
#define AIN12 GPIO(GPIO_PORTB, 4)
#define AIN13 GPIO(GPIO_PORTB, 5)
#define BAT_MON GPIO(GPIO_PORTB, 6)
#define NALERT GPIO(GPIO_PORTB, 16)
#define I2C_HOST_SDA GPIO(GPIO_PORTB, 30)
#define I2C_HOST_SCL GPIO(GPIO_PORTB, 31)

#endif // ATMEL_START_PINS_H_INCLUDED

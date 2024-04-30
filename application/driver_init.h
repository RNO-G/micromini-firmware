/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_INCLUDED
#define DRIVER_INIT_INCLUDED

#include "atmel_start_pins.h"


#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_sync.h>
#include <hal_ext_irq.h>

#include <hal_flash.h>

#include <hal_i2c_s_async.h>
#include <hal_usart_async.h>
#include <hal_spi_m_sync.h>
#include <hal_spi_m_sync.h>
#include <hal_spi_m_sync.h>

#include <hal_i2c_m_async.h>

#include <hal_delay.h>

#include <hal_calendar.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>
#include <hal_timer.h>
#include <hpl_tc_base.h>

#include "hal_usb_device.h"

#include <hal_wdt.h>

#define SERCOM0_I2CS_BUFFER_SIZE 16

extern struct adc_sync_descriptor ANALOGIN;

extern struct flash_descriptor FLASH;

extern struct i2c_s_async_descriptor I2C_DEVICE;
extern struct usart_async_descriptor EXT_USART;
extern struct spi_m_sync_descriptor  SPIFLASH;
extern struct spi_m_sync_descriptor  SDCARD;
extern struct spi_m_sync_descriptor  LORA_SPI;

extern struct i2c_m_async_desc I2C_HOST;

extern struct calendar_descriptor CALENDAR;
extern struct timer_descriptor    LORA_TIMER;
extern struct timer_descriptor    SHARED_TIMER;

extern struct wdt_descriptor INTERNAL_WATCHDOG;

void ANALOGIN_PORT_init(void);
void ANALOGIN_CLOCK_init(void);
void ANALOGIN_init(void);

void FLASH_init(void);
void FLASH_CLOCK_init(void);

void I2C_DEVICE_PORT_init(void);
void I2C_DEVICE_CLOCK_init(void);
void I2C_DEVICE_init(void);

void EXT_USART_PORT_init(void);
void EXT_USART_CLOCK_init(void);
void EXT_USART_init(void);

void SPIFLASH_PORT_init(void);
void SPIFLASH_CLOCK_init(void);
void SPIFLASH_init(void);

void SDCARD_PORT_init(void);
void SDCARD_CLOCK_init(void);
void SDCARD_init(void);

void LORA_SPI_PORT_init(void);
void LORA_SPI_CLOCK_init(void);
void LORA_SPI_init(void);

void I2C_HOST_CLOCK_init(void);
void I2C_HOST_init(void);
void I2C_HOST_PORT_init(void);

void delay_driver_init(void);

void CALENDAR_CLOCK_init(void);
void CALENDAR_init(void);

void USB_0_CLOCK_init(void);
void USB_0_init(void);

void INTERNAL_WATCHDOG_CLOCK_init(void);
void INTERNAL_WATCHDOG_init(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#endif // DRIVER_INIT_INCLUDED

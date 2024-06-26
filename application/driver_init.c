/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hal_init.h>
#include <hpl_gclk_base.h>
#include <hpl_pm_base.h>
#include <hpl_adc_base.h>

/*! The buffer size for USART */
#define EXT_USART_BUFFER_SIZE 16

struct adc_dma_descriptor     ANALOGIN;
struct usart_async_descriptor EXT_USART;
struct spi_m_sync_descriptor  SPIFLASH;
struct spi_m_sync_descriptor  SDCARD;
struct timer_descriptor       SHARED_TIMER;

static uint8_t EXT_USART_buffer[EXT_USART_BUFFER_SIZE];

struct flash_descriptor FLASH;

struct i2c_s_async_descriptor I2C_DEVICE;
uint8_t                       SERCOM0_i2c_s_buffer[SERCOM0_I2CS_BUFFER_SIZE];

struct i2c_m_async_desc I2C_HOST;

struct calendar_descriptor CALENDAR;

struct wdt_descriptor INTERNAL_WATCHDOG;



void ANALOGIN_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, ADC);
	_gclk_enable_channel(ADC_GCLK_ID, CONF_GCLK_ADC_SRC);

	adc_dma_init(&ANALOGIN, ADC);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AIN1, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AIN1, PINMUX_PA03B_ADC_AIN1);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AIN12, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AIN12, PINMUX_PB04B_ADC_AIN12);

	// Disable digital pin circuitry
	gpio_set_pin_direction(AIN13, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(AIN13, PINMUX_PB05B_ADC_AIN13);

	// Disable digital pin circuitry
	gpio_set_pin_direction(BAT_MON, GPIO_DIRECTION_OFF);

	gpio_set_pin_function(BAT_MON, PINMUX_PB06B_ADC_AIN14);

}



void FLASH_CLOCK_init(void)
{

	_pm_enable_bus_clock(PM_BUS_APBB, NVMCTRL);
}

void FLASH_init(void)
{
	FLASH_CLOCK_init();
	flash_init(&FLASH, NVMCTRL);
}

void I2C_DEVICE_PORT_init(void)
{

	gpio_set_pin_pull_mode(I2C_DEVICE_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(I2C_DEVICE_SDA, PINMUX_PA08C_SERCOM0_PAD0);

	gpio_set_pin_pull_mode(I2C_CLIENT_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(I2C_CLIENT_SCL, PINMUX_PA09C_SERCOM0_PAD1);
}

void I2C_DEVICE_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM0);
	_gclk_enable_channel(SERCOM0_GCLK_ID_CORE, CONF_GCLK_SERCOM0_CORE_SRC);
	_gclk_enable_channel(SERCOM0_GCLK_ID_SLOW, CONF_GCLK_SERCOM0_SLOW_SRC);
}

void I2C_DEVICE_init(void)
{
	I2C_DEVICE_CLOCK_init();
	i2c_s_async_init(&I2C_DEVICE, SERCOM0, SERCOM0_i2c_s_buffer, SERCOM0_I2CS_BUFFER_SIZE);
	I2C_DEVICE_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void EXT_USART_CLOCK_init()
{

	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM1);
	_gclk_enable_channel(SERCOM1_GCLK_ID_CORE, CONF_GCLK_SERCOM1_CORE_SRC);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void EXT_USART_PORT_init()
{

	gpio_set_pin_function(EXTUART_TX, PINMUX_PA16C_SERCOM1_PAD0);

	gpio_set_pin_function(EXTUART_RX, PINMUX_PA17C_SERCOM1_PAD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void EXT_USART_init(void)
{
	EXT_USART_CLOCK_init();
	usart_async_init(&EXT_USART, SERCOM1, EXT_USART_buffer, EXT_USART_BUFFER_SIZE, (void *)NULL);
	EXT_USART_PORT_init();
}

void SPIFLASH_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(SPIFLASH_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SPIFLASH_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SPIFLASH_MISO, PINMUX_PA12C_SERCOM2_PAD0);

	gpio_set_pin_level(SPIFLASH_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SPIFLASH_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPIFLASH_MOSI, PINMUX_PA14C_SERCOM2_PAD2);

	gpio_set_pin_level(SPIFLASH_SCLK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SPIFLASH_SCLK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SPIFLASH_SCLK, PINMUX_PA15C_SERCOM2_PAD3);
}

void SPIFLASH_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM2);
	_gclk_enable_channel(SERCOM2_GCLK_ID_CORE, CONF_GCLK_SERCOM2_CORE_SRC);
}

void SPIFLASH_init(void)
{
	SPIFLASH_CLOCK_init();
	spi_m_sync_init(&SPIFLASH, SERCOM2);
	SPIFLASH_PORT_init();
}

void SDCARD_PORT_init(void)
{

	// Set pin direction to input
	gpio_set_pin_direction(SDCARD_MISO, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SDCARD_MISO,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDCARD_MISO, PINMUX_PA22C_SERCOM3_PAD0);

	gpio_set_pin_level(SDCARD_MOSI,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SDCARD_MOSI, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SDCARD_MOSI, PINMUX_PA20D_SERCOM3_PAD2);

	gpio_set_pin_level(SDCARD_SCLK,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(SDCARD_SCLK, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(SDCARD_SCLK, PINMUX_PA21D_SERCOM3_PAD3);
}

void SDCARD_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM3);
	_gclk_enable_channel(SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_SRC);
}

void SDCARD_init(void)
{
	SDCARD_CLOCK_init();
	spi_m_sync_init(&SDCARD, SERCOM3);
	SDCARD_PORT_init();
}



void I2C_HOST_PORT_init(void)
{

	gpio_set_pin_pull_mode(I2C_HOST_SDA,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(I2C_HOST_SDA, PINMUX_PB30D_SERCOM5_PAD0);

	gpio_set_pin_pull_mode(I2C_HOST_SCL,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(I2C_HOST_SCL, PINMUX_PB31D_SERCOM5_PAD1);
}

void I2C_HOST_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, SERCOM5);
	_gclk_enable_channel(SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_SRC);
	_gclk_enable_channel(SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_SRC);
}

void I2C_HOST_init(void)
{
	I2C_HOST_CLOCK_init();
	i2c_m_async_init(&I2C_HOST, SERCOM5);
	I2C_HOST_PORT_init();
}

void delay_driver_init(void)
{
	delay_init(SysTick);
}

void CALENDAR_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, RTC);
	_gclk_enable_channel(RTC_GCLK_ID, CONF_GCLK_RTC_SRC);
}

void CALENDAR_init(void)
{
	CALENDAR_CLOCK_init();
	calendar_init(&CALENDAR, RTC);
}


/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void SHARED_TIMER_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBC, TC4);
	_gclk_enable_channel(TC4_GCLK_ID, CONF_GCLK_TC4_SRC);

	timer_init(&SHARED_TIMER, TC4, _tc_get_timer());
}

void USB_0_PORT_init(void)
{

	gpio_set_pin_direction(USB_DMINUS,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(USB_DMINUS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(USB_DMINUS,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(USB_DMINUS,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA24G_USB_DM"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      PINMUX_PA24G_USB_DM);

	gpio_set_pin_direction(USB_DPLUS,
	                       // <y> Pin direction
	                       // <id> pad_direction
	                       // <GPIO_DIRECTION_OFF"> Off
	                       // <GPIO_DIRECTION_IN"> In
	                       // <GPIO_DIRECTION_OUT"> Out
	                       GPIO_DIRECTION_OUT);

	gpio_set_pin_level(USB_DPLUS,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	gpio_set_pin_pull_mode(USB_DPLUS,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(USB_DPLUS,
	                      // <y> Pin function
	                      // <id> pad_function
	                      // <i> Auto : use driver pinmux if signal is imported by driver, else turn off function
	                      // <PINMUX_PA25G_USB_DP"> Auto
	                      // <GPIO_PIN_FUNCTION_OFF"> Off
	                      // <GPIO_PIN_FUNCTION_A"> A
	                      // <GPIO_PIN_FUNCTION_B"> B
	                      // <GPIO_PIN_FUNCTION_C"> C
	                      // <GPIO_PIN_FUNCTION_D"> D
	                      // <GPIO_PIN_FUNCTION_E"> E
	                      // <GPIO_PIN_FUNCTION_F"> F
	                      // <GPIO_PIN_FUNCTION_G"> G
	                      // <GPIO_PIN_FUNCTION_H"> H
	                      PINMUX_PA25G_USB_DP);
}

/* The USB module requires a GCLK_USB of 48 MHz ~ 0.25% clock
 * for low speed and full speed operation. */
#if (CONF_GCLK_USB_FREQUENCY > (48000000 + 48000000 / 400)) || (CONF_GCLK_USB_FREQUENCY < (48000000 - 48000000 / 400))
#warning USB clock should be 48MHz ~ 0.25% clock, check your configuration!
#endif

void USB_0_CLOCK_init(void)
{

	_pm_enable_bus_clock(PM_BUS_APBB, USB);
	_pm_enable_bus_clock(PM_BUS_AHB, USB);
	_gclk_enable_channel(USB_GCLK_ID, CONF_GCLK_USB_SRC);
}

void USB_0_init(void)
{
	USB_0_CLOCK_init();
	usb_d_init();
	USB_0_PORT_init();
}

void INTERNAL_WATCHDOG_CLOCK_init(void)
{
	_pm_enable_bus_clock(PM_BUS_APBA, WDT);
	_gclk_enable_channel(WDT_GCLK_ID, CONF_GCLK_WDT_SRC);
}

void INTERNAL_WATCHDOG_init(void)
{
	INTERNAL_WATCHDOG_CLOCK_init();
	wdt_init(&INTERNAL_WATCHDOG, WDT);
}

void system_init(void)
{
	init_mcu();

	// GPIO on PA05

	// GPIO on PA13

	// Set pin direction to input
	gpio_set_pin_direction(SPIFLASH_CS, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SPIFLASH_CS,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SPIFLASH_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18

	// Set pin direction to input
	gpio_set_pin_direction(EXT_GPIO0, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(EXT_GPIO0,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(EXT_GPIO0, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19

	// Set pin direction to input
	gpio_set_pin_direction(EXT_GPIO1, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(EXT_GPIO1,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(EXT_GPIO1, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA23

	// Set pin direction to input
	gpio_set_pin_direction(SDCARD_CS, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(SDCARD_CS,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(SDCARD_CS, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB10

	gpio_set_pin_level(AUX_ENABLE, true);
	gpio_set_pin_direction(AUX_ENABLE, GPIO_DIRECTION_OUT);


	gpio_set_pin_function(AUX_ENABLE, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB16

	// Set pin direction to input
	gpio_set_pin_direction(NALERT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(NALERT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_OFF);

	gpio_set_pin_function(NALERT, GPIO_PIN_FUNCTION_OFF);

	ANALOGIN_init();

	FLASH_init();

	I2C_DEVICE_init();
	EXT_USART_init();

	SPIFLASH_init();

	SDCARD_init();


	I2C_HOST_init();

	delay_driver_init();

	CALENDAR_init();

	SHARED_TIMER_init();

	USB_0_init();

	INTERNAL_WATCHDOG_init();
}

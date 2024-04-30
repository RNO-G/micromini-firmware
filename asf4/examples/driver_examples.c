/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

/**
 * Example of using ANALOGIN to generate waveform.
 */
void ANALOGIN_example(void)
{
	uint8_t buffer[2];

	adc_sync_enable_channel(&ANALOGIN, 0);

	while (1) {
		adc_sync_read_channel(&ANALOGIN, 0, buffer, 2);
	}
}

static void button_on_PA07_pressed(void)
{
}

/**
 * Example of using EXTERNAL_IRQ_0
 */
void EXTERNAL_IRQ_0_example(void)
{

	ext_irq_register(PIN_PA07, button_on_PA07_pressed);
}

static uint8_t src_data[128];
static uint8_t chk_data[128];
/**
 * Example of using FLASH to read and write Flash main array.
 */
void FLASH_example(void)
{
	uint32_t page_size;
	uint16_t i;

	/* Init source data */
	page_size = flash_get_page_size(&FLASH);

	for (i = 0; i < page_size; i++) {
		src_data[i] = i;
	}

	/* Write data to flash */
	flash_write(&FLASH, 0x3200, src_data, page_size);

	/* Read data from flash */
	flash_read(&FLASH, 0x3200, chk_data, page_size);
}

static struct io_descriptor *io;

static void I2C_DEVICE_rx_complete(const struct i2c_s_async_descriptor *const descr)
{
	uint8_t c;

	io_read(io, &c, 1);
}

void I2C_DEVICE_example(void)
{
	i2c_s_async_get_io_descriptor(&I2C_DEVICE, &io);
	i2c_s_async_register_callback(&I2C_DEVICE, I2C_S_RX_COMPLETE, I2C_DEVICE_rx_complete);
	i2c_s_async_enable(&I2C_DEVICE);
}

/**
 * Example of using EXT_USART to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_EXT_USART[12] = "Hello World!";

static void tx_cb_EXT_USART(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void EXT_USART_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&EXT_USART, USART_ASYNC_TXC_CB, tx_cb_EXT_USART);
	/*usart_async_register_callback(&EXT_USART, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&EXT_USART, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&EXT_USART, &io);
	usart_async_enable(&EXT_USART);

	io_write(io, example_EXT_USART, 12);
}

/**
 * Example of using SPIFLASH to write "Hello World" using the IO abstraction.
 */
static uint8_t example_SPIFLASH[12] = "Hello World!";

void SPIFLASH_example(void)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SPIFLASH, &io);

	spi_m_sync_enable(&SPIFLASH);
	io_write(io, example_SPIFLASH, 12);
}

/**
 * Example of using SDCARD to write "Hello World" using the IO abstraction.
 */
static uint8_t example_SDCARD[12] = "Hello World!";

void SDCARD_example(void)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&SDCARD, &io);

	spi_m_sync_enable(&SDCARD);
	io_write(io, example_SDCARD, 12);
}

/**
 * Example of using LORA_SPI to write "Hello World" using the IO abstraction.
 */
static uint8_t example_LORA_SPI[12] = "Hello World!";

void LORA_SPI_example(void)
{
	struct io_descriptor *io;
	spi_m_sync_get_io_descriptor(&LORA_SPI, &io);

	spi_m_sync_enable(&LORA_SPI);
	io_write(io, example_LORA_SPI, 12);
}

void I2C_HOST_example(void)
{
	struct io_descriptor *I2C_HOST_io;

	i2c_m_sync_get_io_descriptor(&I2C_HOST, &I2C_HOST_io);
	i2c_m_sync_enable(&I2C_HOST);
	i2c_m_sync_set_slaveaddr(&I2C_HOST, 0x12, I2C_M_SEVEN);
	io_write(I2C_HOST_io, (uint8_t *)"Hello World!", 12);
}

void delay_example(void)
{
	delay_ms(5000);
}

/**
 * Example of using CALENDAR.
 */
static struct calendar_alarm alarm;

static void alarm_cb(struct calendar_descriptor *const descr)
{
	/* alarm expired */
}

void CALENDAR_example(void)
{
	struct calendar_date date;
	struct calendar_time time;

	calendar_enable(&CALENDAR);

	date.year  = 2000;
	date.month = 12;
	date.day   = 31;

	time.hour = 12;
	time.min  = 59;
	time.sec  = 59;

	calendar_set_date(&CALENDAR, &date);
	calendar_set_time(&CALENDAR, &time);

	alarm.cal_alarm.datetime.time.sec = 4;
	alarm.cal_alarm.option            = CALENDAR_ALARM_MATCH_SEC;
	alarm.cal_alarm.mode              = REPEAT;

	calendar_set_alarm(&CALENDAR, &alarm, alarm_cb);
}

static struct timer_task LORA_TIMER_task1, LORA_TIMER_task2;

/**
 * Example of using LORA_TIMER.
 */
static void LORA_TIMER_task1_cb(const struct timer_task *const timer_task)
{
}

static void LORA_TIMER_task2_cb(const struct timer_task *const timer_task)
{
}

void LORA_TIMER_example(void)
{
	LORA_TIMER_task1.interval = 100;
	LORA_TIMER_task1.cb       = LORA_TIMER_task1_cb;
	LORA_TIMER_task1.mode     = TIMER_TASK_REPEAT;
	LORA_TIMER_task2.interval = 200;
	LORA_TIMER_task2.cb       = LORA_TIMER_task2_cb;
	LORA_TIMER_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&LORA_TIMER, &LORA_TIMER_task1);
	timer_add_task(&LORA_TIMER, &LORA_TIMER_task2);
	timer_start(&LORA_TIMER);
}

static struct timer_task SHARED_TIMER_task1, SHARED_TIMER_task2;

/**
 * Example of using SHARED_TIMER.
 */
static void SHARED_TIMER_task1_cb(const struct timer_task *const timer_task)
{
}

static void SHARED_TIMER_task2_cb(const struct timer_task *const timer_task)
{
}

void SHARED_TIMER_example(void)
{
	SHARED_TIMER_task1.interval = 100;
	SHARED_TIMER_task1.cb       = SHARED_TIMER_task1_cb;
	SHARED_TIMER_task1.mode     = TIMER_TASK_REPEAT;
	SHARED_TIMER_task2.interval = 200;
	SHARED_TIMER_task2.cb       = SHARED_TIMER_task2_cb;
	SHARED_TIMER_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&SHARED_TIMER, &SHARED_TIMER_task1);
	timer_add_task(&SHARED_TIMER, &SHARED_TIMER_task2);
	timer_start(&SHARED_TIMER);
}

/**
 * Example of using INTERNAL_WATCHDOG.
 */
void INTERNAL_WATCHDOG_example(void)
{
	uint32_t clk_rate;
	uint16_t timeout_period;

	clk_rate       = 1000;
	timeout_period = 4096;
	wdt_set_timeout_period(&INTERNAL_WATCHDOG, clk_rate, timeout_period);
	wdt_enable(&INTERNAL_WATCHDOG);
}

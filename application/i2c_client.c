#include "application/i2c_client.h"
#include "application/driver_init.h"
#include "application/time.h"
#include "application/reg_map.h"
#include "application/gpio.h"
#include "application/measurement.h"


static struct io_descriptor *io;

static uint8_t tx_queue[64];
static uint32_t tx_queued =0;
static uint32_t tx_sent = 0;
static uint32_t tx_completed = 0;
static uint32_t nrx_callback;
static uint32_t nerr_callback;
static uint32_t ntx_done_callback;
static uint32_t ntx_callback;

static void tx_done_callback(const struct i2c_s_async_descriptor * const desc)
{
  ntx_done_callback++;
  tx_completed += i2c_s_async_get_bytes_sent(desc);
}

i2c_s_status_t  status;
static void err_callback(const struct i2c_s_async_descriptor * const desc)
{
  nerr_callback++;
  i2c_s_async_get_status(desc,&status);
}


static uint8_t fail_byte =0xd0;
static int nwr_fail;
static void tx_callback(const struct i2c_s_async_descriptor * const desc)
{

  ntx_callback++;
  //probably asked for a write when we wanted a read, send fail byte;
  if (tx_queued <= tx_sent)
  {
    io_write(io,&fail_byte,1);
  }
  while (tx_queued > tx_sent)
  {
    int nwr = io_write(io, tx_queue + (tx_sent++ % sizeof(tx_queue)), 1);
    if (nwr!=1) nwr_fail++;
  }
}


static void queue_byte(uint8_t x)
{
  //tx queue is full... that should "never" happen, but we will delay I guess
  while (tx_queued - tx_completed >= sizeof(tx_queue)) delay_ms(1);

  tx_queue[(tx_queued++) % sizeof(tx_queue)] = x;
}


static uint16_t nrx[256];

uint32_t nrdfail;
static  void rx_callback(const struct i2c_s_async_descriptor * const desc)
{
  (void) desc;
  nrx_callback++;
  uint8_t c = 0xff;
  uint8_t v = 0;
  uint8_t v2 = 0;
  int nrd = io_read(io,&c,1);
  if (nrd!=1) 
  {
    return;
    nrdfail++;
  }

  nrx[c]++;
  switch(c)
  {
    case MICROMINI_ID:
      queue_byte(0xab);
      break;
    case MICROMINI_MEASURE:
      measurement_queued=1;
      break;
    case MICROMINI_READ_GPIOS:
      queue_byte(gpio_inputs()); break;
    case MICROMINI_WRITE_GPIOS:
      queue_byte(gpio_outputs);
      io_read(io,&v,1);
      io_read(io,&v2,1);
      if (v)
      {
        gpio_outputs_set_mask = v;
        gpio_outputs_set_vals = v2; // will be acted on on gpio_process
      }
      break;
    case MICROMINI_MEASUREMENT_AGE:
      uint32_t age = uptime() - measurement.when;
      if (age > 0xff) age = 0xff;
      queue_byte(age & 0xff);
      break;
    case MICROMINI_NMEASUREMENTS:
      queue_byte(nmeasurements);
      break;
    case MICROMINI_PV_LSB:
      queue_byte(measurement.pv & 0xff); break;
    case MICROMINI_PV_MSB:
      queue_byte(measurement.pv >> 8 ); break;
    case MICROMINI_TURBINE_LSB:
      queue_byte(measurement.turbine & 0xff); break;
    case MICROMINI_TURBINE_MSB:
      queue_byte(measurement.turbine >> 8 ); break;
    case MICROMINI_DELTA_PV_LSB:
      queue_byte(measurement.delta_pv & 0xff); break;
    case MICROMINI_DELTA_PV_MSB:
      queue_byte(measurement.delta_pv >> 8 ); break;
    case MICROMINI_DELTA_TURBINE_LSB:
      queue_byte(measurement.delta_turbine & 0xff); break;
    case MICROMINI_DELTA_TURBINE_MSB:
      queue_byte(measurement.delta_turbine >> 8 ); break;
    case MICROMINI_T_LOCAL_LSB:
      queue_byte(measurement.T_local & 0xff); break;
    case MICROMINI_T_LOCAL_MSB:
      queue_byte(measurement.T_local >> 8 ); break;
    case MICROMINI_T1_LSB:
      queue_byte(measurement.T1 & 0xff); break;
    case MICROMINI_T1_MSB:
      queue_byte(measurement.T1 >> 8 ); break;
    case MICROMINI_T2_LSB:
      queue_byte(measurement.T2 & 0xff); break;
    case MICROMINI_T2_MSB:
      queue_byte(measurement.T2 >> 8 ); break;
    case MICROMINI_BAT_MON_LSB:
      queue_byte(measurement.bat_mon & 0xff); break;
    case MICROMINI_BAT_MON_MSB:
      queue_byte(measurement.bat_mon >> 8 ); break;
    case MICROMINI_AIN1_MON_LSB:
      queue_byte(measurement.ain1 & 0xff); break;
    case MICROMINI_AIN1_MON_MSB:
      queue_byte(measurement.ain1 >> 8 ); break;
    case MICROMINI_AIN12_MON_LSB:
      queue_byte(measurement.ain12 & 0xff); break;
    case MICROMINI_AIN12_MON_MSB:
      queue_byte(measurement.ain12 >> 8 ); break;
    case MICROMINI_AIN13_MON_LSB:
      queue_byte(measurement.ain13 & 0xff); break;
    case MICROMINI_AIN13_MON_MSB:
      queue_byte(measurement.ain13 >> 8 ); break;
    case MICROMINI_WHEN_BYTE_0:
      queue_byte(measurement.when & 0xff); break;
    case MICROMINI_WHEN_BYTE_1:
      queue_byte(measurement.when >> 8); break;
    case MICROMINI_WHEN_BYTE_2:
      queue_byte(measurement.when >> 16); break;
    case MICROMINI_WHEN_BYTE_3:
      queue_byte(measurement.when >> 24); break;
    default:
      queue_byte(0xff);
  }
}

void i2c_client_init()
{
  i2c_s_async_set_addr(&I2C_DEVICE, 0x72);
  i2c_s_async_get_io_descriptor(&I2C_DEVICE,&io);
  i2c_s_async_register_callback(&I2C_DEVICE,I2C_S_RX_COMPLETE, rx_callback);
  i2c_s_async_register_callback(&I2C_DEVICE,I2C_S_ERROR, err_callback);
  i2c_s_async_register_callback(&I2C_DEVICE,I2C_S_TX_PENDING, tx_callback);
  i2c_s_async_register_callback(&I2C_DEVICE,I2C_S_TX_COMPLETE, tx_done_callback);
  i2c_s_async_enable(&I2C_DEVICE);
}





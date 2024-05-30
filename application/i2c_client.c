#include "application/i2c_client.h"
#include "application/driver_init.h"
#include "application/time.h"
#include "application/reg_map.h"
#include "application/gpio.h"
#include "application/measurement.h"


#define ENABLE_TRACE

#ifdef ENABLE_TRACE

#define TRACE_SIZE 128
uint32_t ntrace;

struct trace
{
  char what; //T,R,D,E
  uint8_t byte;
} trace_buffer[TRACE_SIZE];

#define RX_TRACE(c) trace_buffer[ntrace % TRACE_SIZE].what = 'R';   trace_buffer[ntrace++ % TRACE_SIZE].byte = c
#define TX_TRACE(c) trace_buffer[ntrace % TRACE_SIZE].what = 'T';   trace_buffer[ntrace++ % TRACE_SIZE].byte = c
#define DONE_TRACE(c) trace_buffer[ntrace % TRACE_SIZE].what = 'D';   trace_buffer[ntrace++ % TRACE_SIZE].byte = c
#define ERR_TRACE(c) trace_buffer[ntrace % TRACE_SIZE].what = 'E';    trace_buffer[ntrace++ % TRACE_SIZE].byte = c

#else
#define RX_TRACE(c)
#define TX_TRACE(c)
#define DONE_TRACE(c)
#define ERR_TRACE(c)
#endif

static struct io_descriptor *io;

static uint8_t tx_queue[64];
static uint32_t tx_queued =0;
static uint32_t tx_sent = 0;
static uint32_t tx_completed = 0;
static uint32_t nreadbacks = 0;
static uint32_t nrx_callback;
static uint32_t nerr_callback;
static uint32_t ntx_done_callback;
static uint32_t ntx_callback;
static uint8_t ain_offs;
static uint8_t hist_bin;
static uint8_t ain_nread;
static uint8_t last_reg = 0xff;

static enum
{
  READY,
  AIN_READING,
  AWAITING_AIN_OFFSET,
  AWAITING_AIN_NMEAS,
  AWAITING_AIN_NREAD,
  AWAITING_AIN_SOURCE,
  AWAITING_AIN_RATE,
  AWAITING_AIN_GAIN,
  AWAITING_HIST_BIN,
  AWAITING_RISING_THRESH,
  AWAITING_FALLING_THRESH,
  AWAITING_GPIO_MASK,
  AWAITING_GPIO_VAL
} rx_mode;


static void tx_done_callback(const struct i2c_s_async_descriptor * const desc)
{
  (void) desc;
  DONE_TRACE(ntx_done_callback & 0xff);
  ntx_done_callback++;
  tx_completed++; //... hopefully
  rx_mode= READY;
}

i2c_s_status_t  status;
static void err_callback(const struct i2c_s_async_descriptor * const desc)
{
  ERR_TRACE(nerr_callback & 0xff);
  nerr_callback++;
  i2c_s_async_get_status(desc,&status);
}


static uint8_t fail_byte =0xd0;
static int nwr_fail;


static volatile uint8_t * handle_readbacks(int * N)
{
  // no matter what, if we ask for a read, get into ready state...

  switch (rx_mode)
  {
    case AIN_READING:
      *N = ain_nread;
      return &ain[1 + ain_offs];
    case AWAITING_AIN_OFFSET:
      rx_mode = READY;
      return &ain_offs;
    case AWAITING_AIN_NMEAS:
      rx_mode = READY;
      return &ain_nread_div_8_m1;
    case AWAITING_HIST_BIN:
      rx_mode = READY;
      return &hist_bin;
    case AWAITING_RISING_THRESH:
      rx_mode = READY;
      return &thresh_rising;
    case AWAITING_FALLING_THRESH:
      rx_mode = READY;
      return &thresh_falling;
    case AWAITING_AIN_NREAD:
      rx_mode = READY;
      return &ain_nread;
    case AWAITING_GPIO_MASK:
      rx_mode = READY;
      return &gpio_outputs;
    default:
      rx_mode = READY;
      return &fail_byte;
  }
}

static void tx_callback(const struct i2c_s_async_descriptor * const desc)
{


  ntx_callback++;

  // we are in a special mode (readback or ain_reading)
  if (tx_queued == tx_sent)
  {
    int N = 1;
    uint8_t * tx = handle_readbacks(&N);
    TX_TRACE(*tx);
    if (1!=io_write(io,tx,1)) nwr_fail++;
    nreadbacks++;
  }
  while (tx_queued != tx_sent)
  {
    uint8_t *tx = tx_queue + (tx_sent++ % sizeof(tx_queue));
    TX_TRACE(*tx);
    int nwr = io_write(io,tx , 1);
    if (nwr!=1) nwr_fail++;
  }
}


static void queue_byte(uint8_t x)
{
  //tx queue is full... that should "never" happen, but we will delay I guess
  while (tx_queued - tx_completed  + nreadbacks >= sizeof(tx_queue)) delay_ms(1);

  tx_queue[(tx_queued++) % sizeof(tx_queue)] = x;
}


static uint16_t nrx[256];

uint32_t nrdfail;

static  void rx_callback(const struct i2c_s_async_descriptor * const desc)
{
  (void) desc;
  nrx_callback++;
  uint8_t c = 0xff;
  int nrd = io_read(io,&c,1);
  if (nrd!=1)
  {
    return;
    nrdfail++;
  }

  RX_TRACE(c);
  switch (rx_mode)
  {
    case AWAITING_AIN_OFFSET:
      ain_offs = c;
      rx_mode = READY;
      return;
    case AWAITING_AIN_NMEAS:
      ain_nread_div_8_m1 = c;
      rx_mode = READY;
      return;
    case AWAITING_AIN_NREAD:
      ain_nread = c;
      rx_mode = READY;
      return;
    case AWAITING_GPIO_MASK:
      gpio_outputs_set_mask = c;
      rx_mode = AWAITING_GPIO_VAL;
      return;
    case AWAITING_GPIO_VAL:
      gpio_outputs_set_vals = c;
      rx_mode = READY;
      return;
    case AWAITING_AIN_SOURCE:
      ain_set_source(c & 0xf);
      rx_mode = READY;
      break;
    case AWAITING_AIN_RATE:
      ain_set_rate(c);
      rx_mode = READY;
      break;
    case AWAITING_AIN_GAIN:
      ain_set_gain(c);
      rx_mode = READY;
      break;
    case AWAITING_HIST_BIN:
      hist_bin = c;
      rx_mode = READY;
      return;
    case AWAITING_RISING_THRESH:
      thresh_rising = c;
      recalculate_crossings();
      rx_mode = READY;
      return;
    case AWAITING_FALLING_THRESH:
      thresh_falling = c;
      recalculate_crossings();
      rx_mode = READY;
      return;
    case READY:
    default:
      rx_mode = READY;
      nrx[c]++;
  }

  last_reg = c;


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
      rx_mode = AWAITING_GPIO_MASK;
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
    case MICROMINI_AIN:
      rx_mode = AIN_READING;
      break;
    case MICROMINI_AIN_READY:
      queue_byte(ain_ready); break;
    case MICROMINI_AIN_OFFSET:
      rx_mode = AWAITING_AIN_OFFSET; break;
    case MICROMINI_AIN_NMEAS:
      rx_mode = AWAITING_AIN_NMEAS; break;
    case MICROMINI_AIN_NREAD:
      rx_mode = AWAITING_AIN_NREAD; break;
    case MICROMINI_AIN_HIST_BIN:
      rx_mode = AWAITING_HIST_BIN; break;
    case MICROMINI_AIN_RISING_THRESH:
      rx_mode = AWAITING_RISING_THRESH; break;
    case MICROMINI_AIN_FALLING_THRESH:
      rx_mode = AWAITING_FALLING_THRESH; break;
    case MICROMINI_AIN_SOURCE:
      rx_mode = AWAITING_AIN_SOURCE; break;
    case MICROMINI_AIN_RATE:
      rx_mode = AWAITING_AIN_RATE; break;
    case MICROMINI_AIN_GAIN:
      rx_mode = AWAITING_AIN_GAIN; break;
    case MICROMINI_WHEN_BYTE_0:
      queue_byte(measurement.when & 0xff); break;
    case MICROMINI_WHEN_BYTE_1:
      queue_byte(measurement.when >> 8); break;
    case MICROMINI_WHEN_BYTE_2:
      queue_byte(measurement.when >> 16); break;
    case MICROMINI_WHEN_BYTE_3:
      queue_byte(measurement.when >> 24); break;
    case MICROMINI_AIN_HIST:
      queue_byte(ain_hist[hist_bin] > 255 ? 255 : ain_hist[hist_bin]); break;
    case MICROMINI_AIN_HIST_LSB:
      queue_byte(ain_hist[hist_bin] & 0xff); break;
    case MICROMINI_AIN_HIST_MSB:
      queue_byte(ain_hist[hist_bin] >> 8 ); break;
    case MICROMINI_AIN_HIST_MODE_BIN:
      queue_byte(ain_hist_mode_bin); break;
    case MICROMINI_AIN_HIST_HIGHEST_VAL:
      queue_byte(ain_hist_max); break;
    case MICROMINI_AIN_HIST_LOWEST_VAL:
      queue_byte(ain_hist_min); break;
    case MICROMINI_AIN_NUM_RISING_CROSSINGS:
      queue_byte(N_rising); break;
    case MICROMINI_AIN_NUM_FALLING_CROSSINGS:
      queue_byte(N_falling); break;
    case MICROMINI_RESET_REASON:
      queue_byte(_get_reset_reason()); break;
    case MICROMINI_RESET:
      _reset_mcu();break; //lol
    default:
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





#include "../application/reg_map.h"
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <linux/i2c.h> 
#include <sys/ioctl.h> 
#include <errno.h>
#include <ctype.h>
#include <stdlib.h> 
#include <unistd.h> 
#include <fcntl.h> 
#include <string.h> 
#include <stdint.h>

#define MICROMINI_ADDR 0x72
#define MICROMINI_BUS "/dev/i2c-3"


int fd;

struct subcommand
{
  const char * name;
  const char * arg_name; // NULL if no arg
  const char * doc; 
  enum
  {
    SUBCOMMAND_READ_REG,
    SUBCOMMAND_INTERPRET_REG,
    SUBCOMMAND_WRITE_REG,
    SUBCOMMAND_TOUCH_REG,
    SUBCOMMAND_FN
  } type ;
  union
  {
    struct { uint8_t reg; int hex; } read_opts;
    struct { uint8_t reg; uint8_t mask; } write_opts;
    struct { uint8_t reg; } touch_opts;
    struct { int (* fn) (uint8_t *arg); } fn_opts;
    struct { const char * (* fn) (uint8_t val); uint8_t reg; } interpret_opts;
  };
};



static int touch_reg(const struct subcommand * sub)
{
  uint8_t reg = sub->touch_opts.reg;
  struct i2c_msg msg =
  {
    .addr = MICROMINI_ADDR,
    .len = 1,
    .buf = &reg
  };
  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = &msg, .nmsgs = 1};

  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  return 0;
}

static int write_reg(uint8_t arg, const struct subcommand * sub)
{
  uint8_t buf[2] = {sub->write_opts.reg, arg};
  struct i2c_msg msg =
  {
    .addr = MICROMINI_ADDR,
    .len = 2,
    .buf = buf
  };

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs= &msg, .nmsgs = 1};

  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }
  return 0;
}


static int interpret_reg(const struct subcommand * sub)
{

  uint8_t reg = sub->interpret_opts.reg;

  uint8_t val;
  struct i2c_msg txn[2] =
  {
    { .addr = MICROMINI_ADDR, .len = 1, .buf = &reg},
    { .addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &val}
  };

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = txn, .nmsgs = 2 };

  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  printf("%s\n",sub->interpret_opts.fn(val));

  return 0;
}
static int read_reg(const struct subcommand * sub)
{

  uint8_t reg = sub->read_opts.reg;

  uint8_t val;
  struct i2c_msg txn[2] =
  {
    { .addr = MICROMINI_ADDR, .len = 1, .buf = &reg},
    { .addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &val}
  };

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = txn, .nmsgs = 2 };

  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno;
  }

  if (sub->read_opts.hex)
    printf("%s: 0x%x\n", sub->name, val);
  else
    printf("%s: %d\n", sub->name, val);

  return 0;
}

const char * sixteenths[16] =
{ "0",
  "0625",
  "125",
  "1875",
  "25",
  "3125",
  "375",
  "4375",
  "5",
  "5625",
  "625",
  "6875",
  "75",
  "8125",
  "875",
  "9375",
};
static int read_ain_hist(uint8_t *arg)
{
  (void) arg;
  //start by getting max and min vals 

  uint8_t minv = MICROMINI_AIN_HIST_LOWEST_VAL;
  uint8_t maxv = MICROMINI_AIN_HIST_HIGHEST_VAL;

  struct i2c_msg txn_init[] = {
    {.addr = MICROMINI_ADDR, .len = 1, .buf = &minv},
    {.addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &minv},
    {.addr = MICROMINI_ADDR, .len = 1, .buf = &maxv},
    {.addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &maxv},
  };

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = txn_init, .nmsgs =sizeof(txn_init)/sizeof(*txn_init)};
  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 ) return -errno;

  printf("AIN_HIST = ");
  if (minv == 1)
  {
    printf("[0:0]");
  }
  else if (minv > 1)
  {
    printf("[0-%hhu:0]", minv-1);
  }

  for (uint8_t i = minv; i <= maxv; i++) 
  {
    uint8_t set_bin[2] = {MICROMINI_AIN_HIST_BIN,i};
    uint8_t msb = MICROMINI_AIN_HIST_MSB;
    uint8_t lsb = MICROMINI_AIN_HIST_LSB;
    struct i2c_msg txn[] = {
      { .addr = MICROMINI_ADDR, .len = 2, .buf = set_bin },
      { .addr = MICROMINI_ADDR, .len = 1, .buf = &msb },
      { .addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &msb },
      { .addr = MICROMINI_ADDR, .len = 1, .buf = &lsb },
      { .addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len = 1, .buf = &lsb },
    };
    i2c_data.msgs = txn;
    i2c_data.nmsgs = 5;

    if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 ) return -errno;
    uint16_t val = lsb | (msb << 8);
    printf("[%hhu:%hu]", i,val);
  }
  if (maxv == 254)
  {
    printf("[255:0]");
  }
  else if (maxv < 254)
  {
    printf("[%hhu-255:0]", maxv+1);
  }

  printf("\n");


  return 0;

}


static int read_ain(uint8_t * arg)
{
  (void) arg; 
  // figure out how many we need to read, and the old window size

  uint8_t nmeas = MICROMINI_AIN_NMEAS;
  uint8_t old_nread[] = {MICROMINI_AIN_NREAD,0};
  uint8_t new_nread[] = {MICROMINI_AIN_NREAD,7};//minus 1
  struct i2c_msg txn_init[] = {
    {.addr = MICROMINI_ADDR, .len =1, .buf = &nmeas},
    {.addr = MICROMINI_ADDR,.flags=I2C_M_RD, .len =1, .buf = &nmeas},
    {.addr = MICROMINI_ADDR, .len =1, .buf = old_nread},
    {.addr = MICROMINI_ADDR,.flags=I2C_M_RD, .len =1, .buf = old_nread+1},
    {.addr = MICROMINI_ADDR, .len =2, .buf = new_nread }, // set nread to 8
  };

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = txn_init, .nmsgs =sizeof(txn_init)/sizeof(*txn_init)};
  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 ) return -errno; 

  int N = 8*(nmeas+1);

  // we will read out 8 at a time for now

  int i = 0;
  uint8_t set_offset[2] = { MICROMINI_AIN_OFFSET, 0};
  uint8_t rd[8];
  uint8_t ain = MICROMINI_AIN;

  printf("AIN = [ ");
  while (i < N)
  {
    set_offset[1] = i;
    struct i2c_msg txn[] = {
      { .addr = MICROMINI_ADDR, .len =2 , .buf = set_offset }, 
      { .addr = MICROMINI_ADDR, .len =1 , .buf = &ain },
      { .addr = MICROMINI_ADDR, .len =8 , .flags = I2C_M_RD, .buf = rd}
    };
    i2c_data.msgs = txn;
    i2c_data.nmsgs = 3;

    if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 ) return -errno; 
    for (int j = 0; j < 8; j++) printf(" %hhu ", rd[j]);
    i+=8;
  }

  //restore old nread
  struct i2c_msg txn_final = { .addr = MICROMINI_ADDR, .len =2, .buf = old_nread };
  i2c_data.msgs = &txn_final;
  i2c_data.nmsgs = 1;
  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 ) return -errno;
  printf("]\n");
  return 0;
}

static int read_measurements(uint8_t *arg)
{

  (void) arg;

  uint8_t pv_lsb =   MICROMINI_PV_LSB;
  uint8_t pv_msb =  MICROMINI_PV_MSB;
  uint8_t turb_lsb =  MICROMINI_TURBINE_LSB;
  uint8_t turb_msb =   MICROMINI_TURBINE_MSB;
  uint8_t delta_pv_lsb =  MICROMINI_DELTA_PV_LSB;
  uint8_t delta_pv_msb =   MICROMINI_DELTA_PV_MSB;
  uint8_t delta_turb_lsb =  MICROMINI_DELTA_TURBINE_LSB;
  uint8_t delta_turb_msb =  MICROMINI_DELTA_TURBINE_MSB;
  uint8_t when0 =  MICROMINI_WHEN_BYTE_0;
  uint8_t when1 = MICROMINI_WHEN_BYTE_1;
  uint8_t when2 =  MICROMINI_WHEN_BYTE_2;
  uint8_t when3 = MICROMINI_WHEN_BYTE_3;
  uint8_t tlocal_lsb =  MICROMINI_T_LOCAL_LSB;
  int8_t tlocal_msb =  MICROMINI_T_LOCAL_MSB;
  uint8_t t1_lsb =  MICROMINI_T1_LSB;
  int8_t t1_msb =  MICROMINI_T1_MSB;
  uint8_t t2_lsb =  MICROMINI_T2_LSB;
  int8_t t2_msb = MICROMINI_T2_MSB;

#define QUEUE(x) \
    { .addr = MICROMINI_ADDR, .flags = 0, .len =1, .buf = (uint8_t*) &x},\
    { .addr = MICROMINI_ADDR, .flags = I2C_M_RD, .len =1, .buf = (uint8_t*) &x},

  struct i2c_msg txn[] = {
    QUEUE(pv_lsb)
    QUEUE(pv_msb)
    QUEUE(turb_msb)
    QUEUE(turb_lsb)
    QUEUE(delta_pv_lsb)
    QUEUE(delta_pv_msb)
    QUEUE(delta_turb_msb)
    QUEUE(delta_turb_lsb)
    QUEUE(when0)
    QUEUE(when1)
    QUEUE(when2)
    QUEUE(when3)
    QUEUE(tlocal_lsb)
    QUEUE(tlocal_msb)
    QUEUE(t1_lsb)
    QUEUE(t1_msb)
    QUEUE(t2_lsb)
    QUEUE(t2_msb)
  };
#undef QUEUE

  struct i2c_rdwr_ioctl_data i2c_data = {.msgs = txn, .nmsgs =sizeof(txn)/sizeof(*txn)};
  if (ioctl(fd, I2C_RDWR, &i2c_data) < 0 )
  {
    return -errno; 
  }


  uint32_t uptime = when0 | (when1 << 8) | (when2 <<16) | (when3 << 24);
  printf("Measurement at uptime = %u\n", uptime);
  printf("\t T_local = %d.%s", tlocal_msb, sixteenths[tlocal_lsb>>4]);
  printf("\t T1 = %d.%s", t1_msb, sixteenths[t1_lsb>>4]);
  printf("\t T2 = %d.%s\n", t2_msb, sixteenths[t2_lsb>>4]);
  int turb_v = turb_lsb  | (turb_msb << 8);
  int pv_v = pv_lsb | ( pv_msb << 8);
  int delta_turb_v = delta_turb_lsb  | ( delta_turb_msb << 8);
  int delta_pv_v = delta_pv_lsb  | ( delta_pv_msb << 8);
  printf("\t PV:  %0.3f V, %0.3f A\n", pv_v * 0.025, delta_pv_v * 0.004166666666666666);
  printf("\t TURBINE:  %0.3f V, %0.3f A\n", turb_v * 0.025, delta_turb_v * 0.00416666666);
  return 0;
}

const char * interpret_ain_nmeas(uint8_t val)
{
  static char buf[8];
  int ival = (val+1)*8;
  sprintf(buf,"%u",ival);
  return buf;
}



struct subcommand subcommands[] =
{
  {.name = "reset", .type = SUBCOMMAND_TOUCH_REG, .touch_opts = {.reg = MICROMINI_RESET }, .doc = "Reset micromini"},
  {.name = "measure", .type = SUBCOMMAND_TOUCH_REG, .touch_opts = {.reg = MICROMINI_MEASURE}, .doc = "Initiate a measurement" },
  {.name = "id", .type = SUBCOMMAND_READ_REG,  .read_opts = {. reg = MICROMINI_ID, .hex = 1}, .doc = "Retrieve ID, should return 0xab" },
  {.name = "get-num-sensor-measurements", .type = SUBCOMMAND_READ_REG, .read_opts = { .reg = MICROMINI_NMEASUREMENTS }, .doc= "Returns the number of measurements (wrapping uint8)"},
  {.name = "get-sensor-measurent-age", .type = SUBCOMMAND_READ_REG, .read_opts = { .reg = MICROMINI_MEASUREMENT_AGE }, .doc ="Returns the age of the measurement (saturating uint8)"},
  {.name = "read-sensor-measurement", .type = SUBCOMMAND_FN, .fn_opts = {.fn= read_measurements }, .doc= "Retrieve all sensor measurements"},
  {.name = "get-ain-ready", .type = SUBCOMMAND_READ_REG, .read_opts = { .reg = MICROMINI_AIN_READY }, .doc = "Is AIN ready?"},
  {.name = "get-ain-nrising", .type = SUBCOMMAND_READ_REG, .read_opts = {.reg = MICROMINI_AIN_NUM_RISING_CROSSINGS}, .doc = "Get number of rising threshold crossings"},
  {.name = "get-ain-nfalling", .type = SUBCOMMAND_READ_REG, .read_opts = {.reg = MICROMINI_AIN_NUM_FALLING_CROSSINGS}, .doc = "Get number of falling threshold crossings"},
  {.name = "set-ain-rising-threshold", .type = SUBCOMMAND_WRITE_REG, .arg_name = "rising-threshold", .write_opts = {.reg = MICROMINI_AIN_RISING_THRESH }, .doc = "Set rising threshold"},
  {.name = "set-ain-falling-threshold", .type = SUBCOMMAND_WRITE_REG, .arg_name = "falling-threshold",.write_opts = {.reg = MICROMINI_AIN_FALLING_THRESH },.doc = "Set falling threshold"},
  {.name = "get-ain-rising-threshold", .type = SUBCOMMAND_READ_REG,  .read_opts = {.reg = MICROMINI_AIN_RISING_THRESH }, .doc = "Get rising threshold"},
  {.name = "get-ain-falling-threshold", .type = SUBCOMMAND_READ_REG, .read_opts = {.reg = MICROMINI_AIN_FALLING_THRESH },.doc = "Get falling threshold"},
  {.name = "get-ain-num-samples", .type = SUBCOMMAND_INTERPRET_REG, .interpret_opts = {.reg = MICROMINI_AIN_NMEAS, .fn = interpret_ain_nmeas },.doc = "Retrieve the number of samples to measure"},
  {.name = "set-ain-num-samples", .arg_name = "nsamples-div8-m1", .type = SUBCOMMAND_WRITE_REG, .write_opts = {.reg = MICROMINI_AIN_NMEAS },.doc = "Set the number of samples to measure, divided by 8 and minus 1"},
  {.name = "get-ain-highest-val", .read_opts = {.reg = MICROMINI_AIN_HIST_HIGHEST_VAL},.doc = "Get the maximum value reached in ain"},
  {.name = "get-ain-lowest-val", .read_opts = {.reg = MICROMINI_AIN_HIST_LOWEST_VAL},.doc = "Get the minimum value reached in ain"},
  {.name = "get-ain-mode", .read_opts = {.reg = MICROMINI_AIN_HIST_MODE_BIN},.doc = "Get the most common value in ain"},
  {.name = "get-ain", .type = SUBCOMMAND_FN, .fn_opts = {.fn = read_ain},.doc = "Read out ain"},
  {.name = "get-ain-hist", .type = SUBCOMMAND_FN, .fn_opts = {.fn = read_ain_hist},.doc = "Read out ain hist"},
  {.name = "get-ain-source", .read_opts = {.reg = MICROMINI_AIN_SOURCE},.doc = "Get AIN source index (0 = AIN1, 1 = AIN2, 2 = AIN3, 3 = BAT_MON)"},
  {.name = "set-ain-source", .type = SUBCOMMAND_WRITE_REG, .arg_name = "source-index", .write_opts = {.reg = MICROMINI_AIN_SOURCE},.doc = "Set AIN source index (0 = AIN1, 1 = AIN2, 2 = AIN3, 3 = BAT_MON)"},
  {.name = "get-ain-rate", .read_opts = {.reg = MICROMINI_AIN_RATE},.doc = "Get AIN rate configuration (bits 0-2: ADC prescaler, bits 3-7 SAMPLELEN)"},
  {.name = "set-ain-rate", .type = SUBCOMMAND_WRITE_REG, .arg_name = "rate-cfg", .write_opts = {.reg = MICROMINI_AIN_RATE},.doc = "Set AIN rate configuration (bits 0-2: ADC prescaler, bits 3-7 SAMPLELEN)"},
};


int main(int nargs, char ** args)
{
  if (nargs > 1 && nargs < 4)
  {
    //used for completion! 
    if (!strcmp(args[1],"--shortlist"))
    {

      for (int isub = 0; isub < sizeof(subcommands) / sizeof (*subcommands); isub++)
      {
        printf("%s%s", isub == 0 ?  "" : " ",  subcommands[isub].name);
      }
      printf("\n");
      return 0;
    }
    for (int isub = 0; isub < sizeof(subcommands) / sizeof (*subcommands); isub++)
    {
      if (!strcmp(args[1], subcommands[isub].name)) //match
      {
        //check number of args
        if ( (subcommands[isub].arg_name && nargs == 2) || (!subcommands[isub].arg_name && nargs == 3))
        {
          fprintf(stderr,"subcommand %s has wrong number of args (%d)\n", subcommands[isub].name, nargs-2);
          fprintf(stderr,"Usage: micromini-tool %s %s\n", subcommands[isub].name, subcommands[isub].arg_name ?: "");
          fprintf(stderr,"\t%s\n", subcommands[isub].doc);
          return 1;
        }

        uint8_t arg = 0;
        //check if arg is valid)
        if (nargs ==3)
        {
          char * endptr = "x";
          uint32_t argraw = strtol(args[2],&endptr,0);
          if (*endptr || (argraw & (~0xff)))
          {
            fprintf(stderr,"Invalid argument to %s of %u, expect 0-255 or 0x0-0xff\n", subcommands[isub].name, argraw);
            return 1;
          }
          arg = argraw;
        }

        // open i2c bus
        fd = open(MICROMINI_BUS, O_RDWR);
        if (!fd)
        {
          fprintf(stderr,"Could not open " MICROMINI_BUS "\n");
          return 1;
        }

        if (ioctl(fd, I2C_SLAVE, MICROMINI_ADDR))
        {
          fprintf(stderr,"Could not change address to 0x%x\n", MICROMINI_ADDR);
          return 1;
        }


        switch(subcommands[isub].type)
        {
          case SUBCOMMAND_READ_REG:
            return read_reg(&subcommands[isub]);
          case SUBCOMMAND_TOUCH_REG:
            return touch_reg(&subcommands[isub]);
          case SUBCOMMAND_WRITE_REG:
            return  write_reg(arg, &subcommands[isub]);
          case SUBCOMMAND_INTERPRET_REG:
            return  interpret_reg(&subcommands[isub]);
          case SUBCOMMAND_FN:
            return subcommands[isub].fn_opts.fn( nargs == 3 ? &arg : 0);
          default:
            fprintf(stderr,"Internal error\n");
            return 1;
        }
      }
    }
  }


  fprintf(stderr,"Usage: \n");
  for (int isub = 0; isub < sizeof(subcommands) / sizeof (*subcommands); isub++)
  {
     fprintf(stderr,"\tmicromini-tool %s %s\n", subcommands[isub].name, subcommands[isub].arg_name?:"");
     fprintf(stderr,"\t\t%s\n", subcommands[isub].doc);
  }
}


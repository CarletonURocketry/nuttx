#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <debug.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/nau7802.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

// The NAU7802 supports two modes, 0-100kHz and 0-400kHz
#ifndef CONFIG_SENSORS_NAU7802_I2C_FREQUENCY
    #define CONFIG_SENSORS_NAU7802_I2C_FREQUENCY 400000// is this 100kHz? we'll find out I guess 
#endif

#ifndef CONFIG_SENSORS_NAU7802_THREAD_STACKSIZE
    #define CONFIG_SENSORS_NAU7802_THREAD_STACKSIZE 10000 // ACTUAL TODO THIS IS THE FIRST NUMBER I CAN THINK OF
#endif

#define REG_PU_CTRL 0x00
#define REG_CTRL_1 0x01
#define REG_CTRL_2 0x02
#define REG_OCAL1_B2 0x03
#define REG_OCAL1_B1 0x04
#define REG_OCAL1_B0 0x05
#define REG_GCAL1_B3 0x06
#define REG_GCAL1_B2 0x07
#define REG_GCAL1_B1 0x08
#define REG_GCAL1_B0 0x09
#define REG_OCAL2_B2 0x0A
#define REG_OCAL2_B1 0x0B
#define REG_OCAL2_B0 0x0C
#define REG_GCAL2_B3 0x0D
#define REG_GCAL2_B2 0x0E
#define REG_GCAL2_B1 0x0F
#define REG_GCAL2_B0 0x10
#define REG_I2C_CONTROL 0x11
#define REG_ADCO_B2 0x12 // data bit 23 to 16
#define REG_ADCO_B1 0x13 // data bit 15 to 8
#define REG_ADCO_B0 0x14 // data bit 7 to 0
#define REG_OTP_B1 0x15
#define REG_OTP_B0 0x16

typedef struct {
    struct sensor_lowerhalf_s lower; 
    FAR struct i2c_master_s *i2c;
    uint8_t addr;
    sem_t run;
    mutex_t devlock;
    bool enabled;
    bool interrupts;
    uint32_t odr; // output data rate
} nau7802_dev_s;

// ODR = Ouput Data Rate
enum nau7802_ord_e {
    ODR_10HZ = 0b000,
    ODR_20HZ = 0b001,
    ODR_40HZ = 0b010,
    ODR_80HZ = 0b011,
    ODR_320HZ = 0b111
};

// ODR to time delay in micro-seconds
static const uint32_t ODR_TO_INTERVAL[] = {
    [ODR_10HZ] = 100000,
    [ODR_20HZ] = 50000,
    [ODR_40HZ] = 25000,
    [ODR_80HZ] = 12500,
    [ODR_320HZ] = 3125 
};

static int nau7802_activate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, bool enable);
// static int nau7802_set_interval(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR uint32_t *period_us);
// static int nau7802_selftest(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, unsigned long arg);
static int nau7802_get_info(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR struct sensor_device_info_s *info);
// static int nau7802_control(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, int cmd,unsigned long arg);

static int nau7802_read_reg(FAR nau7802_dev_s *dev, uint8_t addr, void *buf, uint8_t nbytes) {
    struct i2c_msg_s readcmd[2] = {
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTOP,
          .buffer = &addr,
          .length = sizeof(addr),
      },
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_READ,
          .buffer = buf,
          .length = nbytes,
      },

    };

    return I2C_TRANSFER(dev->i2c, readcmd, 2);
}

static int nau7802_write_reg(FAR nau7802_dev_s *dev, uint8_t addr, void *buf, uint8_t nbytes) {
  struct i2c_msg_s writecmd[2] = {
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTOP,
          .buffer = &addr,
          .length = sizeof(addr),
      },
      {
          .frequency = CONFIG_SENSORS_NAU7802_I2C_FREQUENCY,
          .addr = dev->addr,
          .flags = I2C_M_NOSTART,
          .buffer = buf,
          .length = nbytes,
      },
  };

  return I2C_TRANSFER(dev->i2c, writecmd, 2);
}

static int nau7802_read_data(FAR nau7802_dev_s *dev, FAR struct sensor_force *data){
    snerr("Reading data\n");
    int8_t raw_data[3];
    int err = 0;
    err = nau7802_read_reg(dev, REG_ADCO_B2, &raw_data, sizeof(raw_data));
    if(err < 0){
        return err;
    }

    data->timestamp = sensor_get_timestamp();
    data->event = 0;
    data->force = (raw_data[0] << 16) + (raw_data[1] << 8) + raw_data[2]; // this better work

    return err;
}

static int nau7802_set_bit(FAR nau7802_dev_s *dev, uint8_t addr, uint8_t bit, bool val){
    int err = 0;
    uint8_t reg_val;

    err = nau7802_read_reg(dev, addr, &reg_val, sizeof(reg_val));
    if(err < 0){
        return err;
    }

    if(val){
        reg_val = reg_val | (1 << bit); // set the bit
    }
    else{
        reg_val = reg_val & ~(1 << bit); // clear the bit
    }

    return nau7802_write_reg(dev, addr, &reg_val, sizeof(reg_val));
}

static int nau7802_data_available(FAR nau7802_dev_s *dev, bool *data_ready){
    uint8_t reg_data;
    int err = 0;
    err = nau7802_read_reg(dev, REG_PU_CTRL, &reg_data, 1);
    if(err < 0){
        return err;
    }

    *data_ready = ((reg_data >> 5) & 1) == 1; // if the 5th bits is 1 then the data is ready

    return err;
}

static int nau7802_push_data(FAR nau7802_dev_s *dev){
    int err = 0;
    struct sensor_force data;

    err = nxmutex_lock(&dev->devlock);
    if(err < 0){
        return err;
    }

    if(!dev->enabled){
        err = -EAGAIN; // why is it EAGAIN and not like failure?
        goto unlock_ret;
    }

    bool data_ready;
    err = nau7802_data_available(dev, &data_ready);
    if(err < 0){
        goto unlock_ret;
    }

    err = nau7802_read_data(dev, &data);
    if(err < 0){
        goto unlock_ret;
    }

    dev->lower.push_event(dev->lower.priv, &data, sizeof(data));

    // I LOVE GOTO TRAPS GLORY TO GOTO
    unlock_ret:
        nxmutex_unlock(&dev->devlock);

    return err;
}

static int nau7802_on(FAR nau7802_dev_s *dev) {
    // TODO, if this works you can convert this to call the set bit func
    int err;
    uint8_t regval;

    err = nau7802_read_reg(dev, REG_PU_CTRL, &regval, sizeof(regval));
    if (err < 0) {
        return err;
    }

    regval |= 0x06; // setting bits 1 and 2 to 1

    return nau7802_write_reg(dev, REG_PU_CTRL, &regval, sizeof(regval));
}

static int nau7802_set_streaming_mode(FAR nau7802_dev_s *dev){
    // there is 2 streaming modes, the first one is the standard, the second one isn't compatible
    return nau7802_set_bit(dev, REG_I2C_CONTROL, 7, 1);
}

// static int nau7802_set_odr()
// static int nau7802_set_mode()
// static int nau7802_low_power() // not priotity
// static int nau7802_offset_enable() // probably also not priority
// static int nau7802_enable_interrupts() // TODO after
static const struct sensor_ops_s g_sensor_ops =
{
  .activate = nau7802_activate,
//   .set_interval = nau7802_set_interval,
//   .selftest = nau7802_selftest,
  .get_info = nau7802_get_info,
//   .control = nau7802_control,
//   .set_calibvalue = nau7802_set_calibvalue, // not happening 
};

static int nau7802_activate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, bool enable)
{
    FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
    bool start_thread = false;
    int err = 0;

    /* Start the collection thread if not already enabled */
    if (enable && !dev->enabled) {
        start_thread = true;

        err = nau7802_on(dev);
        if(err){
            return err;
        }

        err = nau7802_set_streaming_mode(dev);
        if(err){
            return err;
        }
    }

    dev->enabled = enable; 

    if (start_thread)
    {
        return nxsem_post(&dev->run);
    }

    return 0;
}

static int nau7802_get_info(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR struct sensor_device_info_s *info){
  FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower); // I don't know what this is honestly

  info->version = 0;
  info->power = 0; 
  memcpy(info->name, "NAU7802", sizeof("NAU7802")); // why not strlen?
  memcpy(info->vendor, "Nuvoton", sizeof("Nuvoton"));

  info->min_delay = ODR_TO_INTERVAL[ODR_10HZ];
  info->max_delay = ODR_TO_INTERVAL[ODR_10HZ];
  info->fifo_reserved_event_count = 0;
  info->fifo_max_event_count = 0;
  info->max_range = 0; // It's not fixed so idk
  info->resolution = 24; // I guess 24 bit resolution??? 

  return 0;
}

static int nau7802_thread(int argc, FAR char *argv[]){
    FAR nau7802_dev_s *dev = (FAR nau7802_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
    int err = 0;
    
    while(true){
        if(!dev->enabled){
            err = nxsem_wait(&dev->run);
            if(err < 0){
                continue;
            }
        }

        err = nau7802_push_data(dev);
        if(err < 0){
            return err;
        }
        
        nxsig_usleep(dev->odr); 
    }
}


int nau7802_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr,
                     nau7802_attach attach){
    int err;
    FAR nau7802_dev_s* priv = kmm_zalloc(sizeof(nau7802_dev_s));

    if(priv == NULL){
        snerr("Failed to allocate nau7802 instance\n");
        return -ENOMEM;
    }

    err = nxmutex_init(&priv->devlock);
    if(err < 0){
        snerr("Failed to register nau7802 driver: %d\n", err);
        goto del_mem;
    }

    // I don't know what this semaphore is for
    err = nxsem_init(&priv->run, 0, 0);
    if(err < 0){
        snerr("Failed to register nau7802 driver: %d\n", err);
        goto del_mutex;
    }
    
    priv->i2c = i2c;
    priv->addr = addr;
    priv->lower.ops = &g_sensor_ops;
    priv->lower.type = SENSOR_TYPE_FORCE; // TODO: Ask which sensor type is right for whatever this is
    priv->enabled = false;
    priv->interrupts = false;
    priv->odr = ODR_TO_INTERVAL[ODR_10HZ]; // time to sleep in ms TODO CHANGE THIS

    err = sensor_register(&priv->lower, devno);
    if(err < 0){
        snerr("Failed to register nau7802 driver: %d\n", err);
        goto del_sem;
    }
    else{
        snerr("REGISTERED NAU7802 DRIVER\n");
    }

    // attach is null for now because interrupt mode is in progress
    FAR char *argv[2];
    char arg1[32];

    snprintf(arg1, 16, "%p", priv); // I have no idea what the first 16bits of the lower struct are tbh
    argv[0] = arg1;
    argv[1] = NULL;

    err = kthread_create("nau7802_thread", SCHED_PRIORITY_DEFAULT,
                        CONFIG_SENSORS_NAU7802_THREAD_STACKSIZE, // TODO: set this config somewhere
                        nau7802_thread, argv);
    if (err < 0) {
        snerr("Failed to create the nau7802 notification kthread\n");
        goto sensor_unreg;
    }

    sninfo("Registered nau7802 driver with kernel polling thread\n");

    if (err < 0) {
        sensor_unreg:
            sensor_unregister(&priv->lower, devno);
        del_sem:
            nxsem_destroy(&priv->run);
        del_mutex:
            nxmutex_destroy(&priv->devlock);
        del_mem:
            kmm_free(priv);
        return err;
    }

    sninfo("Registered NAU7802 driver\n");
    return err;
}
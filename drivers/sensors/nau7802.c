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

static int nau7802_set_bits(FAR nau7802_dev_s *dev, uint8_t addr, uint8_t n_bits, uint8_t n_bit_shifts, uint8_t value) {
    int err = 0;
    uint8_t reg_val;

    err = nau7802_read_reg(dev, addr, &reg_val, sizeof(reg_val));
    if(err < 0) {
        return err;
    }

    uint8_t mask = ((1 << n_bits) - 1);
    
    reg_val &= ~(mask << n_bit_shifts);
    
    reg_val |= ((value & mask) << n_bit_shifts);

    return nau7802_write_reg(dev, addr, &reg_val, sizeof(reg_val));
}

static int nau7802_read_bit(FAR nau7802_dev_s *dev, uint8_t addr, uint8_t bit, bool *val){
    int err = 0;
    uint8_t reg_val;

    err = nau7802_read_reg(dev, addr, &reg_val, sizeof(reg_val));
    if(err < 0){
        return err;
    }

    *val = (reg_val >> bit) & 1;
    return err;
}

static int nau7802_reset(FAR nau7802_dev_s *dev){
    int err = 0;
    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_RR, 1);
    if(err < 0){
        return err;
    }

    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_RR, 0);
    if(err < 0){
        return err;
    }

    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUD, 1);
    if(err < 0){
        return err;
    }

    usleep(200); // waiting 200 micoroseconds for the power up

    uint8_t reg_val;
    err = nau7802_read_reg(dev, REG_PU_CTRL, &reg_val, sizeof(reg_val));
    if(err < 0){
        return err;
    }

    // check if power up is successful
    if(((reg_val >> BIT_PUR) & 1) == 1){
        snerr("Power up succesfull\n");
        return 0;
    }
    else{
        snerr("Power up failed\n");
        return -1;
    }
}

static int nau7802_enable(FAR nau7802_dev_s *dev, bool enable){
    int err = 0;
    if(!enable){
        err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUA, 0);
        if(err < 0){
            return err;
        }
        err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUD, 0);
        if(err < 0){
            return err;
        }
        return err;
    }

    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUD, 1);
    if(err < 0){
        return err;
    }

    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_PUA, 1);
    if(err < 0){
        return err;
    }

    usleep(600000); 

    // start cycle
    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_CS, 1);
    if(err < 0){
        return err;
    }

    bool reg_val;
    err = nau7802_read_bit(dev, REG_PU_CTRL, BIT_PUR, &reg_val);
    if(err < 0 || !reg_val){
        return err;
    }
    return err;
}

static int nau7802_data_available(FAR nau7802_dev_s *dev, bool *val){
    return nau7802_read_bit(dev, REG_PU_CTRL, BIT_CR, val);
}

static int nau7802_read_data(FAR nau7802_dev_s *dev, FAR struct sensor_force *data){
    uint8_t msb, mid, lsb;
    int32_t value;
    
    nau7802_read_reg(dev, REG_ADCO_B2, &msb, sizeof(msb));
    nau7802_read_reg(dev, REG_ADCO_B1, &mid, sizeof(mid));
    nau7802_read_reg(dev, REG_ADCO_B0, &lsb, sizeof(lsb));
    
    // Combine into 24-bit value and sign extend to 32-bit
    value = (int32_t)((uint32_t)msb << 16 | (uint32_t)mid << 8 | lsb);
    
    // Sign extend if negative (MSB is set)
    if (value & 0x800000) {
        value |= 0xFF000000;
    }


    data->timestamp = sensor_get_timestamp();
    data->event = 0;
    data->force = value;
    return 0;
}

static int nau7802_set_ldo(FAR nau7802_dev_s *dev, nau7802_ldo_voltage_enum voltage){
    if(voltage == LDO_V_EXTERNAL){
        return nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_AVVDS, 0);
    }
    
    int err = 0;
    err = nau7802_set_bits(dev, REG_PU_CTRL, 1, BIT_AVVDS, 1);
    if(err < 0){
        return err;
    }

    return nau7802_set_bits(dev, REG_CTRL_1, 3, 3, voltage);
}

static int nau7802_set_gain(FAR nau7802_dev_s *dev, nau7802_gain_e gain){
    return nau7802_set_bits(dev, REG_CTRL_1, 3, 0, gain);
}

static int nau7802_set_sample_rate(FAR nau7802_dev_s *dev, nau7802_sample_rate_e rate){
    return nau7802_set_bits(dev, REG_CTRL_2, 3, 4, rate);
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
    if(err < 0 || !data_ready){
        snerr("Data not ready\n");
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

static int nau7802_get_gain_offset_calibvalue(FAR nau7802_dev_s *dev, unsigned long arg){
    uint32_t *calibvalue = (uint32_t *)arg;
    uint8_t reg_b3;
    uint8_t reg_b2;
    uint8_t reg_b1;
    uint8_t reg_b0;

    nau7802_read_reg(dev, REG_GCAL1_B3, &reg_b3, sizeof(reg_b3));
    nau7802_read_reg(dev, REG_GCAL1_B2, &reg_b2, sizeof(reg_b2));
    nau7802_read_reg(dev, REG_GCAL1_B1, &reg_b1, sizeof(reg_b1));
    nau7802_read_reg(dev, REG_GCAL1_B0, &reg_b0, sizeof(reg_b0));

    *calibvalue = (reg_b3 << 24) | (reg_b2 << 16) | (reg_b1 << 8) | reg_b0;
    return 0;
}

static int nau7802_calibrate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, unsigned long int arg) {
    FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
    nau7802_calibration_mode_e mode = (nau7802_calibration_mode_e)arg;
    int err = 0;
    
    // choosing calibration mode
    err = nau7802_set_bits(dev, REG_CTRL_2, 2, 0, mode);
    if(err < 0){
        return err;
    }
    
    // start calibration
    err = nau7802_set_bits(dev, REG_CTRL_2, 1, CAL_START, 1);
    if(err < 0){
        return err;
    }

    // Wait for calibration to complete
    bool reg_val;
    do {
        err = nau7802_read_bit(dev, REG_CTRL_2, CAL_START, &reg_val);
        if(err < 0){
            return err;
        }
        usleep(10000);
    } while(reg_val);

    // Check calibration error bit
    err = nau7802_read_bit(dev, REG_CTRL_2, CAL_ERR, &reg_val);
    if(err < 0){
        return err;
    }

    if(reg_val){
        snerr("Calibration failed\n");
        return -1;
    }
    return 0;
}

static int nau7802_control(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, int cmd, unsigned long arg) {
    FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
    int err = 0;

    err = nxmutex_lock(&dev->devlock);
    if (err){
        return err;
    }

    switch (cmd) {
        // /* Soft reset */
        case SNIOC_RESET:
            err = nau7802_reset(dev);
            break;

        case SNIOC_SET_GAIN:
            err = nau7802_set_gain(dev, arg);
            break;

        case SNIOC_SET_SAMPLE_RATE:
            err = nau7802_set_gain(dev, arg);
            break;
        
        case SNIOC_SET_LDO:
            err = nau7802_set_ldo(dev, arg);
            break;
        
        case SNIOC_GET_GAIN_CALIBVALUE:
            err = nau7802_get_gain_offset_calibvalue(dev, arg);
            break;

        default:
            err = -EINVAL;
            snerr("Unknown command for LIS2MDL: lu\n", cmd);
            break;
    }

    nxmutex_unlock(&dev->devlock);
    return err;
}

static int nau7802_activate(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, bool enable) {
    FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower);
    bool start_thread = false;
    int err = 0;

    /* Start the collection thread if not already enabled */
    if (enable && !dev->enabled) {
        start_thread = true;

        err = nau7802_reset(dev);
        if(err < 0){
            return err;
        }  

        err = nau7802_enable(dev, true);
        if(err < 0){
            return err;
        }
        
        /* Check for NAU7802 revision register (0x1F), low nibble should be 0xF. */
        uint8_t reg_val;
        err = nau7802_read_reg(dev, 0x1F, &reg_val, sizeof(reg_val));
        if(err < 0 || (reg_val & 0xF) != 0xF){
            snerr("Could not read the revision register\n");
            return err;
        }


        err = nau7802_set_ldo(dev, LDO_V_3V0);
        if(err < 0){
            return err;
        }

        err = nau7802_set_gain(dev, GAIN_128);
        if(err < 0){
            return err;
        }

        err = nau7802_set_sample_rate(dev, SPS_10);
        if(err < 0){
            return err;
        }

        // disable ADC chopper 
        err = nau7802_set_bits(dev, REG_ADC, 2, 4, 0x3);
        if(err < 0){
            return err;
        }

        // use low ESR caps
        err = nau7802_set_bits(dev, REG_PGA, 1, 6, 0);
        if(err < 0){
            return err;
        }

        // PGA stabilizer cap on output
        err = nau7802_set_bits(dev, REG_POWER, 1, 7, 1);
        if(err < 0){
            return err;
        }

        // perform internal calibration
        // err = nau7802_calibrate(dev, CALMOD_INTERNAL);
        // if(err < 0){
        //     return err;
        // }
    }

    dev->enabled = enable; 

    if (start_thread)
    {
        return nxsem_post(&dev->run);
    }

    return 0;
}

// TODO FIX THIS
static int nau7802_get_info(FAR struct sensor_lowerhalf_s *lower, FAR struct file *filep, FAR struct sensor_device_info_s *info){
  FAR nau7802_dev_s *dev = container_of(lower, FAR nau7802_dev_s, lower); // I don't know what this is honestly

  info->version = 0;
  info->power = 0; 
  memcpy(info->name, "NAU7802", sizeof("NAU7802")); // why not strlen?
  memcpy(info->vendor, "Nuvoton", sizeof("Nuvoton"));

  info->min_delay = SPS_TO_INTERVAL[SPS_10];
  info->max_delay = SPS_TO_INTERVAL[SPS_10];
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

static const struct sensor_ops_s g_sensor_ops =
{
  .activate = nau7802_activate,
  .get_info = nau7802_get_info,
  .control = nau7802_control,
  .calibrate = nau7802_calibrate
};

int nau7802_register(FAR struct i2c_master_s *i2c, int devno, uint8_t addr,  nau7802_attach attach){
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
    priv->odr = SPS_TO_INTERVAL[SPS_10]; 

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
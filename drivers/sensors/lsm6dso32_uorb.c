/****************************************************************************
 * drivers/sensors/lsm6dso32_uorb.c
 *
 * Contributed by Carleton University InSpace
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <cassert>
#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/semaphore.h>
#include <nuttx/sensors/lsm6dso32.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The value that should be in the WHO_AM_I register. */

#define WHO_AM_I_VAL 0x6c

/* Convert milli Gs to m/s^2 */

#define MILLIG_TO_MS2 (0.0098067f)

/* Convert dps to rad/s */

#define DPS_TO_RADS (180.0f / 3.141592653f)

#ifndef CONFIG_LSM6DSO32_I2C_FREQUENCY
#define CONFIG_LSM6DSO32_I2C_FREQUENCY 400000
#endif /* CONFIG_LSM6DSO32_I2C_FREQUENCY */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ODRs common to the accelerometer and the gyroscope */

enum lsm6dso32_odr_e
{
  ODR_OFF = 0x0,    /* Sensor Deactivated */
  ODR_12_5HZ = 0x1, /* 12.5Hz Rate. */
  ODR_26HZ = 0x2,   /* 26Hz Rate. */
  ODR_52HZ = 0x3,   /* 52Hz Rate. */
  ODR_104HZ = 0x4,  /* 104Hz Rate */
  ODR_208HZ = 0x5,  /* 208Hz Rate */
  ODR_416HZ = 0x6,  /* 416Hz Rate */
  ODR_833HZ = 0x7,  /* 833Hz Rate */
  ODR_1660HZ = 0x8, /* 1.66kHz Rate */
  ODR_3330HZ = 0x9, /* 3.33kHz Rate */
  ODR_6660HZ = 0xa, /* 6.66kHz Rate */
  ODR_1_6HZ = 0xb,  /* 1.6Hz Rate */
};

/* Represents a lower half sensor driver of the LSM6DSO32 */

struct lsm6dso32_sens_s
{
  FAR struct sensor_lowerhalf_s lower; /* Lower-half sensor driver */
  FAR struct lsm6dso32_dev_s *dev;     /* Reference to parent device */
  bool enabled;                        /* If this sensor is enabled */
  enum lsm6dso32_odr_e odr;            /* Measurement interval of this
                                        * sensor */
  int fsr;                             /* Full scale range of this sensor.
                                        * Can be from either gyro or accel
                                        * FSR enum. */
  FAR struct lsm6dso32_dev_s *dev;     /* Reference to parent device */
};

/* Represents the LSM6DSO23 IMU device */

struct lsm6dso32_dev_s
{
  struct lsm6dso32_sens_s gyro;  /* Gyroscope */
  struct lsm6dso32_sens_s accel; /* Accelerometer lower half */
  FAR struct i2c_master_s *i2c;  /* I2C interface. */
  uint8_t addr;                  /* I2C address. */
  mutex_t devlock;
};

#define WHO_AM_I 0x0f   /* Hard-coded address on I2C bus. */
#define TIMESTAMP0 0x40 /* First timestamp register (32 bits) */
#define STATUS_REG 0x1e /* The status register */
#define CTRL1_XL 0x10   /* Accel control reg 1 */
#define CTRL2_G 0x11    /* Gyro control reg 2 */
#define CTRL3_C 0x12    /* Control reg 3 */
#define CTRL4_C 0x13    /* Control reg 4 */
#define CTRL5_C 0x14    /* Control reg 5 */
#define CTRL6_C 0x15    /* Control reg 6 */
#define CTRL7_G 0x16    /* Control reg 7 */
#define CTRL8_XL 0x17   /* Control reg 8 */
#define CTRL9_XL 0x18   /* Control reg 9 */
#define CTRL10_C 0x19   /* Control reg 10 */
#define FIFO_CTRL4 0x0a /* The fourth FIFO control reg  */
#define OUT_TEMP_L 0x20 /* Temp output low byte. */
#define OUT_TEMP_H 0x21 /* Temp output high byte. */
#define OUTX_L_G 0x22   /* Gyro pitch axis (X) low byte. */
#define OUTX_H_G 0x23   /* Gyro pitch axis (X) high byte. */
#define OUTY_L_G 0x24   /* Gyro roll axis (Y) low byte. */
#define OUTY_H_G 0x25   /* Gyro roll axis (Y) high byte. */
#define OUTZ_L_G 0x26   /* Gyro yaw axis (Z) low byte. */
#define OUTZ_H_G 0x27   /* Gyro yaw axis (Z) high byte. */
#define OUTX_L_A 0x28   /* Accel (X) low byte. */
#define OUTX_H_A 0x29   /* Accel (X) high byte. */
#define OUTY_L_A 0x2a   /* Accel (Y) low byte. */
#define OUTY_H_A 0x2b   /* Accel (Y) high byte. */
#define OUTZ_L_A 0x2c   /* Accel (Z) low byte. */
#define OUTZ_H_A 0x2d   /* Accel (Z) high byte. */
#define X_OFS_USR 0x73  /* X offset correction accel */
#define Y_OFS_USR 0x74  /* Y offset correction accel */
#define Z_OFS_USR 0x75  /* Z offset correction accel */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6dso32_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg);
static int lsm6dso32_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable);
static int lsm6dso32_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us);
static int lsm6dso32_fetch(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static int lsm6dso32_selftest(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, unsigned long arg);
static int lsm6dso32_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                    FAR struct file *filep,
                                    unsigned long arg);
static int lsm6dso32_calibrate(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep, unsigned long arg);
static int lsm6dso32_get_info(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR struct sensor_device_info_s *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ODR frequencies to measurement intervals in microseconds */

static const uint32_t ODR_INTERVAL[] = {
    [ODR_OFF] = 0,      [ODR_12_5HZ] = 80000, [ODR_26HZ] = 38462,
    [ODR_52HZ] = 19230, [ODR_104HZ] = 9615,   [ODR_208HZ] = 4807,
    [ODR_416HZ] = 2403, [ODR_833HZ] = 1200,   [ODR_1660HZ] = 602,
    [ODR_3330HZ] = 300, [ODR_6660HZ] = 150,   [ODR_1_6HZ] = 625000,
};

/* Accelerometer FSR sensitivities in m/s^2 per LSB */

static const float FSR_XL_SENS[] = {
    [LSM6DSO32_FSR_XL_4G] = 0.122f * MILLIG_TO_MS2,
    [LSM6DSO32_FSR_XL_32G] = 0.244f * MILLIG_TO_MS2,
    [LSM6DSO32_FSR_XL_8G] = 0.488f * MILLIG_TO_MS2,
    [LSM6DSO32_FSR_XL_16G] = 0.976f * MILLIG_TO_MS2,
};

/* Gyro FSR sensitivities in rad/s per LSB */

static const float FSR_GYRO_SENS[] = {
    [LSM6DSO32_GYRO_250DPS] = 4.375f * DPS_TO_RADS,
    [LSM6DSO32_GYRO_125DPS] = 8.75f * DPS_TO_RADS,
    [LSM6DSO32_GYRO_500DPS] = 17.50f * DPS_TO_RADS,
    [LSM6DSO32_GYRO_1000DPS] = 35.0f * DPS_TO_RADS,
    [LSM6DSO32_GYRO_2000DPS] = 70.0f * DPS_TO_RADS,
};

/* Sensor operations */

static const struct sensor_ops_s g_sensor_ops = {
    .fetch = NULL, // TODO optionally null
    .control = lsm6dso32_control,
    .set_interval = lsm6dso32_set_interval,
    .selftest = lsm6dso32_selftest,
    .set_calibvalue = lsm6dso32_set_calibvalue,
    .calibrate = lsm6dso32_calibrate,
    .get_info = lsm6dso32_get_info,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso32_write_bytes
 *
 * Description:
 *   Write bytes to the LSM6DSO32 sensor. Providing more than one byte will
 *   write to sequential registers starting at the provided address.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   addr    - The register address to write to.
 *   buf     - The buffer of data to write.
 *   nbytes  - The number of bytes in the buffer to write.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_write_bytes(struct lsm6dso32_dev_s *priv, uint8_t addr,
                                 void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  /* Register addressing part of command. */

  cmd[0].frequency = CONFIG_LSM6DSO32_I2C_FREQUENCY;
  cmd[0].addr = priv->addr;
  cmd[0].flags = I2C_M_NOSTOP;
  cmd[0].buffer = &addr;
  cmd[0].length = sizeof(addr);

  /* Data to write. */

  cmd[1].frequency = CONFIG_LSM6DSO32_I2C_FREQUENCY;
  cmd[1].addr = priv->addr;
  cmd[1].flags = I2C_M_NOSTART;
  cmd[1].buffer = buf;
  cmd[1].length = nbytes;

  /* Send command over the wire */

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6dso32_read_bytes
 *
 * Description:
 *   Read bytes from the LSM6DSO32 sensor. Reading more than one byte will
 *   read from sequential registers starting at the provided address.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   addr    - The register address to read from.
 *   buf     - The buffer of data to read into.
 *   nbytes  - The number of bytes to read into the buffer.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_read_bytes(struct lsm6dso32_dev_s *priv, uint8_t addr,
                                void *buf, size_t nbytes)
{
  struct i2c_msg_s cmd[2];

  /* Register addressing part of command. */

  cmd[0].frequency = CONFIG_LSM6DSO32_I2C_FREQUENCY;
  cmd[0].addr = priv->addr;
  cmd[0].flags = I2C_M_NOSTOP;
  cmd[0].buffer = &addr;
  cmd[0].length = sizeof(addr);

  /* Read data into buffer. */

  cmd[1].frequency = CONFIG_LSM6DSO32_I2C_FREQUENCY;
  cmd[1].addr = priv->addr;
  cmd[1].flags = I2C_M_NOSTART | I2C_M_READ;
  cmd[1].buffer = buf;
  cmd[1].length = nbytes;

  /* Send command over the wire */

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

/****************************************************************************
 * Name: lsm6dso32_set_bits
 *
 * Description:
 *   Read current value of desired register and change specified bits
 *   while preserving previous ones.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   addr    - The register address being changed.
 *   set_bits - A mask of the bits to be set to 1.
 *   clear_bits  - A mask of the bits to be set to 0.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_set_bits(struct lsm6dso32_dev_s *priv, uint8_t addr,
                              uint8_t set_bits, uint8_t clear_bits)
{
  int err;
  uint8_t reg;

  err = lsm6dso32_read_bytes(priv, addr, &reg, sizeof(reg));

  if (err < 0)
    {
      return err;
    }

  reg = (reg & ~clear_bits) | set_bits;
  err = lsm6dso32_write_bytes(priv, addr, &reg, sizeof(reg));

  if (err < 0)
    {
      return err;
    }

  return err;
}

/****************************************************************************
 * Name: accel_set_odr
 *
 * Description:
 *      Sets the accelerometer ODR.
 ****************************************************************************/

static int accel_set_odr(FAR struct lsm6dso32_dev_s *dev,
                         enum lsm6dso32_odr_e odr)
{
  int err;

  err = lsm6dso32_set_bits(dev, CTRL1_XL, (odr & 0xf) << 4, 0xf0);

  if (err < 0)
    {
      return err;
    }

  dev->odr = odr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_odr
 *
 * Description:
 *      Sets the gyroscope ODR.
 ****************************************************************************/

static int gyro_set_odr(FAR struct lsm6dso32_dev_s *dev,
                        enum lsm6dso32_odr_e odr)
{
  DEBUGASSERT(odr != ODR_1_6HZ); /* Invalid setting for gyroscope */

  int err;

  err = lsm6dso32_set_bits(dev, CTRL2_G, (odr & 0x0f) << 4, 0xf0);

  if (err < 0)
    {
      return err;
    }

  dev->odr = odr;

  return err;
}

/****************************************************************************
 * Name: accel_set_fsr
 *
 * Description:
 *      Sets the accelerometer FSR.
 ****************************************************************************/

static int accel_set_fsr(FAR struct lsm6dso32_dev_s *dev,
                         enum lsm6dso32_fsr_xl_e fsr)
{
  int err;

  err = lsm6dso32_set_bits(dev, CTRL1_XL, (fsr & 0x3) << 2, 0x0c);

  if (err < 0)
    {
      return err;
    }

  dev->accel.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: gyro_set_fsr
 *
 * Description:
 *      Sets the gyroscope FSR.
 ****************************************************************************/

static int accel_set_fsr(FAR struct lsm6dso32_dev_s *dev,
                         enum lsm6dso32_fsr_gyro_e fsr)
{
  int err;

  err = lsm6dso32_set_bits(dev, CTRL1_G, (fsr & 0x7) << 1, 0x0c);

  if (err < 0)
    {
      return err;
    }

  dev->gyro.fsr = fsr;
  return err;
}

/****************************************************************************
 * Name: lsm6dso32_convert_temp
 *
 * Description:
 *   Converts raw temperature reading into units of degrees Celsius.
 *   TODO
 ****************************************************************************/

static float lsm6dso32_convert_temp(int16_t temp) { return (float)(temp); }

/****************************************************************************
 * Name: lsm6dso32_read_gyro
 *
 * Description:
 *   Reads gyroscope data into UORB structure.
 ****************************************************************************/

static int lsm6dso32_read_gyro(FAR struct lsm6dso32_dev_s *dev,
                               FAR struct sensor_gyro *data)
{
  int16_t raw_data[4]; /* Holds 1 temp, 3 gyro (xyz) */
  int err;

  err = lsm6dso32_read_bytes(dev, OUT_TEMP_L, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  /* Convert data into the format required */

  data->timesamp = sensor_get_timestamp();
  data->temperature = lsm6dso32_convert_temp(raw_data[0]);
  data->x = (float)(raw_data[1]) * FSR_GYRO_SENS[dev->gyro.fsr];
  data->y = (float)(raw_data[2]) * FSR_GYRO_SENS[dev->gyro.fsr];
  data->z = (float)(raw_data[3]) * FSR_GYRO_SENS[dev->gyro.fsr];

  // TODO
  return err;
}

/****************************************************************************
 * Name: lsm6dso32_read_accel
 *
 * Description:
 *   Reads accelerometer data into UORB structure.
 ****************************************************************************/

static int lsm6dso32_read_accel(FAR struct lsm6dso32_dev_s *dev,
                                FAR struct sensor_gyro *data)
{
  int16_t raw_data[3]; /* 3 accel (xyz) */
  int16_t raw_temp;    /* Temperature */
  int err;

  /* Get accelerometer data */

  err = lsm6dso32_read_bytes(dev, OUTX_L_A, raw_data, sizeof(raw_data));
  if (err < 0)
    {
      return err;
    }

  /* Get temperature data TODO can I bundle this with gyro by decoupling? */

  err = lsm6dso32_read_bytes(dev, OUT_TEMP_L, &raw_temp, sizeof(raw_temp));
  if (err < 0)
    {
      return err;
    }

  /* Convert data into the required format */

  data->timesamp = sensor_get_timestamp();
  data->temperature = lsm6dso32_convert_temp(raw_temp);
  data->x = (float)(raw_data[0]) * FSR_XL_SENS[dev->accel.fsr];
  data->y = (float)(raw_data[1]) * FSR_XL_SENS[dev->accel.fsr];
  data->z = (float)(raw_data[2]) * FSR_XL_SENS[dev->accel.fsr];

  return err;
}

/****************************************************************************
 * Name: lsm6dso32_read_raw_data
 *
 * Description:
 *   Read raw acceleration and rotational data of the sensor.
 *
 * Input Parameters:
 *   priv    - The instance of the LSM6DSO32 sensor.
 *   raw_data    - The structure to store the data in.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int lsm6dso32_read_raw_data(struct lsm6dso32_dev_s *priv,
                                   struct lsm6dso32_data_raw_s *raw_data)
{
  int err;
  uint8_t register_data[12];

  /* Read all data registers into array */

  err = lsm6dso32_read_bytes(priv, OUTX_L_G, &register_data,
                             sizeof(register_data));
  if (err < 0)
    {
      return err;
    }

  raw_data->pitch_raw = register_data[0] | (register_data[1] << 8);
  raw_data->roll_raw = register_data[2] | (register_data[3] << 8);
  raw_data->yaw_raw = register_data[4] | (register_data[5] << 8);
  raw_data->x_accel_raw = register_data[6] | (register_data[7] << 8);
  raw_data->y_accel_raw = register_data[8] | (register_data[9] << 8);
  raw_data->z_accel_raw = register_data[10] | (register_data[11] << 8);

  return 0;
}

/****************************************************************************
 * Name: lsm6dso32_convert_raw_data
 *
 * Description:
 *   Convert raw acceleration and rotational data of the sensor to actual
 *values.
 *
 * Input Parameters:
 *   raw_data    - The structure containing the raw data.
 *   conv_data   - The structure to store the converted data.
 *
 ****************************************************************************/

static void lsm6dso32_convert_raw_data(struct lsm6dso32_data_raw_s *raw_data,
                                       struct lsm6dso32_data_s *conv_data)
{
  conv_data->pitch = (raw_data->pitch_raw) * 4.375 * gyro_fsr_level;
  conv_data->roll = (raw_data->roll_raw) * 4.375 * gyro_fsr_level;
  conv_data->yaw = (raw_data->yaw_raw) * 4.375 * gyro_fsr_level;
  conv_data->x_accel = (raw_data->x_accel_raw) * 0.122 * accel_fsr_level;
  conv_data->y_accel = (raw_data->y_accel_raw) * 0.122 * accel_fsr_level;
  conv_data->z_accel = (raw_data->z_accel_raw) * 0.122 * accel_fsr_level;
  return;
}

/****************************************************************************
 * Name: lsm6dso32_activate
 ****************************************************************************/

static int lsm6dso32_activate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, bool enable)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  int err;
}

/****************************************************************************
 * Name: lsm6dso32_set_interval
 ****************************************************************************/

static int lsm6dso32_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                  FAR struct file *filep,
                                  FAR uint32_t *period_us)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  int err;
  enum lsm6dso32_odr_e odr;

  if (*period_us >= 625000 && lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      /* 1.6Hz is requested and this is the accelerometer: if we're low power
       * mode 1.6Hz will apply, but if we're in high performance mode then the
       * nearest rate of 12.5Hz will be chosen.
       */

      odr = ODR_1_6HZ;
    }
  else if (*period_us >= 80000)
    {
      odr = ODR_12_5HZ;
    }
  else if (*period_us >= 38462 && *period_us < 80000)
    {
      odr = ODR_26HZ;
    }
  else if (*period_us >= 19231 && *period_us < 38462)
    {
      odr = ODR_52HZ;
    }
  else if (*period_us >= 9615 && *period_us < 19231)
    {
      odr = ODR_104HZ;
    }
  else if (*period_us >= 4808 && *period_us < 9615)
    {
      odr = ODR_208HZ;
    }
  else if (*period_us >= 2404 && *period_us < 4808)
    {
      odr = ODR_416HZ;
    }
  else if (*period_us >= 1200 && *period_us < 2404)
    {
      odr = ODR_833HZ;
    }
  else if (*period_us >= 602 && *period_us < 1200)
    {
      odr = ODR_1660HZ;
    }
  else if (*period_us >= 300 && *period_us < 602)
    {
      odr = ODR_3330HZ;
    }
  else
    {
      odr = ODR_6660HZ;
    }

  /* Get exclusive device access before setting the ODR */

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      err = accel_set_odr(dev, odr);
    }
  else if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      err = gyro_set_odr(dev, odr);
    }

  if (err < 0)
    {
      goto early_ret;
    }

  /* Only set the interval value if successful */

  *period_us = ODR_INTERVAL[odr];

early_ret:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Name: lsm6dso32_set_info
 ****************************************************************************/

static int lsm6dso32_get_info(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep,
                              FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);

  info->version = 0;
  info->power = 0.55f; /* 0.55mA in high performance */
  memcpy(info->name, "LSM6DSO32", sizeof("LSM6DSO32"));
  memcpy(info->vendor, "STMicro", sizeof("STMicro"));

  // TODO fifo

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      // TODO max range from FSR
      // TODO resolution from FSR
    }
  else if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      // TODO max range from FSR
      // TODO resolution from FSR
    }
}

/****************************************************************************
 * Name: lsm6dso32_control
 ****************************************************************************/

static int lsm6dso32_control(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct lsm6dso32_sens_s *sens =
      container_of(lower, FAR struct lsm6dso32_sens_s, lower);
  FAR struct lsm6dso32_dev_s *dev = sens->dev;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
      /* Read WHO_AM_I value into 8-bit unsigned integer buffer */

    case SNIOC_WHO_AM_I:
      {
        uint8_t *id = (uint8_t *)(arg);
        if (arg == NULL)
          {
            err = -EINVAL;
            break;
          }
        err = lsm6dso32_read_bytes(priv, WHO_AM_I, id, sizeof(uint8_t));
      }
      break;

    case SNIOC_SETFULLSCALE:
      {
        // TODO sanitize inputs
        if (lower->type == SENSOR_TYPE_ACCELEROMETER)
          {
            err = accel_set_fsr(dev, arg);
          }
        else if (lower->type == SENSOR_TYPE_GYROSCOPE)
          {
            err = gyro_set_fsr(dev, arg);
          }
      }
      break;

    default:
      {
        err = -EINVAL;
      }
      break;
    }

  nxmutex_unlock(&priv->devlock);
  return err;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso32_register
 *
 * Description:
 *   Register the LSM6DSO32 character device as a UORB sensor with accel, gyro
 *   and temperature
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM6DSO32
 *   addr    - The I2C address of the LSM6DSO32.
 *   devno   - The device number for the UORB topics registered (i.e.
 *             sensor_accel<n>)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dso32_register(FAR struct i2c_master_s *i2c, uint8_t addr,
                       uint8_t devno)
{
  FAR struct lsm6dso32_dev_s *priv;
  int err;

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x6b || addr == 0x6c);

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct lsm6dso32_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance of LSM6DSO32 driver.\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;

  /* Create mutex */

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("ERROR: Failed to register LSM6DSO32 driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Create gyro lower half */

  priv->gyro.lower.type = SENSOR_TYPE_GYROSCOPE;
  priv->gyro.lower.ops = &g_sensor_ops;
  priv->gyro.lower.nbuffer = -1;
  priv->gyro.lower.enabled = false;
  priv->gyro.interval = 0; /* Default off */
  priv->gyro.dev = priv;

  err = sensor_register(&priv->gyro.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DSO32 gyroscope lower half: %d\n", err);
      goto del_mutex;
    }

  /* Create accel lower half */

  priv->accel.lower.type = SENSOR_TYPE_ACCELEROMETER;
  priv->accel.lower.ops = &g_sensor_ops;
  priv->accel.lower.nbuffer = -1;
  priv->accel.lower.enabled = false;
  priv->accel.interval = 0; /* Default off */
  priv->accel.dev = priv;

  err = sensor_register(&priv->accel.lower, devno);
  if (err < 0)
    {
      snerr("Failed to register LSM6DSO32 accelerometer lower half: %d\n",
            err);
      goto unreg_gyro;
    }

  /* Register interrupt TODO */

  if (err < 0)
    {
      snerr("ERROR: Failed to register LSM6DSO32 driver: %d\n", err);
    unreg_accel:
      sensor_unregister(&priv->accel.lower, devno);
    unreg_gyro:
      sensor_unregister(&priv->gyro.lower, devno);
    del_mutex:
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  return err;
}

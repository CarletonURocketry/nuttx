/****************************************************************************
 * drivers/sensors/lsm6dso32.c
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/random.h>
#include <nuttx/sensors/lsm6dso32.h>

#include <stdio.h>
#include <unistd.h>

#define CONFIG_I2C
#define CONFIG_SENSORS_LSM6DSO32
#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSO32)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The value that should be in the WHO_AM_I register. */
#define WHO_AM_I_VAL 0x6c

#ifndef CONFIG_LSM6DSO32_I2C_FREQUENCY
#define CONFIG_LSM6DSO32_I2C_FREQUENCY 40000
#endif // CONFIG_LSM6DSO32_I2C_FREQUENCY

/****************************************************************************
 * Private
 ****************************************************************************/

struct lsm6dso32_dev_s
{
  FAR struct i2c_master_s *i2c; /* I2C interface. */
  uint8_t addr;                 /* I2C address. */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked; /* True, driver has been unlinked. */
#endif           // CONFIG_DISABLE_PSEUDOFS_OPERATIONS
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  int16_t crefs; /* Number of open references. */
#endif           // CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  mutex_t devlock;
};

enum lsm6dso32_reg
{
  WHO_AM_I = 0x0F,   /* Returns the hard-coded address
                      * of the IMU on the I2C bus. */
  TIMESTAMP0 = 0x40, /* First timestamp register (32 bits total) */
  STATUS_REG = 0x1E, /* The status register of whether data is available. */
  CTRL1_XL = 0x10,   /* Accelerometer control register 1 */
  CTRL2_G = 0x11,    /* Gyroscope control register 2 */
  CTRL3_C = 0x12,    /* Control register 3 */
  CTRL4_C = 0x13,    /* Control register 4 */
  CTRL5_C = 0x14,    /* Control register 5 */
  CTRL6_C = 0x15,    /* Control register 6 */
  CTRL7_G = 0x16,    /* Control register 7 */
  CTRL8_XL = 0x17,   /* Control register 8 */
  CTRL9_XL = 0x18,   /* Control register 9 */
  CTRL10_C = 0x19,   /* Control register 10 */
  FIFO_CTRL4 = 0x0A, /* The fourth FIFO control register (for setting
                        continuous mode) */
  OUT_TEMP_L = 0x20, /* The temperature data output
                      * low byte. (Two's complement) */
  OUT_TEMP_H = 0x21, /* The temperature data output
                      * high byte. (Two's complement) */
  OUTX_L_G = 0x22, /* The angular rate sensor pitch axis (X) low byte. (Two's
                      complement) */
  OUTX_H_G = 0x23, /* The angular rate sensor pitch axis (X) high byte. (Two's
                      complement) */
  OUTY_L_G = 0x24, /* The angular rate sensor roll axis (Y) low byte. (Two's
                      complement) */
  OUTY_H_G = 0x25, /* The angular rate sensor roll axis (Y) high byte. (Two's
                      complement) */
  OUTZ_L_G = 0x26, /* The angular rate sensor yaw axis (Z) low byte. (Two's
                      complement) */
  OUTZ_H_G = 0x27, /* The angular rate sensor yaw axis (Z) high byte. (Two's
                      complement) */
  OUTX_L_A = 0x28, /* The linear acceleration (X)
                    * low byte. (Two's complement) */
  OUTX_H_A = 0x29, /* The linear acceleration (X)
                    * high byte. (Two's complement) */
  OUTY_L_A = 0x2A, /* The linear acceleration (Y)
                    * low byte. (Two's complement) */
  OUTY_H_A = 0x2B, /* The linear acceleration (Y)
                    * high byte. (Two's complement) */
  OUTZ_L_A = 0x2C, /* The linear acceleration (Z)
                    * low byte. (Two's complement) */
  OUTZ_H_A = 0x2D, /* The linear acceleration (Z)
                    * high byte. (Two's complement) */
  X_OFS_USR = 0x73, /* The x-axis user offset correction
                     * for linear acceleration. */
  Y_OFS_USR = 0x74, /* The y-axis user offset correction
                     * for linear acceleration. */
  Z_OFS_USR = 0x75, /* The z-axis user offset correction
                     * for linear acceleration. */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6dso32_open(FAR struct file *filep);
static int lsm6dso32_close(FAR struct file *filep);
static ssize_t lsm6dso32_write(FAR struct file *filep, FAR const char *buffer,
                               size_t buflen);
static ssize_t lsm6dso32_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static int lsm6dso32_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int lsm6dso32_unlink(FAR struct inode *inode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/*Bit Masks for Setting Sample Rates*/
static const uint8_t ODR_BIT_MASKS[12][2] =
{
  {0b00000000, 0b00001111},  //[LSM6DSO32_OFF] 
  {0b00010000, 0b00011111},  //[LSM6DSO32_12_5HZ] 
  {0b00100000, 0b00101111},  //[LSM6DSO32_26HZ] 
  {0b00110000, 0b00111111},  //[LSM6DSO32_52HZ] 
  {0b01000000, 0b01001111},  //[LSM6DSO32_104HZ] 
  {0b01010000, 0b01011111},  //[LSM6DSO32_208HZ] 
  {0b01100000, 0b01101111},  //[LSM6DSO32_416HZ] 
  {0b01110000, 0b01111111},  //[LSM6DSO32_833HZ] 
  {0b10000000, 0b10001111},  //[LSM6DSO32_1660HZ] 
  {0b10010000, 0b10011111},  //[LSM6DSO32_3330HZ] 
  {0b10100000, 0b10101111},  //[LSM6DSO32_6660HZ] 
  {0b10110000, 0b10111111}   //[LSM6DSO32_1_6HZ] 
};

/*Bit Masks for Setting Resolutions*/
static const uint8_t FSR_BIT_MASKS[5][2] =
{ 
  {0b00000000, 0b11110001},   //[LSM6DSO32_GYRO_250DPS] 
  {0b00000100, 0b11110101},   //[LSM6DSO32_GYRO_500DPS] 
  {0b00001000, 0b11111001},   //[LSM6DSO32_GYRO_1000DPS]
  {0b00001100, 0b11111101},   //[LSM6DSO32_GYRO_2000DPS]
  {0b00000010, 0b11110011}    //[LSM6DSO32_GYRO_125DPS] 
};

/*Store the current FSR levels for conversion of raw data*/
static int gyro_fsr_level = 2;
static int accel_fsr_level = 1;

static const struct file_operations g_lsm6dso32fops = {
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .open = lsm6dso32_open,
    .close = lsm6dso32_close,
#else
    .open = NULL,
    .close = NULL,
#endif
    .read = lsm6dso32_read,
    .write = lsm6dso32_write,
    .seek = NULL,
    .ioctl = lsm6dso32_ioctl,
    .mmap = NULL,
    .truncate = NULL,
    .poll = NULL,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
    .unlink = lsm6dso32_unlink,
#endif
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
  reg = (reg & clear_bits) | set_bits;
  err = lsm6dso32_write_bytes(priv, addr, &reg, sizeof(reg));
  if (err < 0)
    {
      return err;
    }
  return 0;
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

static int lsm6dso32_read_raw_data(struct lsm6dso32_dev_s *priv, struct lsm6dso32_data_raw_s *raw_data)
{
  int err;
  uint8_t register_data[12];
  // Read all data registers into array
  err = lsm6dso32_read_bytes(priv, OUTX_L_G, &register_data, sizeof(register_data));
  if (err < 0)
    {
      return err;
    }
  raw_data->pitch_raw = register_data[0] | (register_data[1]<<8);
  raw_data->roll_raw = register_data[2] | (register_data[3]<<8);
  raw_data->yaw_raw = register_data[4] | (register_data[5]<<8);
  raw_data->x_accel_raw = register_data[6] | (register_data[7]<<8);
  raw_data->y_accel_raw = register_data[8] | (register_data[9]<<8);
  raw_data->z_accel_raw = register_data[10] | (register_data[11]<<8);
  return 0;
}

/****************************************************************************
 * Name: lsm6dso32_convert_raw_data
 *
 * Description:
 *   Convert raw acceleration and rotational data of the sensor to actual values.
 *
 * Input Parameters:
 *   raw_data    - The structure containing the raw data.
 *   conv_data   - The structure to store the converted data.
 *
 ****************************************************************************/

static void lsm6dso32_convert_raw_data(struct lsm6dso32_data_raw_s *raw_data, struct lsm6dso32_data_s *conv_data)
{
  conv_data->pitch = (raw_data->pitch_raw)*4.375*gyro_fsr_level;
  conv_data->roll = (raw_data->roll_raw)*4.375*gyro_fsr_level;
  conv_data->yaw = (raw_data->yaw_raw)*4.375*gyro_fsr_level;
  conv_data->x_accel = (raw_data->x_accel_raw)*0.122*accel_fsr_level;
  conv_data->y_accel = (raw_data->y_accel_raw)*0.122*accel_fsr_level;
  conv_data->z_accel = (raw_data->z_accel_raw)*0.122*accel_fsr_level;
  return;
}

/****************************************************************************
 * Name: lsm6dso32_open
 *
 * Description:
 *   This function is called whenever the lsm6dso32 device is opened.
 *
 ****************************************************************************/
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_open(FAR struct file *filep)
{
  // TODO: implementation

  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm6dso32_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Increment the count of open references on the driver */

  priv->crefs++;
  DEBUGASSERT(priv->crefs > 0);

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif // CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Name: lsm6dso32_close
 *
 * Description:
 *   This function is called whenever the lsm6dso32 device is closed.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm6dso32_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Decrement the count of open references on the driver */

  DEBUGASSERT(priv->crefs > 0);
  priv->crefs--;

  /* If the count has decremented to zero and the driver has been unlinked,
   * then free memory now.
   */

  if (priv->crefs <= 0 && priv->unlinked)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      return 0;
    }

  nxmutex_unlock(&priv->devlock);
  return 0;
}
#endif // CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Name: lsm6dso32_write
 *
 * Description:
 *     Not implemented.
 ****************************************************************************/

static ssize_t lsm6dso32_write(FAR struct file *filep, FAR const char *buffer,
                               size_t buflen)
{
  // TODO: implementation (is one necessary?)
  return -ENOSYS;
}

/****************************************************************************
 * Name: lsm6dso32_read
 *
 * Description:
 *     Character driver interface to sensor for debugging.
 *
 ****************************************************************************/
static ssize_t lsm6dso32_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm6dso32_dev_s *priv = inode->i_private;
  ssize_t length = 0;
  int err;

  /* If file position is non-zero, then we're at the end of file. */

  if (filep->f_pos > 0)
    {
      return 0;
    }

  /* Get exclusive access */

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxmutex_unlock(&priv->devlock);
      return 0;
    }
#endif
  //Configure Accelerometer and Gyroscope to normal mode (104Hz)
  int8_t control = 0x40;
  err = lsm6dso32_write_bytes(priv, CTRL1_XL, &control, sizeof(control));
  if (err < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return err;
    }
  gyro_fsr_level = 2;
  err = lsm6dso32_write_bytes(priv, CTRL2_G, &control, sizeof(control));
  if (err < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return err;
    }
  accel_fsr_level = 1;
  //read and convert data
  extern struct lsm6dso32_data_raw_s raw_data;
  extern struct lsm6dso32_data_s data;
  lsm6dso32_read_raw_data(priv, &raw_data);
  lsm6dso32_convert_raw_data(&raw_data, &data);
  
  length = snprintf(buffer, buflen, 
  "X: %dmg Y: %dmg Z: %dmg Pitch: %dmd/s Roll: %dmd/s Yaw: %dmd/s\n", 
  data.x_accel, data.y_accel, data.z_accel, data.pitch, data.roll, data.yaw);
  if (length > buflen)
    {
      length = buflen;
    }

  filep->f_pos += length;
  nxmutex_unlock(&priv->devlock);
  return length;
}

static int lsm6dso32_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{

  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm6dso32_dev_s *priv = inode->i_private;
  int err;

  err = nxmutex_lock(&priv->devlock);
  if (err < 0)
    {
      return err;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (priv->unlinked)
    {
      /* Do not allow operations on unlinked sensors. This allows
       * sensor use on hot swappable I2C bus.
       */

      nxmutex_unlock(&priv->devlock);
      return -ENODEV;
    }
#endif

  switch (cmd)
    {

    case SNIOC_WHO_AM_I:
      /* Read WHO_AM_I value into 8-bit unsigned integer buffer */
      err = lsm6dso32_read_bytes(priv, WHO_AM_I, (uint8_t *)(arg),
                                 sizeof(uint8_t));
      break;
    case SNIOC_SETSAMPLERATE:
      if (arg > 11 && arg <= 22){
        err = lsm6dso32_set_bits(priv, CTRL2_G, ODR_BIT_MASKS[arg-12][0],
                                 ODR_BIT_MASKS[arg-12][1]);
      } else if (arg >= 0) {
        err = lsm6dso32_set_bits(priv, CTRL1_XL, ODR_BIT_MASKS[arg][0],
                                 ODR_BIT_MASKS[arg][1]);
      } else {
        err = -EINVAL;
      }
      break;
    case SNIOC_SET_RESOLUTION:
      if (arg > 4 && arg <= 8){
        err = lsm6dso32_set_bits(priv, CTRL1_XL, FSR_BIT_MASKS[arg-5][0],
                                 FSR_BIT_MASKS[arg-5][1]);
        if (err == 0) {
          accel_fsr_level = arg - 4;
        }
      } else if (arg >= 0) {
        err = lsm6dso32_set_bits(priv, CTRL2_G, FSR_BIT_MASKS[arg][0],
                                 FSR_BIT_MASKS[arg][1]);
        if (err == 0) {
          if (arg == 4) {
            gyro_fsr_level = 1;
          } else {
            gyro_fsr_level = arg + 2;
          }
        }
      } else {
        err = -EINVAL;
      }
      break;
    case SNIOC_READ_RAW_DATA:
      err = lsm6dso32_read_raw_data(priv, ((FAR struct lsm6dso32_data_raw_s *)(arg)));
      break;
    case SNIOC_READ_CONVERT_DATA:
      {
        extern struct lsm6dso32_data_raw_s raw_data;
        err = lsm6dso32_read_raw_data(priv, &raw_data);
        lsm6dso32_convert_raw_data(&raw_data, ((FAR struct lsm6dso32_data_s *)(arg)));
      }
      break;
    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&priv->devlock);
  return err;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int lsm6dso32_unlink(FAR struct inode *inode) { return -ENOSYS; }
#endif // CONFIG_DISABLE_PSEUDOFS_OPERATIONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dso32_register
 *
 * Description:
 *   Register the LSM6DSO32 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/imu0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM6DSO32
 *   addr    - The I2C address of the LSM6DSO32.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dso32_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                       uint8_t addr)
{
  FAR struct lsm6dso32_dev_s *priv;
  int err;

  DEBUGASSERT(i2c != NULL);
  // TODO: assert address is one of the valid LSM6DSO32 addresses

  /* Initialize the device structure */

  priv = kmm_zalloc(sizeof(struct lsm6dso32_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance of LSM6DSO32 driver.\n");
      return -ENOMEM;
    }

  priv->i2c = i2c;
  priv->addr = addr;

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      snerr("ERROR: Failed to register LSM6DSO32 driver: %d\n", err);
      kmm_free(priv);
      return err;
    }

  /* Register the character driver */

  err = register_driver(devpath, &g_lsm6dso32fops, 0666, priv);
  if (err < 0)
    {
      snerr("ERROR: Failed to register LSM6DSO32 driver: %d\n", err);
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
    }

  return err;
}

#endif // defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM6DSO32)

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

static ssize_t lsm6dso32_write(FAR struct file *filep, FAR const char *buffer,
                               size_t buflen)
{
  // TODO: implementation (is one necessary?)
  return -ENOSYS;
}

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

  // TODO: implementation of returning some data (probably accel and gyro
  // values)
  // For now returning WHO_AM_I
  uint8_t whoami = 0;
  err = lsm6dso32_read_bytes(priv, WHO_AM_I, &whoami, sizeof(whoami));
  if (err < 0)
    {
      nxmutex_unlock(&priv->devlock);
      return err;
    }

  length = snprintf(buffer, buflen, "WHOAMI: %02x\n", whoami);
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

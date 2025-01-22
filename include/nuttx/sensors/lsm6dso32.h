/****************************************************************************
 * include/nuttx/sensors/lsm6dso32.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s; /* Forward reference */

struct lsm6dso32_data_s
{
  int16_t pitch;
  int16_t roll;
  int16_t yaw;
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
};

struct lsm6dso32_data_raw_s
{
  int16_t pitch_raw;
  int16_t roll_raw;
  int16_t yaw_raw;
  int16_t x_accel_raw;
  int16_t y_accel_raw;
  int16_t z_accel_raw;
};

/* Gyroscope FSRs */

enum lsm6dso32_gyro_fsr_e
{
  LSM6DSO32_GYRO_250DPS = 0x0,  /* +-250dps Precision */
  LSM6DSO32_GYRO_500DPS = 0x2,  /* +-500dps Precision */
  LSM6DSO32_GYRO_1000DPS = 0x4, /* +-1000dps Precision */
  LSM6DSO32_GYRO_2000DPS = 0x6, /* +-2000dps Precision */
  LSM6DSO32_GYRO_125DPS = 0x1,  /* +-125dps Precision */
};

/* Accelerometer FSRs */

enum lsm6dso32_accel_fsr_levels_e
{
  LSM6DSO32_ACCEL_4G = 5,  /* +-4g Precision */
  LSM6DSO32_ACCEL_32G = 6, /* +-32g Precision */
  LSM6DSO32_ACCEL_8G = 7,  /* +-8g Precision. */
  LSM6DSO32_ACCEL_16G = 8, /* +-16g Precision. */
};

/****************************************************************************
 * Public Function Prototypes
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
                       uint8_t addr);

#endif // __INCLUDE_NUTTX_SENSORS_LSM6DSO32_H

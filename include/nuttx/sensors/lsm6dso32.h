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

/* Accelerometer Sample Rate Levels for IOCTL commands. */

enum lsm6dso32_Accel_ODR_levels_e
{
  LSM6DSO32_ACCEL_OFF = 0,  /* Sensor Deactivated */
  LSM6DSO32_ACCEL_12_5HZ = 1, /* 12.5Hz Rate. */
  LSM6DSO32_ACCEL_26HZ = 1, /* 26Hz Rate. */
  LSM6DSO32_ACCEL_52HZ = 3, /* 52Hz Rate. */
  LSM6DSO32_ACCEL_104HZ = 4, /* 104Hz Rate */
  LSM6DSO32_ACCEL_208HZ = 5, /* 208Hz Rate */
  LSM6DSO32_ACCEL_416HZ = 6, /* 416Hz Rate */
  LSM6DSO32_ACCEL_833HZ = 7, /* 833Hz Rate */
  LSM6DSO32_ACCEL_1660HZ = 8, /* 1.66kHz Rate */
  LSM6DSO32_ACCEL_3330HZ = 9, /* 3.33kHz Rate */
  LSM6DSO32_ACCEL_6660HZ = 10, /* 6.66kHz Rate */
  LSM6DSO32_ACCEL_1_6HZ = 11,  /* 1.6Hz Rate */
};

/* Gyroscope Sample Rate Levels for IOCTL commands. */

enum lsm6dso32_Gyro_ODR_levels_e
{
  LSM6DSO32_GYRO_OFF = 12,  /* Sensor Deactivated */
  LSM6DSO32_GYRO_12_5HZ = 13, /* 12.5Hz Rate. */
  LSM6DSO32_GYRO_26HZ = 14, /* 26Hz Rate. */
  LSM6DSO32_GYRO_52HZ = 15, /* 52Hz Rate. */
  LSM6DSO32_GYRO_104HZ = 16, /* 104Hz Rate */
  LSM6DSO32_GYRO_208HZ = 17, /* 208Hz Rate */
  LSM6DSO32_GYRO_416HZ = 18, /* 416Hz Rate */
  LSM6DSO32_GYRO_833HZ = 19, /* 833Hz Rate */
  LSM6DSO32_GYRO_1660HZ = 20, /* 1.66kHz Rate */
  LSM6DSO32_GYRO_3330HZ = 21, /* 3.33kHz Rate */
  LSM6DSO32_GYRO_6660HZ = 22, /* 6.66kHz Rate */
};

/* Gyroscope Resolution Levels for IOCTL commands. */

enum lsm6dso32_Gyro_FSR_levels_e
{
  LSM6DSO32_GYRO_250DPS = 0,  /* +-250dps Precision */
  LSM6DSO32_GYRO_500DPS = 1,  /* +-500dps Precision */
  LSM6DSO32_GYRO_1000DPS = 2, /* +-1000dps Precision */
  LSM6DSO32_GYRO_2000DPS = 3, /* +-2000dps Precision */
  LSM6DSO32_GYRO_125DPS = 4, /* +-125dps Precision */
};

/* Accelerometer Resolution Levels for IOCTL commands. */

enum lsm6dso32_Accel_FSR_levels_e
{
  LSM6DSO32_ACCEL_4g = 5,  /* +-4g Precision */
  LSM6DSO32_ACCEL_32g = 6,  /* +-32g Precision */
  LSM6DSO32_ACCEL_8g = 7, /* +-8g Precision. */
  LSM6DSO32_ACCEL_16g = 8, /* +-16g Precision. */
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
